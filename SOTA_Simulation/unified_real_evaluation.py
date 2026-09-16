"""Real September 10 camera merge: no quality gates, resumable serial stages."""
from __future__ import annotations
import argparse
from collections import Counter
import json
from pathlib import Path
import platform
import subprocess
import cv2
import numpy as np
import scipy
from SOTA_Simulation import real_evaluation as r
from SOTA_Simulation.shah_solver import self_test

ROOT = r.ROOT
SESSIONS = ('cam0-1_02', 'cam1-3_02', 'cam0-3_01')
CAMERAS = ('0', '1', '3')


def usable_observation(raw, name):
    """Membership depends only on finite proper transforms, never quality flags."""
    a = r.transform(raw.get('robot_pose_matrix_4x4'))
    c = raw.get('cams', {}).get(name, {}).get('charuco') or {}
    return a, r.transform(c.get('T_cam_board_4x4'))


def serializable(v):
    if isinstance(v, np.ndarray):
        return serializable(v.tolist())
    if isinstance(v, np.generic):
        return serializable(v.item())
    if isinstance(v, float) and not np.isfinite(v):
        return str(v)
    if isinstance(v, dict):
        return {k: serializable(x) for k, x in v.items()}
    if isinstance(v, (list, tuple)):
        return [serializable(x) for x in v]
    return v


def save(path, v):
    r.write_json(path, serializable(v))


def read_rgb(path):
    # OpenCV imread on Windows can fail on Korean/Unicode directory names.
    return cv2.imdecode(np.frombuffer(Path(path).read_bytes(), dtype=np.uint8), cv2.IMREAD_COLOR)


def prepare():
    records, audit, sources, cameras = [], [], [], {}
    geometry = None
    keys = ('squares_x', 'squares_y', 'square_length_m', 'marker_length_m', 'dictionary_name', 'marker_id_start', 'legacy_pattern')
    for session in SESSIONS:
        path = ROOT / 'datasets/260910' / session / 'meta.json'
        meta = json.loads(path.read_text(encoding='utf-8'))
        cfg = meta['capture_config']
        if (meta.get('simulation') or meta['schema_version'] != 'shah_capture_v1' or 'flange' not in cfg['robot_pose_convention'] or
                cfg['transform_convention'] != 'T_destination_source, translation in metre'):
            raise ValueError('incompatible real capture convention')
        current = {k: meta['board_config'].get(k, False if k == 'legacy_pattern' else None) for k in keys}
        if geometry is not None and geometry != current:
            raise ValueError('different physical board geometry')
        geometry = current
        board = r.board_from_meta(meta['board_config'])
        board.setLegacyPattern(current['legacy_pattern'])
        points = board.getChessboardCorners().astype(float)
        sources.append(dict(session=session, file=str(path.relative_to(ROOT)), sha256=r.sha256(path),
                            raw_events=len(meta['captures']), board=current, capture_config=cfg))
        for i in meta['cam_indices']:
            name, ip = str(i), ROOT / 'intrinsics' / f'cam{i}.npz'
            serial = meta['cam_serials'][name]
            if name in cameras and cameras[name]['serial'] != serial:
                raise ValueError('camera device changed across sessions')
            with np.load(ip, allow_pickle=False) as data:
                if (str(data['serial'].item()) != serial or bool(data['is_gripper'].item()) or
                        (int(data['color_w']), int(data['color_h'])) != (cfg['width'], cfg['height'])):
                    raise ValueError('intrinsic device/role/resolution mismatch')
                k, d = data['color_K'].astype(float), data['color_D'].astype(float)
                if not np.isfinite(k).all() or not np.isfinite(d).all():
                    raise ValueError('invalid intrinsics')
                cameras[name] = dict(K=k, D=d, serial=serial, file=str(ip.relative_to(ROOT)), sha256=r.sha256(ip))
        seen = set()
        for raw in meta['captures']:
            eid = raw['event_id']
            if eid in seen:
                raise ValueError('duplicate event within source session')
            seen.add(eid)
            record = dict(key=f'{session}:{eid}', source_session=session, source_event_id=eid,
                          recorded_at=raw.get('recorded_at', ''), cams={})
            for i in meta['cam_indices']:
                name = str(i)
                cam = raw.get('cams', {}).get(name, {})
                c = cam.get('charuco') or {}
                item = dict(key=record['key'], camera=name, included=False, reason=None,
                    stored_corners=c.get('n_corners'), stored_reprojection_px=c.get('reproj_error_px'),
                    capture_gate=raw.get('capture_gate'), stored_ok=c.get('ok'), foreign_ids=cam.get('foreign_marker_ids', []))
                try:
                    a, b = usable_observation(raw, name)
                    record['robot'] = a
                    record['cams'][name] = dict(visual=b, pixels=np.empty((0, 2)), points=np.empty((0, 3)))
                    item['included'] = True
                except (ValueError, TypeError, KeyError) as exc:
                    item['reason'] = str(exc)
                if item['included']:
                    try:
                        rgb = r.relative_file(path.parent, cam['rgb_path'])
                        image = read_rgb(rgb)
                        if image is None or image.shape[:2] != (cfg['height'], cfg['width']):
                            raise ValueError('unavailable_evaluation_rgb')
                        ids, pixels, source = r.pixels_from_record(c, image, board, 1)
                        record['cams'][name].update(pixels=pixels, points=points[ids])
                        item.update(evaluation_corners=len(ids), corner_source=source,
                                    rgb_file=str(rgb.relative_to(ROOT)), rgb_sha256=r.sha256(rgb))
                    except (ValueError, KeyError, OSError, cv2.error) as exc:
                        item['pixel_error'] = str(exc)
                audit.append(item)
            if record['cams']:
                records.append(record)
        print(f'Loaded {session}', flush=True)
    records.sort(key=lambda x: (x['recorded_at'], x['source_session'], x['source_event_id']))
    for uid, record in enumerate(records):
        record['id'] = uid
    train, held, groups = split_records(records)
    return dict(records=records, audit=audit, sources=sources, cameras=cameras,
        split=dict(policy='Chronological connected-pose groups, every fourth group held out; grouping removes no observations',
            near_translation_mm=1.5, near_rotation_deg=1., train_keys=[x['key'] for x in train], heldout_keys=[x['key'] for x in held],
            groups=[[records[uid]['key'] for uid in group] for group in groups]),
        versions=dict(python=platform.python_version(), opencv=cv2.__version__, numpy=np.__version__, scipy=scipy.__version__),
        git_commit=subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip(),
        implementation_hashes={str(p.relative_to(ROOT)):r.sha256(p) for p in (Path(__file__), ROOT/'SOTA_Simulation/real_evaluation.py', ROOT/'SOTA_Simulation/shah_solver.py', ROOT/'SOTA_Simulation/tsai_combined_demo.py')})


def split_records(records):
    groups = r.pose_groups(records, translation_mm=1.5, rotation_deg=1.)
    held = {uid for i, group in enumerate(groups) if i % 4 == 3 for uid in group}
    return [x for x in records if x['id'] not in held], [x for x in records if x['id'] in held], groups


def restore(data):
    for c in data['cameras'].values():
        c['K'], c['D'] = np.array(c['K']), np.array(c['D'])
    for x in data['records']:
        x['robot'] = np.array(x['robot'])
        for c in x['cams'].values():
            c['visual'] = np.array(c['visual'])
            c['pixels'], c['points'] = np.array(c['pixels']).reshape(-1, 2), np.array(c['points']).reshape(-1, 3)
    return data


def check_sources(data):
    files = [(s['file'], s['sha256']) for s in data['sources']]
    files += [(c['file'], c['sha256']) for c in data['cameras'].values()]
    files += [(a['rgb_file'], a['rgb_sha256']) for a in data['audit'] if 'rgb_file' in a]
    for file, digest in files:
        if r.sha256(ROOT / file) != digest:
            raise ValueError(f'input changed: {file}')


def fit_cameras(records, method):
    fits = {}
    for name in CAMERAS:
        selected = [x for x in records if name in x['cams']]
        try:
            y, mount = r.fit(selected, name, method)
            fits[name] = dict(success=True, count=len(selected), T_base_camera=y.tolist(), T_gripper_board=mount.tolist())
        except (ValueError, cv2.error, np.linalg.LinAlgError, AssertionError) as exc:
            fits[name] = dict(success=False, count=len(selected), error=str(exc))
    return fits


def score(records, cameras, fits, method, split):
    rows = []
    for name in CAMERAS:
        if not fits[name]['success']:
            continue
        selected = [x for x in records if name in x['cams']]
        mapping = {x['id']: x for x in selected}
        for row in r.score(selected, name, cameras[name], np.array(fits[name]['T_base_camera']), np.array(fits[name]['T_gripper_board'])):
            origin = mapping[row['event_id']]
            rows.append(dict(method=method, split=split, key=origin['key'], source_session=origin['source_session'], source_event_id=origin['source_event_id'], **row))
    return rows


def registration(records, fits):
    rows = []
    for x in records:
        names = sorted(x['cams'])
        for i, a in enumerate(names):
            for b in names[i+1:]:
                if fits[a]['success'] and fits[b]['success']:
                    ya, yb = np.array(fits[a]['T_base_camera']), np.array(fits[b]['T_base_camera'])
                    dt, dr = r.errors(r.inv(yb) @ ya @ x['cams'][a]['visual'], x['cams'][b]['visual'])
                    rows.append(dict(key=x['key'], pair=f'{a}-{b}', translation_mm=dt, rotation_deg=dr))
    return rows


def collect(output, data):
    summaries, cameras, events, pairs, full = [], [], [], [], {}
    for method in r.METHODS:
        result = json.loads((output/f'{method}.json').read_text(encoding='utf-8'))
        if result['prepared_sha256'] != r.sha256(output/'prepared.json'):
            raise ValueError('different input population')
        success = all(c['success'] for c in result['train_estimates'].values())
        m = r.aggregate(result['heldout_rows']) if success else None
        regs = result['registration_rows']
        summaries.append(dict(method=method, success=success,
            heldout_translation_mm=m['chain_translation_mm'] if m else None, heldout_rotation_deg=m['chain_rotation_deg'] if m else None,
            camera_pose_translation_error_mm=None, camera_pose_rotation_error_deg=None,
            registration_translation_error_mm=None, registration_rotation_error_deg=None,
            heldout_reprojection_rmse_px=m['reprojection_rmse_px'] if m else None,
            registration_consistency_mm=float(np.mean([x['translation_mm'] for x in regs])) if regs and success else None,
            registration_consistency_deg=float(np.mean([x['rotation_deg'] for x in regs])) if regs and success else None,
            heldout_camera_observations=len(result['heldout_rows']), registration_pair_observations=len(regs)))
        full[method] = result['full_data_estimates']
        for name in CAMERAS:
            obs = [a for a in data['audit'] if a['camera'] == name]
            scores = [x for x in result['heldout_rows'] if x['camera'] == name]
            cameras.append(dict(method=method, camera=name, raw=len(obs), included=sum(a['included'] for a in obs),
                train=result['train_estimates'][name]['count'], heldout=len(scores), success=result['train_estimates'][name]['success'], **(r.aggregate(scores) or {})))
        events += result['heldout_rows'] + result['training_rows']
        pairs += [dict(method=method, **x) for x in regs]
    report = dict(status='review_pending_not_deployed', summary=summaries, sources=data['sources'], split=data['split'],
        versions=data['versions'], git_commit=data['git_commit'], implementation_hashes=data['implementation_hashes'],
        raw_events=sum(s['raw_events'] for s in data['sources']), included_events=len(data['records']),
        counts={name:dict(raw=sum(a['camera']==name for a in data['audit']), included=sum(a['camera']==name and a['included'] for a in data['audit'])) for name in CAMERAS},
        excluded_reasons=dict(Counter(a['reason'] for a in data['audit'] if not a['included'])),
        definitions=dict(heldout='Mean held-out board chain residual FK*train mount vs train camera*stored PnP; internal consistency, not GT',
            camera_pose='Independent extrinsic GT error unavailable: null', registration='Independent relative-camera GT error unavailable: null',
            reprojection='sqrt(sum(du^2+dv^2)/N corners), all held-out cameras pooled; null if any observation unscorable',
            registration_consistency='Same-event board pose transfer i->j with train extrinsics vs j PnP; auxiliary internal closure, not GT accuracy'),
        assumptions=['Requested merge assumes unchanged camera mounts and flange-board attachment; not independently proven by metadata.',
            'Documented flange robot frame used; per-event tool null.', 'Intrinsics match serial/resolution; capture-time intrinsic hashes unavailable.',
            'Stored PnP unchanged. Pixels re-detected for evaluation only. No quality gates, synthetic noise, robust loss or outlier removal.'])
    save(output/'analysis.json', report)
    save(output/'full_data_estimates.json', dict(status='review_pending_not_deployed', note='One cam0/1/3 set per method, all usable poses; separate from held-out training estimates.', methods=full))
    r.write_csv(output/'summary.csv', summaries)
    r.write_csv(output/'per_camera.csv', cameras)
    r.write_csv(output/'event_metrics.csv', events)
    r.write_csv(output/'registration_pairs.csv', pairs)
    print(json.dumps(dict(counts=report['counts'], summary=summaries), indent=2), flush=True)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--method', choices=('prepare', 'collect') + r.METHODS, required=True)
    args = parser.parse_args(argv)
    output = args.output.resolve()
    cv2.setNumThreads(1)
    if args.method == 'prepare':
        if output.exists():
            raise ValueError('previous output exists; select a new directory')
        self_test()
        data = prepare()
        check_sources(data)
        output.mkdir(parents=True, exist_ok=False)
        save(output/'prepared.json', data)
        save(output/'audit.json', data['audit'])
        save(output/'split.json', data['split'])
        return
    data = restore(json.loads((output/'prepared.json').read_text(encoding='utf-8')))
    check_sources(data)
    if args.method == 'collect':
        collect(output, data)
        return
    path = output/f'{args.method}.json'
    if path.exists():
        raise ValueError('method already calculated; refuses overwrite')
    self_test()
    keys = set(data['split']['train_keys'])
    records = data['records']
    train, held = [x for x in records if x['key'] in keys], [x for x in records if x['key'] not in keys]
    print(f'Fitting {args.method}: cameras sequentially, one CPU thread', flush=True)
    fits = fit_cameras(train, args.method)
    result = dict(method=args.method, prepared_sha256=r.sha256(output/'prepared.json'), train_estimates=fits,
        full_data_estimates=fit_cameras(records, args.method), heldout_rows=score(held, data['cameras'], fits, args.method, 'heldout'),
        training_rows=score(train, data['cameras'], fits, args.method, 'train'), registration_rows=registration(held, fits))
    check_sources(data)
    save(path, result)
    print(f'Saved {path}', flush=True)


if __name__ == '__main__':
    main()
