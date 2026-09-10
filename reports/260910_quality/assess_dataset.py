"""Read-only assessment of the recorded 260910 dataset. Outputs stay in reports."""
from pathlib import Path
import collections
import hashlib
import json
import sys
import numpy as np
import cv2
from scipy.spatial.transform import Rotation
from PIL import Image, ImageDraw

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from SOTA_Simulation.shah_solver import solve_shah_eye_to_hand, self_test

DATA = ROOT / 'datasets/260910'
OUT = Path(__file__).resolve().parent


def inv(T):
    x = np.eye(4); x[:3,:3] = T[:3,:3].T; x[:3,3] = -T[:3,:3].T@T[:3,3]
    return x


def err(a,b):
    return [float(np.linalg.norm(a[:3,3]-b[:3,3])*1000),
            float(np.degrees(Rotation.from_matrix(a[:3,:3].T@b[:3,:3]).magnitude()))]


def avg(ts):
    t=np.eye(4); t[:3,:3]=Rotation.from_matrix([x[:3,:3] for x in ts]).mean().as_matrix()
    t[:3,3]=np.median([x[:3,3] for x in ts],axis=0)
    return t


def dist(vals):
    vals=np.asarray(vals,float).ravel(); vals=vals[np.isfinite(vals)]
    if not len(vals): return None
    return dict(zip(('min','median','p90','max','mean'),
                    [float(v) for v in (*np.percentile(vals,[0,50,90,100]),np.mean(vals))]))


def edist(vals):
    vals=np.asarray(vals,float)
    return {'translation_mm':dist(vals[:,0]),'rotation_deg':dist(vals[:,1])}


def validT(value):
    t=np.asarray(value,float)
    return (t.shape==(4,4) and np.isfinite(t).all() and
            np.allclose(t[3],[0,0,0,1],atol=1e-7) and
            np.linalg.norm(t[:3,:3].T@t[:3,:3]-np.eye(3))<1e-4 and np.linalg.det(t[:3,:3])>0.9999)


def cv_valid(entry, cfg):
    c=entry.get('charuco',{}); px=c.get('reproj_error_px')
    return bool(entry.get('saved') and c.get('ok') and
                validT(c.get('T_cam_board_4x4')) and
                px is not None and np.isfinite(px) and px<=cfg['max_charuco_reproj_px'] and
                c.get('n_corners',0)>=cfg['min_charuco_corners'])


def fit(A,B,method='SHAH'):
    r=solve_shah_eye_to_hand(A,B,method)
    if not validT(r.T_gripper_board) or not validT(r.T_base_fixed_i):
        raise ValueError('Nonfinite/invalid solver result')
    return r.T_gripper_board, r.T_base_fixed_i


def validation(A,B,ids):
    X,Y=fit(A,B)
    errors=[err(a@X,Y@b) for a,b in zip(A,B)]
    loo=[]; loo_fits=[]; failures=[]
    for i,eid in enumerate(ids):
        keep=[j for j in range(len(A)) if j!=i]
        try:
            x,y=fit([A[j] for j in keep],[B[j] for j in keep])
            loo.append({'event_id':eid,'translation_mm':err(A[i]@x,y@B[i])[0],
                        'rotation_deg':err(A[i]@x,y@B[i])[1]})
            loo_fits.append(err(Y,y))
        except Exception as e: failures.append({'event_id':eid,'error':str(e)})
    rng=np.random.default_rng(260910)
    sample_errors=[]; bfail=0
    for _ in range(100):
        ix=rng.choice(len(A),size=max(4,int(len(A)*0.8)),replace=False)
        try:
            x,y=fit([A[i] for i in ix],[B[i] for i in ix])
            sample_errors.append(err(Y,y))
        except Exception: bfail+=1
    li={}
    try:
        lx,ly=fit(A,B,'LI')
        li={'camera_vs_shah':err(Y,ly),'fit':edist([err(a@lx,ly@b) for a,b in zip(A,B)])}
    except Exception as e: li={'error':str(e)}
    return {'T_gripper_board':X.tolist(),'T_base_camera':Y.tolist(),
            'training':edist(errors),'loo':edist([[v['translation_mm'],v['rotation_deg']] for v in loo]),
            'loo_per_event':loo,'loo_camera_change':edist(loo_fits),'loo_failures':failures,
            'subsample_80pct_100runs_change_from_full':edist(sample_errors),
            'subsample_failures':bfail,'li_comparison':li}


def diversity(A):
    angles=[]; diffs=[]; rotvecs=[]
    for i in range(len(A)):
        for j in range(i):
            r=A[j][:3,:3].T@A[i][:3,:3]
            rv=Rotation.from_matrix(r).as_rotvec()
            angles.append(np.degrees(np.linalg.norm(rv)))
            diffs.append(r-np.eye(3));rotvecs.append(rv)
    s=np.linalg.svd(np.concatenate(diffs),compute_uv=False)
    rv_s=np.linalg.svd(np.asarray(rotvecs),compute_uv=False)
    xyz=np.asarray([a[:3,3] for a in A])*1000
    # Count near duplicates against already retained poses (2 mm, 2 degrees).
    unique=[]
    for a in A:
        if not any((lambda e:e[0]<2 and e[1]<2)(err(a,b)) for b in unique):unique.append(a)
    return {'pair_rotation_deg':dist(angles),'xyz_range_mm':np.ptp(xyz,axis=0).tolist(),
            'rotation_difference_singular_values':s.tolist(),
            'rotation_difference_condition':float(s[0]/s[-1]),
            'rotation_vector_singular_values':rv_s.tolist(),'unique_2mm_2deg':len(unique)}


def contact_sheet(session,m,ids):
    cams=m['cam_indices']; tiles=[]
    for eid in ids:
        cap=next(c for c in m['captures'] if c['event_id']==eid)
        for cam in cams:
            c=cap['cams'][str(cam)]; im=cv2.imread(str(session/c['rgb_path']))
            if im is None:continue
            im=cv2.cvtColor(cv2.resize(im,(426,240)),cv2.COLOR_BGR2RGB)
            tile=Image.new('RGB',(426,270),'white');tile.paste(Image.fromarray(im),(0,30))
            cc=c.get('charuco',{});px=cc.get('reproj_error_px')
            txt=f"{session.name} e{eid:02d} cam{cam} n={cc.get('n_corners')} px={px:.2f}" if px is not None else f"{session.name} e{eid:02d} cam{cam} n={cc.get('n_corners')} no pose"
            ImageDraw.Draw(tile).text((4,8),txt,fill='black');tiles.append(tile)
    cols=4;rows=(len(tiles)+cols-1)//cols
    sheet=Image.new('RGB',(426*cols,270*rows),'#bbb')
    for n,t in enumerate(tiles):sheet.paste(t,((n%cols)*426,(n//cols)*270))
    sheet.save(OUT/(session.name+'_contact.jpg'))


def main():
    assert self_test(verbose=False), 'Solver convention self test failed'
    result={'criteria':{'min_corners':12,'max_reproj_px':1.5,'finite_SE3_required':True},'sessions':{}}
    all_relative={}
    for path in sorted(DATA.rglob('meta.json')):
        m=json.loads(path.read_text());session=path.parent;cs=m['captures'];cams=m['cam_indices'];cfg=m['capture_config']
        eligible=[c for c in cs if validT(c['robot_pose_matrix_4x4']) and all(cv_valid(c['cams'][str(k)],cfg) for k in cams)]
        ids=[c['event_id'] for c in eligible]
        summary={'source':str(path),'source_sha256':hashlib.sha256(path.read_bytes()).hexdigest(),
                 'n':len(cs),'stored_gate_pass':sum(c['capture_gate']['pass'] for c in cs),
                 'eligible_count':len(ids),'eligible_ids':ids,'rejected_ids':[c['event_id'] for c in cs if c not in eligible],
                 'board':m['board_config'],'serials':m['cam_serials'],'cameras':{},'missing_files':[],
                 'metadata_stored_root':m['root_folder'],'capture_span_ms':dist([c['capture_span_ms'] for c in cs]),
                 'cross_camera_receipt_span_ms':dist([np.ptp([c['cams'][str(k)]['host_monotonic_ts_ms'] for k in cams]) for c in cs])}
        for c in cs:
            for cam in cams:
                for key in ('rgb_path','depth_path'):
                    value=c['cams'][str(cam)].get(key)
                    if value and not (session/value).is_file():summary['missing_files'].append(value)
        A=[np.asarray(c['robot_pose_matrix_4x4'],float) for c in eligible]
        summary['diversity']=diversity(A)
        Bs={}
        for cam in cams:
            entries=[c['cams'][str(cam)] for c in cs]
            nonfinite=[c['event_id'] for c in cs if c['cams'][str(cam)].get('charuco',{}).get('ok') and not validT(c['cams'][str(cam)]['charuco'].get('T_cam_board_4x4'))]
            good=[c for c in cs if cv_valid(c['cams'][str(cam)],cfg)]
            B=[np.asarray(c['cams'][str(cam)]['charuco']['T_cam_board_4x4'],float) for c in eligible];Bs[cam]=B
            tilt=[np.degrees(np.arccos(np.clip(abs(b[2,2]),0,1))) for b in B]
            stats={'camera_eligible_count':len(good),'nonfinite_pose_ids':nonfinite,
                   'corners_all':dist([e.get('charuco',{}).get('n_corners',0) for e in entries]),
                   'reprojection_px_all_finite':dist([e.get('charuco',{}).get('reproj_error_px',np.nan) for e in entries]),
                   'reprojection_px_pair_eligible':dist([c['cams'][str(cam)]['charuco']['reproj_error_px'] for c in eligible]),
                   'board_tilt_deg_eligible':dist(tilt),
                   'failure_reasons':dict(collections.Counter((e.get('gate_reason') or e.get('skip_reason') or 'pass').split('(')[0] for e in entries)),
                   'calibration':validation(A,B,ids)}
            summary['cameras'][str(cam)]=stats
        i,j=cams
        rel=[a@inv(b) for a,b in zip(Bs[i],Bs[j])]
        center=avg(rel)
        errors=[err(center,t) for t in rel]
        summary['relative_camera']={'convention':f'T_cam{i}_cam{j}', 'transform':center.tolist(),
                                    'scatter':edist(errors),'per_event':[dict(event_id=e,translation_mm=x[0],rotation_deg=x[1]) for e,x in zip(ids,errors)]}
        xi=np.array(summary['cameras'][str(i)]['calibration']['T_gripper_board'])
        xj=np.array(summary['cameras'][str(j)]['calibration']['T_gripper_board'])
        summary['camera_board_offset_disagreement']=err(xi,xj)
        yi=np.array(summary['cameras'][str(i)]['calibration']['T_base_camera'])
        yj=np.array(summary['cameras'][str(j)]['calibration']['T_base_camera'])
        summary['relative_visual_vs_robot_calibration']=err(center,inv(yi)@yj)
        result['sessions'][session.name]=summary;all_relative[(i,j)]=center
        # Show diverse examples plus high error and failed data.
        candidates=[cs[0]['event_id'],cs[len(cs)//2]['event_id'],cs[-1]['event_id']]
        worst=sorted(cs,key=lambda c:max([float(c['cams'][str(k)].get('charuco',{}).get('reproj_error_px') or 0) for k in cams]),reverse=True)
        candidates += [c['event_id'] for c in worst[:2]]
        candidates += summary['rejected_ids'][:2]
        contact_sheet(session,m,list(dict.fromkeys(candidates))[:6])
        print(session.name,'eligible',len(ids),'of',len(cs),'relative scatter',summary['relative_camera']['scatter'],flush=True)
        print('  X disagreement mm/deg',summary['camera_board_offset_disagreement'],flush=True)
        for k,v in summary['cameras'].items():print(' ',k,'LOO',v['calibration']['loo'],flush=True)
    if all(k in all_relative for k in ((0,1),(1,3),(0,3))):
        result['cross_session_loop_closure']=dict(zip(('translation_mm','rotation_deg'),err(all_relative[(0,3)],all_relative[(0,1)]@all_relative[(1,3)])))
    (OUT/'assessment.json').write_text(json.dumps(result,indent=2,allow_nan=False))
    print('loop',result.get('cross_session_loop_closure'),flush=True)

if __name__=='__main__':main()
