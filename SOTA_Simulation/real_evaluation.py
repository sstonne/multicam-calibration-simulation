"""Offline, session-isolated evaluation of recorded fixed-camera calibrations.

Stored PnP poses are solver inputs. Re-detected pixels, when necessary, are only
evaluation observations. No synthetic noise, hardware access, or GT is involved.
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import platform
import subprocess
import sys
from itertools import combinations
from pathlib import Path

import cv2
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import scipy
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
from SOTA_Simulation.shah_solver import solve_shah_eye_to_hand
from SOTA_Simulation.tsai_combined_demo import solve_hand_eye

METHODS = ("shah", "tsai", "park", "horaud", "andreff", "daniilidis")


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def write_json(path, value):
    Path(path).write_text(json.dumps(value, indent=2, ensure_ascii=False,
                                     allow_nan=False) + "\n", encoding="utf-8")


def transform(value):
    t = np.asarray(value, dtype=float)
    if t.shape != (4, 4) or not np.isfinite(t).all():
        raise ValueError("nonfinite_or_wrong_shape_transform")
    if not np.allclose(t[3], [0, 0, 0, 1], atol=1e-7):
        raise ValueError("invalid_homogeneous_row")
    if (not np.allclose(t[:3, :3].T @ t[:3, :3], np.eye(3), atol=1e-5)
            or abs(np.linalg.det(t[:3, :3]) - 1) > 1e-5):
        raise ValueError("invalid_SO3")
    return t


def inv(t):
    result = np.eye(4)
    result[:3, :3] = t[:3, :3].T
    result[:3, 3] = -t[:3, :3].T @ t[:3, 3]
    return result


def errors(a, b):
    return (float(1000 * np.linalg.norm(a[:3, 3] - b[:3, 3])),
            float(np.degrees(Rotation.from_matrix(a[:3, :3].T @ b[:3, :3]).magnitude())))


def average(ts):
    t = np.eye(4)
    t[:3, :3] = Rotation.from_matrix(np.stack([x[:3, :3] for x in ts])).mean().as_matrix()
    t[:3, 3] = np.mean([x[:3, 3] for x in ts], axis=0)
    return t


def relative_file(base, name):
    path = (base / name).resolve()
    if not path.is_relative_to(base.resolve()):
        raise ValueError("reference_outside_session")
    return path


def board_from_meta(config):
    dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, config["dictionary_name"]))
    nx, ny = config["squares_x"], config["squares_y"]
    start = config["marker_id_start"]
    ids = np.arange(start, start + nx * ny // 2, dtype=np.int32).reshape(-1, 1)
    return cv2.aruco.CharucoBoard((nx, ny), config["square_length_m"],
                                 config["marker_length_m"], dictionary, ids)


def pixels_from_record(charuco, image, board, scale):
    if "corner_ids" in charuco and "corners_px" in charuco:
        if charuco.get("corner_coordinate_space", "original_image_pixels") != "original_image_pixels":
            raise ValueError("unknown_corner_coordinate_space")
        ids = np.asarray(charuco["corner_ids"], dtype=int).reshape(-1)
        pixels = np.asarray(charuco["corners_px"], dtype=float).reshape(-1, 2)
        source = "stored"
    else:
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        if scale != 1:
            gray = cv2.resize(gray, None, fx=scale, fy=scale, interpolation=cv2.INTER_CUBIC)
        corners, found_ids, _, _ = cv2.aruco.CharucoDetector(board).detectBoard(gray)
        if corners is None:
            return np.empty(0, int), np.empty((0, 2)), "redetection_failed"
        ids = found_ids.reshape(-1)
        pixels = corners.reshape(-1, 2).astype(float)
        if scale != 1:
            pixels = (pixels + .5) / scale - .5
        source = "redetected_rgb_evaluation_only"
    count = len(board.getChessboardCorners())
    if (len(ids) != len(pixels) or len(set(ids.tolist())) != len(ids)
            or not np.isfinite(pixels).all() or np.any(ids < 0) or np.any(ids >= count)):
        raise ValueError("invalid_corner_observations")
    return ids, pixels, source


def project(t, points, camera):
    transformed = points @ t[:3, :3].T + t[:3, 3]
    if not np.isfinite(transformed).all() or np.any(transformed[:, 2] <= 0):
        raise ValueError("predicted_corners_behind_camera")
    pixels, _ = cv2.projectPoints(points.astype(float), cv2.Rodrigues(t[:3, :3])[0],
                                 t[:3, 3], camera["K"], camera["D"])
    return pixels.reshape(-1, 2)


def load_session(spec, intrinsics_dir):
    path = ROOT / spec["meta"]
    meta = json.loads(path.read_text(encoding="utf-8"))
    if meta.get("simulation") or meta.get("schema_version") != "shah_capture_v1":
        raise ValueError("expected_real_shah_capture_v1")
    if meta["cam_indices"] != spec["cameras"]:
        raise ValueError("camera_manifest_mismatch")
    cfg = meta["capture_config"]
    if cfg["transform_convention"] != "T_destination_source, translation in metre":
        raise ValueError("unknown_transform_convention")
    if "flange" not in cfg["robot_pose_convention"]:
        raise ValueError("missing_flange_convention")
    board = board_from_meta(meta["board_config"])
    points = board.getChessboardCorners().astype(float)
    cameras, provenance = {}, {}
    for index in spec["cameras"]:
        name = str(index)
        intrinsic_path = intrinsics_dir / f"cam{index}.npz"
        with np.load(intrinsic_path, allow_pickle=False) as data:
            if str(data["serial"].item()) != meta["cam_serials"][name]:
                raise ValueError("intrinsic_serial_mismatch")
            if bool(data["is_gripper"].item()):
                raise ValueError("unexpected_wrist_camera")
            if (int(data["color_w"]), int(data["color_h"])) != (cfg["width"], cfg["height"]):
                raise ValueError("intrinsic_resolution_mismatch")
            k, d = data["color_K"].astype(float), data["color_D"].astype(float)
            if not np.isfinite(k).all() or not np.isfinite(d).all():
                raise ValueError("nonfinite_intrinsics")
            cameras[name] = dict(K=k, D=d)
            provenance[name] = dict(file=intrinsic_path.name, sha256=sha256(intrinsic_path),
                                    serial=meta["cam_serials"][name], K=k.tolist(), D=d.tolist())
    records, audit, observations = [], [], []
    ids_seen = set()
    for raw in meta["captures"]:
        eid = raw["event_id"]
        if eid in ids_seen:
            raise ValueError("duplicate_event_id")
        ids_seen.add(eid)
        item = dict(event_id=eid, accepted=False, reasons=[], cameras={},
                    foreign_marker_ids={name: raw.get("cams", {}).get(name, {}).get(
                        "charuco", {}).get("foreign_marker_ids", []) for name in cameras})
        try:
            robot = transform(raw["robot_pose_matrix_4x4"])
            if raw.get("capture_block") != "B_eyetohand":
                raise ValueError("not_eye_to_hand")
            if raw.get("tool") not in (None, 1):
                raise ValueError("not_flange_tool1")
            if raw.get("capture_gate", {}).get("pass") is not True:
                raise ValueError("stored_capture_gate_failed")
            if raw.get("eligible_for_calibration") is False:
                raise ValueError("marked_ineligible")
            pose6 = raw.get("robot_pose_6dof")
            if pose6 is not None:
                expected = np.eye(4)
                expected[:3, 3] = np.asarray(pose6[:3]) / 1000
                expected[:3, :3] = Rotation.from_euler("ZYX", pose6[3:], degrees=True).as_matrix()
                if not np.allclose(expected, robot, atol=1e-6):
                    raise ValueError("robot_pose_unit_or_convention_mismatch")
            cams, times = {}, []
            for name in cameras:
                cam = raw["cams"][name]
                c = cam["charuco"]
                if cam.get("is_gripper") or not cam.get("saved") or not c.get("ok"):
                    raise ValueError(f"cam{name}_invalid_detection")
                if (c["n_corners"] < max(4, cfg["min_charuco_corners"])
                        or c.get("reproj_error_px") is None
                        or not np.isfinite(c["reproj_error_px"])
                        or c["reproj_error_px"] > cfg["max_charuco_reproj_px"]):
                    raise ValueError(f"cam{name}_stored_quality_failed")
                if c.get("foreign_marker_ids") or cam.get("foreign_marker_ids"):
                    raise ValueError(f"cam{name}_foreign_marker_ids")
                visual = transform(c["T_cam_board_4x4"])
                if np.any((points @ visual[:3, :3].T + visual[:3, 3])[:, 2] <= 0):
                    raise ValueError(f"cam{name}_nonpositive_depth")
                rgb = relative_file(path.parent, cam["rgb_path"])
                image = cv2.imread(str(rgb))
                if image is None or image.shape[:2] != (cfg["height"], cfg["width"]):
                    raise ValueError(f"cam{name}_missing_or_invalid_rgb")
                scale = c.get("detection_scale", cfg.get("detection_scale", 1))
                corner_ids, pixels, source = pixels_from_record(c, image, board, scale)
                obj = points[corner_ids]
                local_rmse = None
                if len(pixels):
                    local_rmse = float(np.sqrt(np.mean(np.sum(
                        (project(visual, obj, cameras[name]) - pixels) ** 2, axis=1))))
                cams[name] = dict(visual=visual, pixels=pixels, points=obj, rgb=rgb)
                item["cameras"][name] = dict(stored_corners=c["n_corners"],
                    stored_pnp_reprojection_px=c["reproj_error_px"],
                    evaluation_corners=len(pixels), corner_source=source,
                    stored_pose_vs_evaluation_pixels_rmse_px=local_rmse,
                    rgb_sha256=sha256(rgb))
                observations.append(dict(event_id=eid, camera=name, corner_ids=corner_ids.tolist(),
                    corners_px=pixels.tolist(), source=source, detection_scale=scale))
                if cam.get("host_monotonic_ts_ms") is not None:
                    times.append(cam["host_monotonic_ts_ms"])
            span = float(max(times) - min(times)) if len(times) == len(cameras) else None
            item.update(accepted=True, tool_reference_verified=raw.get("tool") == 1,
                        camera_receipt_span_ms=span, capture_span_ms=raw.get("capture_span_ms"))
            records.append(dict(id=eid, robot=robot, cams=cams))
        except (KeyError, TypeError, ValueError, cv2.error) as exc:
            item["reasons"].append(str(exc))
        audit.append(item)
    return meta, cameras, sorted(records, key=lambda r: r["id"]), audit, observations, provenance


def pose_groups(records, translation_mm=1.5, rotation_deg=1.):
    # Connected components keep repeated/nearly identical poses on one side.
    parent = list(range(len(records)))
    def root(i):
        while parent[i] != i:
            parent[i] = parent[parent[i]]
            i = parent[i]
        return i
    for i, j in combinations(range(len(records)), 2):
        dt, dr = errors(records[i]["robot"], records[j]["robot"])
        if dt < translation_mm and dr < rotation_deg:
            parent[root(j)] = root(i)
    groups = {}
    for i, record in enumerate(records):
        groups.setdefault(root(i), []).append(record["id"])
    return sorted(groups.values(), key=min)


def make_split(records, settings):
    groups = pose_groups(records, settings["near_translation_mm"], settings["near_rotation_deg"])
    if len(groups) < 4:
        raise ValueError("fewer_than_four_distinct_pose_groups")
    held = {eid for i, group in enumerate(groups) if i % 4 == 3 for eid in group}
    train = [r for r in records if r["id"] not in held]
    test = [r for r in records if r["id"] in held]
    if len(train) < 3 or not test:
        raise ValueError("insufficient_split")
    return train, test, groups


def diversity(records):
    if len(records) < 2:
        return {"count": len(records)}
    pairs = [errors(a["robot"], b["robot"]) for a, b in combinations(records, 2)]
    rotvecs = np.array([Rotation.from_matrix(records[0]["robot"][:3, :3].T @
                        r["robot"][:3, :3]).as_rotvec() for r in records])
    singular = np.linalg.svd(np.degrees(rotvecs - rotvecs.mean(axis=0)), compute_uv=False)
    return dict(count=len(records), max_pair_translation_mm=max(x[0] for x in pairs),
                mean_pair_rotation_deg=float(np.mean([x[1] for x in pairs])),
                max_pair_rotation_deg=max(x[1] for x in pairs),
                centered_rotation_singular_values_deg=singular.tolist())


def fit(records, name, method):
    robot = [r["robot"] for r in records]
    visual = [r["cams"][name]["visual"] for r in records]
    if len(robot) < 3:
        raise ValueError("fewer_than_three_poses")
    if method == "tsai":
        # OpenCV 4.x Tsai skips pairs outside 0.3 <= 2*sin(theta/2) <= 1.7.
        # With <2 remaining pairs it logs an error but returns identity/zero.
        # Detect this failure instead of treating that finite matrix as a fit.
        # https://github.com/opencv/opencv/blob/4.x/modules/calib3d/src/calibration_handeye.cpp
        informative = 0
        for i, j in combinations(range(len(robot)), 2):
            magnitudes = [2 * np.sin(Rotation.from_matrix(
                values[i][:3, :3].T @ values[j][:3, :3]).magnitude() / 2)
                for values in (robot, visual)]
            informative += int(all(.3 <= value <= 1.7 for value in magnitudes))
        if informative < 2:
            raise ValueError(f"tsai_insufficient_informative_motion_pairs:{informative}")
    if method == "shah":
        result = solve_shah_eye_to_hand(robot, visual)
        y, x = result.T_base_fixed_i, result.T_gripper_board
    else:
        y = solve_hand_eye(robot, visual, eye_to_hand=True, method=method)
        # Nuisance mount estimated ONLY on fitting events, never on held-out data.
        y = transform(y)
        x = average([inv(a) @ y @ b for a, b in zip(robot, visual)])
    return transform(y), transform(x)


def score(records, name, camera, y, x):
    rows = []
    for record in records:
        cam = record["cams"][name]
        dt, dr = errors(record["robot"] @ x, y @ cam["visual"])
        row = dict(event_id=record["id"], camera=name, chain_translation_mm=dt,
                   chain_rotation_deg=dr, corner_count=len(cam["pixels"]),
                   pixel_sse=None, reprojection_rmse_px=None, pixel_failure=None)
        if len(cam["pixels"]):
            try:
                prediction = project(inv(y) @ record["robot"] @ x, cam["points"], camera)
                row["pixel_sse"] = float(np.sum((prediction - cam["pixels"]) ** 2))
                row["reprojection_rmse_px"] = float(np.sqrt(row["pixel_sse"] / len(prediction)))
            except (ValueError, cv2.error) as exc:
                row["pixel_failure"] = str(exc)
        rows.append(row)
    return rows


def aggregate(rows):
    if not rows:
        return None
    available = [r for r in rows if r["pixel_sse"] is not None]
    count = sum(r["corner_count"] for r in available)
    complete = len(available) == len(rows)
    return dict(chain_translation_mm=float(np.mean([r["chain_translation_mm"] for r in rows])),
                chain_rotation_deg=float(np.mean([r["chain_rotation_deg"] for r in rows])),
                reprojection_rmse_px=float(np.sqrt(sum(r["pixel_sse"] for r in available) / count))
                    if count and complete else None,
                reprojection_complete=complete, scored_event_camera_pairs=len(rows),
                pixel_event_camera_pairs=len(available), scored_corners=count)


def sensitivity(train, groups, name, method, baseline):
    entries = []
    train_ids = {r["id"] for r in train}
    for group in groups:
        removed = train_ids.intersection(group)
        if not removed:
            continue
        subset = [r for r in train if r["id"] not in removed]
        item = dict(omitted_event_ids=sorted(removed))
        try:
            y, _ = fit(subset, name, method)
            dt, dr = errors(y, baseline)
            item.update(success=True, translation_change_mm=dt, rotation_change_deg=dr)
        except (ValueError, cv2.error, np.linalg.LinAlgError, AssertionError) as exc:
            item.update(success=False, error=str(exc))
        entries.append(item)
    ok = [e for e in entries if e["success"]]
    return dict(note="Training-only leave-one-pose-group-out change; not a confidence interval.",
                trials=entries, failures=len(entries)-len(ok),
                max_translation_change_mm=max((e["translation_change_mm"] for e in ok), default=None),
                max_rotation_change_deg=max((e["rotation_change_deg"] for e in ok), default=None))


def evaluate_session(spec, intrinsics_dir, split_settings, output):
    meta, cameras, records, audit, observations, provenance = load_session(spec, intrinsics_dir)
    output.mkdir(parents=True, exist_ok=False)
    write_json(output / "audit.json", audit)
    write_json(output / "observations.json", observations)
    warnings = ["No independent camera-pose ground truth; consistency is not absolute accuracy.",
                "Do not merge extrinsics across sessions without confirming unchanged camera mounting.",
                "Intrinsics match serial/resolution; captures did not store an intrinsic hash."]
    if any(not a["accepted"] for a in audit):
        warnings.append("Conservative shared-event filtering applied to both cameras; see audit.json. Foreign IDs indicate possible contamination, not proven bad poses.")
    if any(o["source"] != "stored" for o in observations):
        warnings.append("Missing original pixels were re-detected from compressed RGB for evaluation only. Detector/version and JPEG differences can change corner counts.")
    if any(a.get("accepted") and not a.get("tool_reference_verified") for a in audit):
        warnings.append("Tool is null: flange/tool1 convention follows session documentation, not per-frame verification.")
    if "추정" in meta["board_config"].get("note", ""):
        warnings.append("Board marker ID note was unconfirmed at capture time; verify physical board identity.")
    if len(records) < 10:
        warnings.append("Fewer than 10 selected poses; limited evaluation support.")
    report = dict(session=spec, source_sha256=sha256(ROOT / spec["meta"]),
                  cameras=provenance, capture_config=meta["capture_config"],
                  board_config=meta["board_config"], warnings=warnings,
                  raw_count=len(meta["captures"]), accepted_count=len(records),
                  excluded_count=len(audit)-len(records), all_pose_diversity=diversity(records), methods={})
    train, test, groups = make_split(records, split_settings)
    report["split"] = dict(settings=split_settings, pose_groups=groups,
                          train=[r["id"] for r in train], heldout=[r["id"] for r in test])
    report["train_pose_diversity"] = diversity(train)
    if report["train_pose_diversity"].get("mean_pair_rotation_deg", 0) < 10:
        warnings.append("Small training rotations: inspect subset sensitivity and solver failures.")
    write_json(output / "split.json", report["split"])
    summaries, event_rows, final_fits = [], [], {}
    for method in METHODS:
        method_report = dict(cameras={})
        all_test_rows, boards = [], []
        final_fits[method] = {}
        for name, camera in cameras.items():
            camera_report = {}
            try:
                y, x = fit(train, name, method)
                rows = score(test, name, camera, y, x)
                train_rows = score(train, name, camera, y, x)
                camera_report.update(success=True, T_base_camera=y.tolist(), T_gripper_board=x.tolist(),
                    heldout=aggregate(rows), training=aggregate(train_rows),
                    subset_sensitivity=sensitivity(train, groups, name, method, y))
                all_test_rows.extend(rows)
                boards.append(x)
                event_rows.extend(dict(method=method, split="heldout", **r) for r in rows)
                event_rows.extend(dict(method=method, split="train", **r) for r in train_rows)
                if method == "shah" and rows and rows[0]["pixel_sse"] is not None:
                    overlay(test[0], name, camera, y, x, output / f"shah_cam{name}_heldout.png")
            except (ValueError, cv2.error, np.linalg.LinAlgError, AssertionError) as exc:
                camera_report.update(success=False, error=str(exc))
            method_report["cameras"][name] = camera_report
            # Full-data fits are separate from all held-out metrics and not deployed.
            try:
                y_all, x_all = fit(records, name, method)
                final_fits[method][name] = dict(success=True, T_base_camera=y_all.tolist(),
                                               T_gripper_board=x_all.tolist())
            except (ValueError, cv2.error, np.linalg.LinAlgError, AssertionError) as exc:
                final_fits[method][name] = dict(success=False, error=str(exc))
        success = all(c["success"] for c in method_report["cameras"].values())
        metrics = aggregate(all_test_rows) if success else None
        method_report.update(success=success, heldout=metrics)
        if success:
            spread = [errors(x, average(boards)) for x in boards]
            method_report["mount_spread"] = dict(translation_mm=float(np.mean([p[0] for p in spread])),
                                                rotation_deg=float(np.mean([p[1] for p in spread])))
        report["methods"][method] = method_report
        summaries.append(dict(session=spec["name"], method=method, success=success,
                              train_count=len(train), heldout_count=len(test),
                              **(metrics or dict(chain_translation_mm=None, chain_rotation_deg=None,
                                                 reprojection_rmse_px=None))))
    write_json(output / "report.json", report)
    write_json(output / "full_data_estimates.json", dict(
        status="provisional_not_deployed", event_ids=[r["id"] for r in records],
        note="All selected poses used. Not the estimates evaluated in report.json. No method automatically selected.",
        methods=final_fits))
    write_csv(output / "event_metrics.csv", event_rows)
    return report, summaries


def overlay(record, name, camera, y, x, path):
    cam = record["cams"][name]
    image = cv2.imread(str(cam["rgb"]))
    predicted = project(inv(y) @ record["robot"] @ x, cam["points"], camera)
    for actual, estimate in zip(cam["pixels"], predicted):
        a, b = tuple(np.rint(actual).astype(int)), tuple(np.rint(estimate).astype(int))
        cv2.circle(image, a, 4, (0, 220, 0), 1)
        if -10000 < b[0] < 10000 and -10000 < b[1] < 10000:
            cv2.drawMarker(image, b, (0, 0, 255), cv2.MARKER_CROSS, 9, 1)
            cv2.line(image, a, b, (0, 200, 255), 1)
    cv2.putText(image, f"Shah held-out event {record['id']} cam{name}: green=observed red=predicted",
                (15, 30), cv2.FONT_HERSHEY_SIMPLEX, .6, (255, 255, 255), 2)
    if not cv2.imwrite(str(path), image):
        raise OSError(f"Cannot write {path}")


def write_csv(path, rows):
    if not rows:
        return
    fields = list(dict.fromkeys(key for row in rows for key in row))
    with Path(path).open("w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def plot_summary(rows, path):
    sessions = list(dict.fromkeys(r["session"] for r in rows))
    fig, axes = plt.subplots(len(sessions), 3, figsize=(13, 3.6 * len(sessions)), squeeze=False)
    for i, session in enumerate(sessions):
        selected = [r for r in rows if r["session"] == session]
        for j, (metric, label) in enumerate((("chain_translation_mm", "Held-out chain [mm]"),
                                             ("chain_rotation_deg", "Held-out chain [deg]"),
                                             ("reprojection_rmse_px", "Held-out reprojection [px]"))):
            ax = axes[i, j]
            for k, row in enumerate(selected):
                value = row.get(metric)
                if value is not None:
                    ax.scatter(k, max(value, 1e-8), color="#277c8e" if row["method"] != "shah" else "#cb7037")
                else:
                    ax.text(k, .04, "FAIL/NA", rotation=90, ha="center", va="bottom",
                            transform=ax.get_xaxis_transform(), fontsize=8, color="red")
            ax.set_xticks(range(len(selected)), [r["method"] for r in selected], rotation=30)
            ax.set_yscale("log")
            ax.set_title(session.replace("session_", "") + "\n" + label)
            ax.grid(axis="y", alpha=.25)
    fig.suptitle("Real recapture evaluation — independent sessions, fixed held-out groups", fontsize=13)
    fig.tight_layout(rect=(0, 0, 1, .96))
    fig.savefig(path, dpi=160)
    plt.close(fig)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", type=Path, default=ROOT / "datasets/real_analysis_manifest.json")
    parser.add_argument("--intrinsics-dir", type=Path, default=ROOT / "intrinsics")
    parser.add_argument("--output", type=Path, required=True, help="New directory; refuses to overwrite a previous run")
    args = parser.parse_args(argv)
    manifest = json.loads(args.manifest.read_text(encoding="utf-8"))
    if args.output.exists():
        parser.error("output directory already exists; select a new run directory")
    if manifest["split"]["algorithm"] != "connected_pose_groups_every_fourth_v1":
        parser.error("unknown split algorithm")
    for spec in manifest["sessions"]:
        if Path(spec["meta"]).parent.name != spec["name"]:
            parser.error("session name and metadata directory do not match")
        if (spec["name"] in manifest["excluded_sessions"] or
                any(name in spec["meta"] for name in manifest["excluded_sessions"])):
            parser.error("excluded backup session in analysis inputs")
    args.output.mkdir(parents=True)
    run = dict(manifest=manifest, manifest_sha256=sha256(args.manifest),
               versions=dict(python=platform.python_version(), numpy=np.__version__, scipy=scipy.__version__,
                             opencv=cv2.__version__, matplotlib=matplotlib.__version__),
               runner_sha256=sha256(__file__), solver_sha256=sha256(ROOT / "SOTA_Simulation/shah_solver.py"),
               handeye_wrapper_sha256=sha256(ROOT / "SOTA_Simulation/tsai_combined_demo.py"),
               git_commit=subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip(),
               metrics_note="No GT. Chain residual compares FK*estimated_mount with estimated_camera*stored_PnP. Pixel errors use held-out measured corners. No 5.4x heuristic. success means a numerical solver output, not validated physical accuracy.",
               sessions=[])
    summary = []
    for spec in manifest["sessions"]:
        print(f"Evaluating {spec['name']} ...", flush=True)
        try:
            report, rows = evaluate_session(spec, args.intrinsics_dir.resolve(), manifest["split"], args.output / spec["name"])
            run["sessions"].append(dict(name=spec["name"], status="evaluated", split=report["split"],
                                        warnings=report["warnings"]))
            summary.extend(rows)
        except (ValueError, KeyError, OSError, cv2.error) as exc:
            run["sessions"].append(dict(name=spec["name"], status="failed", error=str(exc)))
            print(f"Session failed: {exc}", flush=True)
    write_json(args.output / "run.json", run)
    write_csv(args.output / "summary.csv", summary)
    if summary:
        plot_summary(summary, args.output / "comparison.png")
    print(json.dumps(summary, indent=2, allow_nan=False), flush=True)
    return 1 if any(s["status"] == "failed" for s in run["sessions"]) else 0


if __name__ == "__main__":
    raise SystemExit(main())
