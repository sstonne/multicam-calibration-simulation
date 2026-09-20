"""Calibrate the wrist camera and cross-check the four-camera rig on fixedBoard.

The fixedBoard data are never appended to the moving gripper-board calibration.
They form a separate eye-in-hand calibration and cross-target evaluation.
"""
from __future__ import annotations

import argparse
from collections import Counter
import json
from itertools import combinations
from pathlib import Path
import platform
import subprocess

import cv2
import numpy as np
import scipy
from scipy.spatial.transform import Rotation

from capture.board_config import TABLE_BOARD
from SOTA_Simulation import real_evaluation as r
from SOTA_Simulation.shah_solver import solve_shah_eye_in_hand, self_test
from SOTA_Simulation.tsai_combined_demo import solve_hand_eye


ROOT = r.ROOT
DATASET = ROOT / "datasets" / "fixedBoard"
PRIOR_ESTIMATES = ROOT / "results" / "real_260910_unified_20260917_v2" / "full_data_estimates.json"
CAMERA_FILES = {
    "0": "cam_039422061216",
    "1": "cam_fixed2",
    "2": "cam_gripper",
    "3": "cam_fixed3",
}
SERIALS = {
    "0": "039422061216",
    "1": "319522062138",
    "2": "752112070297",
    "3": "912322060991",
}
FIXED_CAMERAS = ("0", "1", "3")
ALL_CAMERAS = ("0", "1", "2", "3")
HELDOUT_IDS = (2, 5, 9, 12)


def serializable(value):
    if isinstance(value, np.ndarray):
        return serializable(value.tolist())
    if isinstance(value, np.generic):
        return serializable(value.item())
    if isinstance(value, dict):
        return {key: serializable(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [serializable(item) for item in value]
    return value


def save(path, value):
    r.write_json(path, serializable(value))


def read_image(path, flags=cv2.IMREAD_COLOR):
    return cv2.imdecode(np.frombuffer(Path(path).read_bytes(), dtype=np.uint8), flags)


def robot_pose(pose6):
    """i611 [x,y,z,rz,ry,rx] mm/degree to T_base_robot_pose."""
    values = np.asarray(pose6, dtype=float)
    if values.shape != (6,) or not np.isfinite(values).all():
        raise ValueError("invalid_robot_pose_6dof")
    transform = np.eye(4)
    transform[:3, :3] = Rotation.from_euler("ZYX", values[3:], degrees=True).as_matrix()
    transform[:3, 3] = values[:3] / 1000.0
    return r.transform(transform)


def make_board():
    config = dict(
        squares_x=TABLE_BOARD.squares_x,
        squares_y=TABLE_BOARD.squares_y,
        square_length_m=TABLE_BOARD.square_length_m,
        marker_length_m=TABLE_BOARD.marker_length_m,
        dictionary_name=TABLE_BOARD.dictionary_name,
        marker_id_start=TABLE_BOARD.marker_id_start,
        legacy_pattern=TABLE_BOARD.legacy_pattern,
    )
    board = r.board_from_meta(config)
    board.setLegacyPattern(config["legacy_pattern"])
    return config, board


def detect_pose(image, board, camera):
    corners, corner_ids, _, _ = cv2.aruco.CharucoDetector(board).detectBoard(image)
    if corners is None or corner_ids is None or len(corner_ids) < 4:
        raise ValueError("fewer_than_four_charuco_corners")
    ids = corner_ids.reshape(-1).astype(int)
    pixels = corners.reshape(-1, 2).astype(float)
    if len(set(ids.tolist())) != len(ids) or not np.isfinite(pixels).all():
        raise ValueError("invalid_charuco_corners")
    points = board.getChessboardCorners().astype(float)[ids]
    success, rvec, tvec = cv2.solvePnP(
        points, pixels, camera["K"], camera["D"], flags=cv2.SOLVEPNP_ITERATIVE
    )
    if not success:
        raise ValueError("solvepnp_failed")
    visual = np.eye(4)
    visual[:3, :3] = cv2.Rodrigues(rvec)[0]
    visual[:3, 3] = np.asarray(tvec).reshape(3)
    visual = r.transform(visual)
    if np.any((points @ visual[:3, :3].T + visual[:3, 3])[:, 2] <= 0):
        raise ValueError("nonpositive_pnp_depth")
    prediction = r.project(visual, points, camera)
    rmse = float(np.sqrt(np.mean(np.sum((prediction - pixels) ** 2, axis=1))))
    return visual, ids, pixels, points, rmse


def prepare():
    board_config, board = make_board()
    cameras = {}
    for name in ALL_CAMERAS:
        path = ROOT / "intrinsics" / f"cam{name}.npz"
        with np.load(path, allow_pickle=False) as data:
            serial = str(data["serial"].item())
            if serial != SERIALS[name]:
                raise ValueError(f"cam{name}_intrinsic_serial_mismatch")
            if bool(data["is_gripper"].item()) != (name == "2"):
                raise ValueError(f"cam{name}_role_mismatch")
            if (int(data["color_w"]), int(data["color_h"])) != (1280, 720):
                raise ValueError(f"cam{name}_resolution_mismatch")
            k, d = data["color_K"].astype(float), data["color_D"].astype(float)
        if not np.isfinite(k).all() or not np.isfinite(d).all():
            raise ValueError(f"cam{name}_invalid_intrinsic")
        cameras[name] = dict(
            K=k,
            D=d,
            serial=serial,
            file=str(path.relative_to(ROOT)),
            sha256=r.sha256(path),
            role="gripper" if name == "2" else "fixed",
            source_stem=CAMERA_FILES[name],
        )

    folders = sorted(path for path in DATASET.iterdir() if path.is_dir())
    if [path.name for path in folders] != [f"{index:03d}" for index in range(15)]:
        raise ValueError("expected_fixed_board_folders_000_to_014")
    records, audit, input_files = [], [], []
    for folder in folders:
        metadata_path = folder / "robot.json"
        raw = json.loads(metadata_path.read_text(encoding="utf-8"))
        event_id = int(folder.name)
        if raw.get("capture_index") != event_id:
            raise ValueError(f"capture_index_mismatch:{folder.name}")
        if "pose=[x,y,z,rz,ry,rx] mm/deg" not in raw.get("pose_convention", ""):
            raise ValueError(f"unknown_robot_pose_convention:{folder.name}")
        robot = robot_pose(raw.get("pose"))
        input_files.append(dict(file=str(metadata_path.relative_to(ROOT)), sha256=r.sha256(metadata_path)))
        record = dict(
            id=event_id,
            key=folder.name,
            timestamp=raw.get("timestamp"),
            robot=robot,
            robot_pose_6dof=raw["pose"],
            joints=raw.get("joints"),
            replayed_from=raw.get("replayed_from"),
            cams={},
        )
        for name in ALL_CAMERAS:
            rgb = folder / f"{CAMERA_FILES[name]}.png"
            depth = folder / f"{CAMERA_FILES[name]}_depth.png"
            item = dict(event_id=event_id, camera=name, included=False, reason=None)
            try:
                image = read_image(rgb)
                depth_image = read_image(depth, cv2.IMREAD_UNCHANGED)
                if image is None or image.shape != (720, 1280, 3) or image.dtype != np.uint8:
                    raise ValueError("invalid_rgb")
                if depth_image is None or depth_image.shape != (720, 1280) or depth_image.dtype != np.uint16:
                    raise ValueError("invalid_depth")
                visual, ids, pixels, points, rmse = detect_pose(image, board, cameras[name])
                record["cams"][name] = dict(
                    visual=visual,
                    corner_ids=ids,
                    pixels=pixels,
                    points=points,
                )
                item.update(
                    included=True,
                    detected_corners=len(ids),
                    pnp_reprojection_rmse_px=rmse,
                    T_camera_board=visual,
                )
            except (OSError, ValueError, TypeError, cv2.error) as exc:
                item["reason"] = str(exc)
            for kind, path in (("rgb", rgb), ("depth", depth)):
                if not path.is_file():
                    raise ValueError(f"missing_{kind}:{path}")
                digest = r.sha256(path)
                item[f"{kind}_file"] = str(path.relative_to(ROOT))
                item[f"{kind}_sha256"] = digest
                input_files.append(dict(file=str(path.relative_to(ROOT)), sha256=digest))
            audit.append(item)
        if "2" not in record["cams"]:
            raise ValueError(f"wrist_observation_unavailable:{folder.name}")
        records.append(record)

    groups = r.pose_groups(records, translation_mm=1.5, rotation_deg=1.0)
    desired = set(HELDOUT_IDS)
    heldout = {event_id for group in groups if desired.intersection(group) for event_id in group}
    if heldout != desired:
        raise ValueError(f"heldout_pose_group_expansion:{sorted(heldout)}")
    train = [record for record in records if record["id"] not in heldout]
    test = [record for record in records if record["id"] in heldout]
    if len(train) != 11 or len(test) != 4:
        raise ValueError("unexpected_fixed_split_size")

    if not PRIOR_ESTIMATES.is_file():
        raise ValueError("missing_prior_fixed_camera_estimates")
    prior_hash = r.sha256(PRIOR_ESTIMATES)
    input_files.append(dict(file=str(PRIOR_ESTIMATES.relative_to(ROOT)), sha256=prior_hash))
    return dict(
        records=records,
        audit=audit,
        cameras=cameras,
        board=board_config,
        camera_mapping={
            name: dict(
                source_stem=CAMERA_FILES[name],
                camera_index=int(name),
                serial=SERIALS[name],
                intrinsic=f"intrinsics/cam{name}.npz",
                mapping_basis=(
                    "serial_in_filename" if name == "0" else
                    "user_confirmed_gripper_is_cam2" if name == "2" else
                    "user_confirmed_fixed_camera_order_0_1_3"
                ),
            )
            for name in ALL_CAMERAS
        },
        split=dict(
            policy="Existing common simulation held-out IDs 2,5,9,12; near-pose groups checked before fitting",
            near_translation_mm=1.5,
            near_rotation_deg=1.0,
            train_keys=[record["key"] for record in train],
            heldout_keys=[record["key"] for record in test],
            groups=groups,
        ),
        input_files=input_files,
        prior_fixed_estimates=dict(
            file=str(PRIOR_ESTIMATES.relative_to(ROOT)),
            sha256=prior_hash,
            meaning="September 10 moving gripper-board full-data fixed-camera estimates",
        ),
        source=dict(
            directory=str(DATASET.relative_to(ROOT)),
            raw_events=len(records),
            robot_pose_frame="i611 pose frame; exact active tool is not encoded in robot.json",
            depth_usage="hashed and audited, not used in RGB corner/PnP calibration",
        ),
        versions=dict(
            python=platform.python_version(),
            opencv=cv2.__version__,
            numpy=np.__version__,
            scipy=scipy.__version__,
        ),
        git_commit=subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip(),
        implementation_hashes={
            str(path.relative_to(ROOT)): r.sha256(path)
            for path in (
                Path(__file__),
                ROOT / "SOTA_Simulation" / "real_evaluation.py",
                ROOT / "SOTA_Simulation" / "shah_solver.py",
                ROOT / "SOTA_Simulation" / "tsai_combined_demo.py",
                ROOT / "capture" / "board_config.py",
            )
        },
    )


def restore(data):
    for camera in data["cameras"].values():
        camera["K"] = np.asarray(camera["K"], dtype=float)
        camera["D"] = np.asarray(camera["D"], dtype=float)
    for record in data["records"]:
        record["robot"] = np.asarray(record["robot"], dtype=float)
        for camera in record["cams"].values():
            camera["visual"] = np.asarray(camera["visual"], dtype=float)
            camera["corner_ids"] = np.asarray(camera["corner_ids"], dtype=int)
            camera["pixels"] = np.asarray(camera["pixels"], dtype=float).reshape(-1, 2)
            camera["points"] = np.asarray(camera["points"], dtype=float).reshape(-1, 3)
    return data


def check_sources(data):
    for item in data["input_files"]:
        if r.sha256(ROOT / item["file"]) != item["sha256"]:
            raise ValueError(f"input_changed:{item['file']}")


def informative_tsai_pairs(records):
    robot = [record["robot"] for record in records]
    visual = [record["cams"]["2"]["visual"] for record in records]
    count = 0
    for first, second in combinations(range(len(records)), 2):
        magnitudes = [
            2 * np.sin(Rotation.from_matrix(values[first][:3, :3].T @ values[second][:3, :3]).magnitude() / 2)
            for values in (robot, visual)
        ]
        count += int(all(0.3 <= value <= 1.7 for value in magnitudes))
    return count


def fit_wrist(records, method):
    robot = [record["robot"] for record in records]
    visual = [record["cams"]["2"]["visual"] for record in records]
    if len(records) < 3:
        raise ValueError("fewer_than_three_wrist_poses")
    if method == "tsai":
        informative = informative_tsai_pairs(records)
        if informative < 2:
            raise ValueError(f"tsai_insufficient_informative_motion_pairs:{informative}")
    if method == "shah":
        result = solve_shah_eye_in_hand(robot, visual)
        gripper_camera, base_board = result.T_gripper_wrist, result.T_base_board
    else:
        gripper_camera = solve_hand_eye(robot, visual, eye_to_hand=False, method=method)
        gripper_camera = r.transform(gripper_camera)
        base_board = r.average([
            base_gripper @ gripper_camera @ camera_board
            for base_gripper, camera_board in zip(robot, visual)
        ])
    return r.transform(gripper_camera), r.transform(base_board)


def load_prior(method):
    raw = json.loads(PRIOR_ESTIMATES.read_text(encoding="utf-8"))["methods"][method]
    result = {}
    for name in FIXED_CAMERAS:
        if not raw[name]["success"]:
            raise ValueError(f"prior_cam{name}_{method}_failed")
        result[name] = r.transform(raw[name]["T_base_camera"])
    return result


def camera_poses(record, gripper_camera, prior_fixed):
    poses = dict(prior_fixed)
    poses["2"] = record["robot"] @ gripper_camera
    return poses


def score_records(records, cameras, gripper_camera, base_board, prior_fixed, method, split):
    rows = []
    for record in records:
        poses = camera_poses(record, gripper_camera, prior_fixed)
        for name, observed in record["cams"].items():
            calibrated_pose = poses[name]
            observed_pose = base_board @ r.inv(observed["visual"])
            pose_mm, pose_deg = r.errors(calibrated_pose, observed_pose)
            chain_mm, chain_deg = r.errors(calibrated_pose @ observed["visual"], base_board)
            item = dict(
                method=method,
                split=split,
                event_id=record["id"],
                camera=name,
                chain_translation_mm=chain_mm,
                chain_rotation_deg=chain_deg,
                camera_pose_translation_mm=pose_mm,
                camera_pose_rotation_deg=pose_deg,
                corner_count=len(observed["pixels"]),
                pixel_sse=None,
                reprojection_rmse_px=None,
                pixel_failure=None,
            )
            try:
                predicted_visual = r.inv(calibrated_pose) @ base_board
                prediction = r.project(predicted_visual, observed["points"], cameras[name])
                item["pixel_sse"] = float(np.sum((prediction - observed["pixels"]) ** 2))
                item["reprojection_rmse_px"] = float(np.sqrt(item["pixel_sse"] / len(prediction)))
            except (ValueError, cv2.error) as exc:
                item["pixel_failure"] = str(exc)
            rows.append(item)
    return rows


def registration_rows(records, cameras, gripper_camera, base_board, prior_fixed, method, split):
    rows = []
    for record in records:
        poses = camera_poses(record, gripper_camera, prior_fixed)
        names = sorted(record["cams"])
        for first_index, first in enumerate(names):
            for second in names[first_index + 1:]:
                first_observation = record["cams"][first]
                second_observation = record["cams"][second]
                calibrated_relative = r.inv(poses[second]) @ poses[first]
                observed_relative = second_observation["visual"] @ r.inv(first_observation["visual"])
                translation_mm, rotation_deg = r.errors(calibrated_relative, observed_relative)
                item = dict(
                    method=method,
                    split=split,
                    event_id=record["id"],
                    pair=f"{first}-{second}",
                    translation_mm=translation_mm,
                    rotation_deg=rotation_deg,
                    directed_corner_count=0,
                    cross_view_pixel_sse=0.0,
                    cross_view_reprojection_rmse_px=None,
                    pixel_failure=None,
                )
                try:
                    directions = (
                        (first, second, first_observation, second_observation, calibrated_relative),
                        (second, first, second_observation, first_observation, r.inv(calibrated_relative)),
                    )
                    for _, destination, source_observation, destination_observation, transfer in directions:
                        predicted_board = transfer @ source_observation["visual"]
                        prediction = r.project(predicted_board, destination_observation["points"], cameras[destination])
                        item["cross_view_pixel_sse"] += float(np.sum(
                            (prediction - destination_observation["pixels"]) ** 2
                        ))
                        item["directed_corner_count"] += len(prediction)
                    item["cross_view_reprojection_rmse_px"] = float(np.sqrt(
                        item["cross_view_pixel_sse"] / item["directed_corner_count"]
                    ))
                except (ValueError, cv2.error) as exc:
                    item["pixel_failure"] = str(exc)
                    item["cross_view_pixel_sse"] = None
                    item["directed_corner_count"] = 0
                rows.append(item)
    return rows


def aggregate_scores(rows):
    if not rows:
        return None
    pixel_rows = [row for row in rows if row["pixel_sse"] is not None]
    corners = sum(row["corner_count"] for row in pixel_rows)
    per_camera = {}
    for name in sorted({row["camera"] for row in rows}):
        selected = [row for row in rows if row["camera"] == name]
        per_camera[name] = dict(
            translation_mm=float(np.mean([row["camera_pose_translation_mm"] for row in selected])),
            rotation_deg=float(np.mean([row["camera_pose_rotation_deg"] for row in selected])),
        )
    return dict(
        heldout_translation_mm=float(np.mean([row["chain_translation_mm"] for row in rows])),
        heldout_rotation_deg=float(np.mean([row["chain_rotation_deg"] for row in rows])),
        camera_pose_translation_mm=float(np.mean([value["translation_mm"] for value in per_camera.values()])),
        camera_pose_rotation_deg=float(np.mean([value["rotation_deg"] for value in per_camera.values()])),
        camera_pose_aggregation="equal-camera macro after per-camera event mean",
        reprojection_rmse_px=(
            float(np.sqrt(sum(row["pixel_sse"] for row in pixel_rows) / corners))
            if corners and len(pixel_rows) == len(rows) else None
        ),
        reprojection_complete=len(pixel_rows) == len(rows),
        observations=len(rows),
        corners=corners,
    )


def aggregate_registration(rows):
    if not rows:
        return None
    per_pair = {}
    for pair in sorted({row["pair"] for row in rows}):
        selected = [row for row in rows if row["pair"] == pair]
        per_pair[pair] = dict(
            translation_mm=float(np.mean([row["translation_mm"] for row in selected])),
            rotation_deg=float(np.mean([row["rotation_deg"] for row in selected])),
        )
    pixel_rows = [row for row in rows if row["cross_view_pixel_sse"] is not None]
    corners = sum(row["directed_corner_count"] for row in pixel_rows)
    return dict(
        registration_translation_mm=float(np.mean([value["translation_mm"] for value in per_pair.values()])),
        registration_rotation_deg=float(np.mean([value["rotation_deg"] for value in per_pair.values()])),
        registration_aggregation="equal-pair macro after per-event pair mean",
        cross_view_reprojection_rmse_px=(
            float(np.sqrt(sum(row["cross_view_pixel_sse"] for row in pixel_rows) / corners))
            if corners and len(pixel_rows) == len(rows) else None
        ),
        cross_view_complete=len(pixel_rows) == len(rows),
        pair_observations=len(rows),
        directed_corners=corners,
    )


def cross_target_fixed_estimates(records, base_board):
    estimates = {}
    for name in FIXED_CAMERAS:
        poses = [base_board @ r.inv(record["cams"][name]["visual"])
                 for record in records if name in record["cams"]]
        estimates[name] = dict(
            success=bool(poses),
            count=len(poses),
            T_base_camera=r.average(poses).tolist() if poses else None,
        )
    return estimates


def fixed_estimate_matrices(records, base_board):
    result = {}
    for name in FIXED_CAMERAS:
        poses = [base_board @ r.inv(record["cams"][name]["visual"])
                 for record in records if name in record["cams"]]
        if not poses:
            raise ValueError(f"no_fixed_board_observations_for_cam{name}")
        result[name] = r.transform(r.average(poses))
    return result


def collect(output, data):
    summaries, camera_rows, event_rows, pair_rows, estimates = [], [], [], [], {}
    prior_summaries, prior_event_rows, prior_pair_rows = [], [], []
    for method in r.METHODS:
        result = json.loads((output / f"{method}.json").read_text(encoding="utf-8"))
        if result["prepared_sha256"] != r.sha256(output / "prepared.json"):
            raise ValueError("different_prepared_input_population")
        scores = result["heldout_rows"]
        registrations = result["registration_rows"]
        score_summary = aggregate_scores(scores)
        registration_summary = aggregate_registration(registrations)
        summaries.append(dict(
            method=method,
            success=result["train_estimate"]["success"],
            **score_summary,
            **registration_summary,
            reference_kind="fixed-board held-out PnP consistency; not independent physical GT",
        ))
        prior_scores = result["prior_validation_rows"]
        prior_registrations = result["prior_registration_rows"]
        prior_summaries.append(dict(
            method=method,
            **aggregate_scores(prior_scores),
            **aggregate_registration(prior_registrations),
            meaning="September 9 fixedBoard compared with September 10 moving-board fixed-camera calibration",
        ))
        for name in ALL_CAMERAS:
            selected = [row for row in scores if row["camera"] == name]
            item = aggregate_scores(selected)
            camera_rows.append(dict(
                method=method,
                camera=name,
                raw=15,
                included=sum(a["camera"] == name and a["included"] for a in data["audit"]),
                train=sum(record["key"] in data["split"]["train_keys"] and name in record["cams"] for record in data["records"]),
                heldout=len(selected),
                **item,
            ))
        event_rows += result["training_rows"] + scores
        pair_rows += result["training_registration_rows"] + registrations
        prior_event_rows += prior_scores
        prior_pair_rows += prior_registrations
        estimates[method] = result["full_data_estimate"]

    report = dict(
        status="review_pending_not_deployed",
        summary=summaries,
        prior_calibration_comparison=prior_summaries,
        source=data["source"],
        board=data["board"],
        camera_mapping=data["camera_mapping"],
        split=data["split"],
        versions=data["versions"],
        git_commit=data["git_commit"],
        implementation_hashes=data["implementation_hashes"],
        prior_fixed_estimates=data["prior_fixed_estimates"],
        counts={
            name: dict(
                raw=15,
                included=sum(item["camera"] == name and item["included"] for item in data["audit"]),
            )
            for name in ALL_CAMERAS
        },
        excluded_reasons=dict(Counter(
            item["reason"] for item in data["audit"] if not item["included"]
        )),
        definitions=dict(
            heldout="Mean fixed-board base-pose chain residual across held-out event-camera observations; cam0/1/3 and cam2 are all fitted on fixedBoard train only.",
            camera_pose="Equal-camera macro discrepancy between calibrated camera poses and fixed-board PnP reference poses; not physical GT.",
            registration="Equal-pair macro discrepancy between calibrated relative cameras and same-event fixed-board PnP relative cameras; not physical GT.",
            reprojection="Corner-pooled held-out RMSE from frozen train calibration; sqrt(sum(||duv||^2)/N corners).",
            cross_view_reprojection="Both directions pooled for each available camera pair using source PnP and calibrated relative camera transform.",
        ),
        assumptions=[
            "Only the separate prior-comparison diagnostic assumes unchanged fixed-camera mounts between September 9 and September 10; metadata cannot prove this.",
            "robot.json identifies an i611 pose but does not encode the exact active tool; the recorded pose frame is assumed rigidly attached to cam2.",
            "Camera mapping follows the user-confirmed RealSense fixed order 0,1,3 and gripper camera 2.",
            "Current intrinsics are matched by confirmed camera identity and resolution; capture-time intrinsic hashes are unavailable.",
            "No corner-count/reprojection quality threshold, synthetic noise, robust loss, or outlier removal was used.",
            "Depth PNGs are preserved and hashed but excluded because depth scale and RGB-depth alignment metadata are absent.",
        ],
    )
    save(output / "analysis.json", report)
    save(output / "full_data_estimates.json", dict(
        status="review_pending_not_deployed",
        note="cam2 eye-in-hand and fixed-board-derived cam0/1/3 estimates; separate from held-out train estimates.",
        methods=estimates,
    ))
    r.write_csv(output / "summary.csv", summaries)
    r.write_csv(output / "per_camera.csv", camera_rows)
    r.write_csv(output / "event_metrics.csv", event_rows)
    r.write_csv(output / "registration_pairs.csv", pair_rows)
    r.write_csv(output / "prior_calibration_comparison.csv", prior_summaries)
    r.write_csv(output / "prior_event_metrics.csv", prior_event_rows)
    r.write_csv(output / "prior_registration_pairs.csv", prior_pair_rows)
    print(json.dumps(dict(counts=report["counts"], summary=summaries), indent=2), flush=True)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--method", choices=("prepare", "collect") + r.METHODS, required=True)
    args = parser.parse_args(argv)
    output = args.output.resolve()
    cv2.setNumThreads(1)
    if args.method == "prepare":
        if output.exists():
            raise ValueError("previous_output_exists_select_new_directory")
        if not self_test():
            raise ValueError("shah_convention_self_test_failed")
        data = prepare()
        check_sources(data)
        output.mkdir(parents=True, exist_ok=False)
        save(output / "prepared.json", data)
        save(output / "audit.json", data["audit"])
        save(output / "split.json", data["split"])
        print(f"Prepared {len(data['records'])} fixedBoard events", flush=True)
        return

    data = restore(json.loads((output / "prepared.json").read_text(encoding="utf-8")))
    check_sources(data)
    if args.method == "collect":
        collect(output, data)
        return
    path = output / f"{args.method}.json"
    if path.exists():
        raise ValueError("method_result_exists_refuses_overwrite")
    if not self_test():
        raise ValueError("shah_convention_self_test_failed")
    train_keys = set(data["split"]["train_keys"])
    train = [record for record in data["records"] if record["key"] in train_keys]
    heldout = [record for record in data["records"] if record["key"] not in train_keys]
    print(f"Fitting cam2 eye-in-hand with {args.method}: one CPU thread", flush=True)
    try:
        gripper_camera, base_board = fit_wrist(train, args.method)
        fitted_fixed = fixed_estimate_matrices(train, base_board)
        train_estimate = dict(
            success=True,
            count=len(train),
            informative_tsai_pairs=(informative_tsai_pairs(train) if args.method == "tsai" else None),
            T_gripper_camera=gripper_camera,
            T_base_fixed_board=base_board,
            fixed_cameras={
                name: dict(
                    count=sum(name in record["cams"] for record in train),
                    T_base_camera=value,
                )
                for name, value in fitted_fixed.items()
            },
        )
        training_rows = score_records(train, data["cameras"], gripper_camera, base_board, fitted_fixed, args.method, "train")
        heldout_rows = score_records(heldout, data["cameras"], gripper_camera, base_board, fitted_fixed, args.method, "heldout")
        training_registration = registration_rows(train, data["cameras"], gripper_camera, base_board, fitted_fixed, args.method, "train")
        heldout_registration = registration_rows(heldout, data["cameras"], gripper_camera, base_board, fitted_fixed, args.method, "heldout")
        prior = load_prior(args.method)
        prior_validation_rows = score_records(heldout, data["cameras"], gripper_camera, base_board, prior, args.method, "prior_comparison")
        prior_registration_rows = registration_rows(heldout, data["cameras"], gripper_camera, base_board, prior, args.method, "prior_comparison")
    except (ValueError, cv2.error, np.linalg.LinAlgError, AssertionError) as exc:
        train_estimate = dict(success=False, count=len(train), error=str(exc))
        training_rows, heldout_rows = [], []
        training_registration, heldout_registration = [], []
        prior_validation_rows, prior_registration_rows = [], []

    try:
        full_gripper_camera, full_base_board = fit_wrist(data["records"], args.method)
        full_estimate = dict(
            success=True,
            count=len(data["records"]),
            T_gripper_camera=full_gripper_camera,
            T_base_fixed_board=full_base_board,
            cross_target_fixed_cameras=cross_target_fixed_estimates(data["records"], full_base_board),
        )
    except (ValueError, cv2.error, np.linalg.LinAlgError, AssertionError) as exc:
        full_estimate = dict(success=False, count=len(data["records"]), error=str(exc))

    result = dict(
        method=args.method,
        prepared_sha256=r.sha256(output / "prepared.json"),
        train_estimate=train_estimate,
        full_data_estimate=full_estimate,
        training_rows=training_rows,
        heldout_rows=heldout_rows,
        training_registration_rows=training_registration,
        registration_rows=heldout_registration,
        prior_validation_rows=prior_validation_rows,
        prior_registration_rows=prior_registration_rows,
    )
    check_sources(data)
    save(path, result)
    print(f"Saved {path}", flush=True)


if __name__ == "__main__":
    main()
