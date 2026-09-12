"""Run six eye-to-hand solvers on the quality-gated 2026-09-10 captures.

The input selection is frozen in event_selection.json.  Full-data transforms are
provisional calibration outputs; leave-one-out metrics are kept separate and do
not use the omitted pose while fitting either camera or board mount.
"""
from __future__ import annotations

import hashlib
import json
import os
import platform
import sys
from itertools import combinations
from pathlib import Path

os.environ.setdefault("MPLBACKEND", "Agg")

import cv2
import numpy as np
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

from SOTA_Simulation.shah_solver import self_test, solve_shah_eye_to_hand
from SOTA_Simulation.tsai_combined_demo import solve_hand_eye

HERE = Path(__file__).resolve().parent
DATA = ROOT / "datasets" / "260910"
OUTPUT = ROOT / "reports" / "260910_calibration"
METHODS = ("shah", "tsai", "park", "horaud", "andreff", "daniilidis")


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def lf_sha256(path: Path) -> str:
    """Hash text as LF so Git's Windows CRLF checkout does not change provenance."""
    return hashlib.sha256(path.read_bytes().replace(b"\r\n", b"\n")).hexdigest()


def inverse(t: np.ndarray) -> np.ndarray:
    result = np.eye(4)
    result[:3, :3] = t[:3, :3].T
    result[:3, 3] = -t[:3, :3].T @ t[:3, 3]
    return result


def valid_transform(value) -> bool:
    t = np.asarray(value, dtype=float)
    return bool(t.shape == (4, 4) and np.isfinite(t).all()
                and np.allclose(t[3], [0, 0, 0, 1], atol=1e-7)
                and np.allclose(t[:3, :3].T @ t[:3, :3], np.eye(3), atol=1e-5)
                and abs(np.linalg.det(t[:3, :3]) - 1) < 1e-5)


def error(a: np.ndarray, b: np.ndarray) -> tuple[float, float]:
    translation = 1000 * np.linalg.norm(a[:3, 3] - b[:3, 3])
    rotation = Rotation.from_matrix(a[:3, :3].T @ b[:3, :3]).magnitude()
    return float(translation), float(np.degrees(rotation))


def average_transforms(values: list[np.ndarray]) -> np.ndarray:
    result = np.eye(4)
    result[:3, :3] = Rotation.from_matrix(
        np.stack([value[:3, :3] for value in values])).mean().as_matrix()
    result[:3, 3] = np.mean([value[:3, 3] for value in values], axis=0)
    return result


def stats(values: list[float]) -> dict | None:
    if not values:
        return None
    data = np.asarray(values, dtype=float)
    return {
        "median": float(np.median(data)),
        "p90": float(np.percentile(data, 90)),
        "max": float(np.max(data)),
        "mean": float(np.mean(data)),
    }


def tsai_informative_pairs(robot: list[np.ndarray], visual: list[np.ndarray]) -> int:
    """Mirror OpenCV's Tsai rotation-pair gate to catch silent identity output."""
    count = 0
    for i, j in combinations(range(len(robot)), 2):
        magnitudes = []
        for values in (robot, visual):
            angle = Rotation.from_matrix(
                values[i][:3, :3].T @ values[j][:3, :3]).magnitude()
            magnitudes.append(2 * np.sin(angle / 2))
        count += int(all(0.3 <= value <= 1.7 for value in magnitudes))
    return count


def fit(robot: list[np.ndarray], visual: list[np.ndarray], method: str):
    if len(robot) < 3:
        raise ValueError("at least three poses required")
    if method == "shah":
        answer = solve_shah_eye_to_hand(robot, visual)
        camera, mount = answer.T_base_fixed_i, answer.T_gripper_board
    else:
        if method == "tsai":
            informative = tsai_informative_pairs(robot, visual)
            if informative < 2:
                raise ValueError(f"Tsai has only {informative} informative motion pairs")
        camera = solve_hand_eye(robot, visual, eye_to_hand=True, method=method)
        mount = average_transforms([
            inverse(a) @ camera @ b for a, b in zip(robot, visual)
        ])
    if not valid_transform(camera) or not valid_transform(mount):
        raise ValueError("solver returned an invalid transform")
    return camera, mount


def load_session(name: str, selected: dict):
    meta_path = DATA / name / "meta.json"
    meta = json.loads(meta_path.read_text(encoding="utf-8"))
    if meta["board_config"] != selected["board"]:
        raise ValueError(f"{name}: board definition changed after assessment")
    if lf_sha256(meta_path) != selected["source_sha256"]:
        raise ValueError(f"{name}: meta.json hash differs from assessment")
    wanted = selected["eligible_ids"]
    indexed = {capture["event_id"]: capture for capture in meta["captures"]}
    if len(wanted) != selected["eligible_count"] or any(eid not in indexed for eid in wanted):
        raise ValueError(f"{name}: invalid frozen event selection")
    records = []
    for eid in wanted:
        capture = indexed[eid]
        if capture.get("capture_gate", {}).get("pass") is not True:
            raise ValueError(f"{name} event {eid}: selected event no longer passes gate")
        robot = np.asarray(capture["robot_pose_matrix_4x4"], dtype=float)
        if not valid_transform(robot):
            raise ValueError(f"{name} event {eid}: invalid robot transform")
        cameras = {}
        for camera in meta["cam_indices"]:
            visual = np.asarray(capture["cams"][str(camera)]["charuco"]["T_cam_board_4x4"], dtype=float)
            if not valid_transform(visual):
                raise ValueError(f"{name} event {eid} cam{camera}: invalid visual transform")
            cameras[str(camera)] = visual
        records.append({"event_id": eid, "robot": robot, "cameras": cameras})
    return meta_path, meta, records


def calibrate_session(name: str, assessment: dict) -> dict:
    meta_path, meta, records = load_session(name, assessment)
    result = {
        "status": "provisional_not_deployed",
        "meta": str(meta_path.relative_to(ROOT)).replace("\\", "/"),
        "meta_sha256_worktree_bytes": sha256(meta_path),
        "meta_sha256_lf_normalized": lf_sha256(meta_path),
        "eligible_event_ids": [record["event_id"] for record in records],
        "camera_serials": meta["cam_serials"],
        "board_config": meta["board_config"],
        "methods": {},
    }
    for method in METHODS:
        method_result = {"success": True, "cameras": {}}
        mounts = []
        for camera in map(str, meta["cam_indices"]):
            robot = [record["robot"] for record in records]
            visual = [record["cameras"][camera] for record in records]
            camera_result = {}
            try:
                y, x = fit(robot, visual, method)
                loo_translation, loo_rotation, failures = [], [], []
                for omitted, record in enumerate(records):
                    keep = [index for index in range(len(records)) if index != omitted]
                    try:
                        y_loo, x_loo = fit([robot[index] for index in keep],
                                           [visual[index] for index in keep], method)
                        dt, dr = error(record["robot"] @ x_loo,
                                       y_loo @ record["cameras"][camera])
                        loo_translation.append(dt)
                        loo_rotation.append(dr)
                    except Exception as exc:
                        failures.append({"event_id": record["event_id"], "error": str(exc)})
                camera_result = {
                    "success": True,
                    "T_base_camera": y.tolist(),
                    "T_gripper_board": x.tolist(),
                    "leave_one_out": {
                        "translation_mm": stats(loo_translation),
                        "rotation_deg": stats(loo_rotation),
                        "successes": len(loo_translation),
                        "failures": failures,
                    },
                }
                mounts.append(x)
            except Exception as exc:
                camera_result = {"success": False, "error": str(exc)}
                method_result["success"] = False
            method_result["cameras"][camera] = camera_result
        if method_result["success"]:
            center = average_transforms(mounts)
            spread = [error(center, mount) for mount in mounts]
            method_result["camera_pair_mount_disagreement"] = {
                "translation_mm_mean": float(np.mean([value[0] for value in spread])),
                "rotation_deg_mean": float(np.mean([value[1] for value in spread])),
            }
        result["methods"][method] = method_result
    return result


def main() -> int:
    if not self_test(verbose=False):
        raise RuntimeError("Shah convention self-test failed")
    assessment_path = HERE / "assessment.json"
    assessment = json.loads(assessment_path.read_text(encoding="utf-8"))
    selection_path = HERE / "event_selection.json"
    selection = json.loads(selection_path.read_text(encoding="utf-8"))
    if assessment["criteria"] != selection["criteria"]:
        raise ValueError("assessment and selection criteria differ")
    sessions = {}
    for name in ("cam0-1_02", "cam1-3_02", "cam0-3_01"):
        print(f"Calibrating {name} ...", flush=True)
        sessions[name] = calibrate_session(name, assessment["sessions"][name])
    intrinsics = {}
    for camera in (0, 1, 3):
        path = ROOT / "intrinsics" / f"cam{camera}.npz"
        intrinsics[str(camera)] = {"path": str(path.relative_to(ROOT)).replace("\\", "/"),
                                   "sha256": sha256(path)}
    output = {
        "schema_version": "real_calibration_comparison_v1",
        "status": "provisional_not_deployed",
        "transform_convention": "T_destination_source, translation in metre",
        "selection": str(selection_path.relative_to(ROOT)).replace("\\", "/"),
        "selection_sha256": sha256(selection_path),
        "assessment_sha256": sha256(assessment_path),
        "versions": {"python": platform.python_version(), "opencv": cv2.__version__,
                     "numpy": np.__version__},
        "intrinsics": intrinsics,
        "sessions": sessions,
    }
    OUTPUT.mkdir(parents=True, exist_ok=True)
    target = OUTPUT / "calibration_results.json"
    target.write_text(json.dumps(output, indent=2, ensure_ascii=False,
                                 allow_nan=False) + "\n", encoding="utf-8")
    primary = sessions["cam1-3_02"]
    candidate = {
        "schema_version": "provisional_eye_to_hand_calibration_v1",
        "status": "provisional_not_deployed",
        "reason": "Best-supported 2026-09-10 camera pair; Shah matches the AX=YB problem and jointly estimates camera and board-mount transforms.",
        "session": "cam1-3_02",
        "method": "shah",
        "transform_convention": output["transform_convention"],
        "eligible_event_ids": primary["eligible_event_ids"],
        "camera_serials": primary["camera_serials"],
        "board_config": primary["board_config"],
        "cameras": primary["methods"]["shah"]["cameras"],
        "camera_pair_mount_disagreement": primary["methods"]["shah"]["camera_pair_mount_disagreement"],
        "source_results": str(target.relative_to(ROOT)).replace("\\", "/"),
        "warning": "Internal consistency only. Validate against an independent physical reference before robot deployment.",
    }
    candidate_path = OUTPUT / "cam1-3_02_shah_candidate.json"
    candidate_path.write_text(json.dumps(candidate, indent=2, ensure_ascii=False,
                                         allow_nan=False) + "\n", encoding="utf-8")
    print(target, flush=True)
    print(candidate_path, flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
