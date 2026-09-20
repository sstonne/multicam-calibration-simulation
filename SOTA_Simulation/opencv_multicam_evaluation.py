#!/usr/bin/env python3
"""Held-out, camera-pose, registration and reprojection evaluation.

Every OpenCV method receives the same calibration poses and noisy observations.
The held-out poses are never passed to ``calibrateHandEye``.
"""

from __future__ import annotations

import argparse
import csv
from itertools import combinations
import json
import os
from pathlib import Path
import sys

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
os.environ.setdefault("QT_API", "PyQt5")

import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial.transform import Rotation

from SOTA_Simulation.sota_simulation import (
    CameraModel,
    generate_case,
    invert_transform,
    load_config,
    pose_error,
    project_points,
)
from SOTA_Simulation.tsai_combined_demo import (
    HAND_EYE_METHODS,
    build_eye_in_hand_session,
    solve_hand_eye,
)
from SOTA_Simulation.tsai_noise_sweep import (
    make_corner_observations,
    prepare_fixed_inputs,
)

from SOTA_Simulation.shah_solver import (
    solve_shah_eye_in_hand,
    solve_shah_eye_to_hand,
)

from SOTA_Simulation.tabb_solver import (
    PixelObservations,
    TabbConfig,
    solve_tabb_eye_in_hand,
    solve_tabb_eye_to_hand,
)

# Shah는 AX=YB를 풀므로 HAND_EYE_METHODS에 섞지 않고 별도 set으로 관리
ROBOT_WORLD_METHODS = {"shah": "SHAH", "li": "LI"}

# Tabb(2017)는 AX=ZB를 iterative로 푼다. cost function / separable 여부 /
# rotation parameterization / multi-eye 결합 여부의 조합이므로 registry로 관리한다.
# joint=True면 논문 Sec. 2.3처럼 fixed camera 3대가 X 하나를 공유한다.
TABB_METHODS = {
    "tabb_c1_sep": (TabbConfig(cost="c1", separable=True), False),
    "tabb_c1_sim": (TabbConfig(cost="c1", separable=False), False),
    "tabb_c2_sep": (TabbConfig(cost="c2", separable=True), False),
    "tabb_c2_sim": (TabbConfig(cost="c2", separable=False), False),
    "tabb_c2_sim_joint": (TabbConfig(cost="c2", separable=False), True),
    "tabb_rp1": (TabbConfig(cost="rp1"), False),
    "tabb_rp1_joint": (TabbConfig(cost="rp1"), True),
    "tabb_rp2": (TabbConfig(cost="rp2"), False),
    # rotation parameterization ablation (논문 Sec. 5.2.3)
    "tabb_c2_sim_euler": (
        TabbConfig(cost="c2", separable=False, rotation="euler"), False),
    "tabb_c2_sim_quat": (
        TabbConfig(cost="c2", separable=False, rotation="quaternion"), False),
    # 대표 설정: 논문 Sec. 5.5의 "best rrmse/rae without reprojection cost"
    "tabb": (TabbConfig(cost="c2", separable=False, rotation="axis_angle"), False),
}
# pixel 관측이 필요한 cost function (class 2)
PIXEL_COST_FUNCTIONS = {"rp1", "rp2"}
TABB_LADDER = [
    "tabb_c1_sep", "tabb_c1_sim", "tabb_c2_sep", "tabb_c2_sim",
    "tabb_c2_sim_joint", "tabb_rp1", "tabb_rp1_joint",
]

ALL_KNOWN_METHODS = (
    set(HAND_EYE_METHODS) | set(ROBOT_WORLD_METHODS) | set(TABB_METHODS)
)

DEFAULT_NOISE_MM = (0.0, 1.0, 3.0, 5.0)
DEFAULT_HELDOUT = (2, 5, 9, 12)

# README 8절의 네 가지 통합 지표 + rotation 보조 지표
INTEGRATED_METRICS = (
    "heldout_translation_error_mm",
    "heldout_rotation_error_deg",
    "camera_pose_translation_error_mm",
    "camera_pose_rotation_error_deg",
    "pairwise_registration_translation_error_mm",
    "pairwise_registration_rotation_error_deg",
    "heldout_reprojection_rmse_px",
)
# README 8.2/11-9: camera별 결과도 원시 CSV에 함께 남긴다.
# registration은 camera pair 지표라 camera 하나로 분해되지 않으므로 제외한다.
PER_CAMERA_METRICS = (
    "camera_pose_translation_error_mm",
    "camera_pose_rotation_error_deg",
    "heldout_translation_error_mm",
    "heldout_rotation_error_deg",
    "heldout_reprojection_rmse_px",
)


def average_transforms(transforms: list[np.ndarray]) -> np.ndarray:
    output = np.eye(4)
    output[:3, :3] = Rotation.from_matrix(
        np.stack([transform[:3, :3] for transform in transforms])
    ).mean().as_matrix()
    output[:3, 3] = np.mean(
        [transform[:3, 3] for transform in transforms], axis=0
    )
    return output


def load_wrist_camera(config_path: Path) -> CameraModel:
    payload = np.load((config_path.parent / "../intrinsics/cam2.npz").resolve())
    return CameraModel(
        name="wrist",
        K=np.asarray(payload["color_K"], dtype=float),
        distortion=np.asarray(payload["color_D"], dtype=float).reshape(-1),
        width=int(payload["color_w"]),
        height=int(payload["color_h"]),
    )


def reprojection_rmse(camera, observed_corners, predicted_pose, board_points):
    observed_pixels, observed_valid = project_points(observed_corners, camera)
    predicted_corners = (
        board_points @ predicted_pose[:3, :3].T + predicted_pose[:3, 3]
    )
    predicted_pixels, predicted_valid = project_points(predicted_corners, camera)
    valid = observed_valid & predicted_valid
    if not np.any(valid):
        return float("nan"), 0
    squared = np.sum((observed_pixels[valid] - predicted_pixels[valid]) ** 2, axis=1)
    return float(np.sqrt(squared.mean())), int(valid.sum())


def provenance() -> dict:
    """사용한 구현과 dependency version을 결과에 함께 남긴다 (README 11-10).

    OpenCV 5개 방법과 Shah는 OpenCV 공개 구현을 그대로 호출하므로 OpenCV 버전이
    곧 알고리즘 버전이다. Tabb는 저자 C++ 코드가 아니라 논문 식을 재구현한 것이라
    저장소 commit hash가 해당 구현을 특정한다.
    """
    import platform
    import subprocess

    import cv2
    import scipy

    root = Path(__file__).resolve().parent.parent
    commit, dirty = None, None
    try:
        commit = subprocess.run(
            ["git", "rev-parse", "HEAD"], cwd=root,
            capture_output=True, text=True, check=True,
        ).stdout.strip()
        dirty = bool(subprocess.run(
            ["git", "status", "--porcelain"], cwd=root,
            capture_output=True, text=True, check=True,
        ).stdout.strip())
    except (OSError, subprocess.SubprocessError):
        pass
    return {
        "repository_commit": commit,
        "repository_dirty": dirty,
        "python": platform.python_version(),
        "numpy": np.__version__,
        "scipy": scipy.__version__,
        "opencv": cv2.__version__,
        "tsai_park_horaud_andreff_daniilidis_shah":
            "OpenCV calib3d calibration_handeye.cpp (public implementation)",
        "tabb":
            "SOTA_Simulation/tabb_solver.py: paper Eq. 5-17 reimplementation, "
            "not the authors' C++/Ceres code",
    }


def pose_error_table(pairs):
    """pair별 (translation mm, rotation deg) 오차. camera별 결과를 남길 때 쓴다."""
    return np.asarray([pose_error(estimate, truth) for estimate, truth in pairs])


def mean_pose_errors(pairs):
    errors = pose_error_table(pairs)
    return float(errors[:, 0].mean()), float(errors[:, 1].mean())


def failure_record(method, trial, noise, train, heldout, camera_names, reason):
    """실패한 trial을 삭제하지 않고 NaN으로 남긴다 (README 11-7).

    통계는 ``summarise()``가 ``status == "ok"`` 행만 골라 계산하고, 실패 건수는
    report.json의 ``failure_count``로 따로 보고한다.
    """
    record = {
        "method": method,
        "trial": trial,
        "noise_sigma_mm": float(noise),
        "status": reason,
        "calibration_pose_count": len(train),
        "heldout_pose_count": len(heldout),
    }
    for metric in INTEGRATED_METRICS:
        record[metric] = float("nan")
    for name in camera_names:
        for metric in PER_CAMERA_METRICS:
            record[f"{name}_{metric}"] = float("nan")
    return record


def build_pixel_observations(corner_sets, camera, board_points, events):
    """Tabb rp1/rp2용 2D 관측을 만든다.

    다른 방법들이 쓰는 pose-level 입력과 **같은 noisy 3D corner**를 그대로
    카메라에 투영한 것이라, noise sample과 trajectory는 공유된다.  다만
    rp1/rp2는 corner 하나하나를 residual로 쓰므로 pose로 요약된 입력보다
    정보량이 많다는 점은 결과 해석에서 분리해서 봐야 한다.
    """
    image_points, point_indices = [], []
    for event in events:
        pixels, valid = project_points(corner_sets[event], camera)
        index = np.flatnonzero(valid)
        point_indices.append(index)
        image_points.append(pixels[index])
    return PixelObservations(
        board_points=board_points,
        K=camera.K,
        distortion=camera.distortion,
        image_points=image_points,
        point_indices=point_indices,
    )


def run_evaluation(case, wrist, wrist_camera, noise_levels, trials, seed,
                   methods, heldout_events):
    fixed = prepare_fixed_inputs(case)
    event_count = len(wrist["T_base_gripper"])
    heldout = sorted(set(heldout_events))
    if any(event < 0 or event >= event_count for event in heldout):
        raise ValueError("held-out event is outside the trajectory")
    train = [event for event in range(event_count) if event not in heldout]
    if len(train) < 3 or not heldout:
        raise ValueError("need at least three calibration poses and one held-out pose")
    board_points = case.data.board_points
    camera_names = sorted(["wrist", *fixed])
    records = []

    for trial in range(trials):
        rng = np.random.default_rng(seed + trial)
        wrist_unit = rng.standard_normal((event_count, len(board_points), 3))
        fixed_unit = {
            name: rng.standard_normal((len(item["visual"]), len(board_points), 3))
            for name, item in fixed.items()
        }
        for noise in noise_levels:
            wrist_poses, wrist_corners = make_corner_observations(
                wrist["T_wrist_board"], board_points, wrist_unit, noise
            )
            fixed_observations = {
                name: make_corner_observations(
                    item["visual"], board_points, fixed_unit[name], noise
                )
                for name, item in fixed.items()
            }
            for method in methods:
                try:
                    if method in ROBOT_WORLD_METHODS:
                        # Shah: AX=YB — cv2.calibrateRobotWorldHandEye 경유
                        # 다른 5개 방법과 달리 eye-to-hand에서 robot pose를 반전하지 않는다.
                        shah_method = ROBOT_WORLD_METHODS[method]
                        wrist_result = solve_shah_eye_in_hand(
                            [wrist["T_base_gripper"][event] for event in train],
                            [wrist_poses[event] for event in train],
                            method=shah_method,
                        )
                        wrist_estimate = wrist_result.T_gripper_wrist
                        fixed_estimates = {}
                        for name, item in fixed.items():
                            fixed_result = solve_shah_eye_to_hand(
                                [item["robot"][event] for event in train],
                                [fixed_observations[name][0][event] for event in train],
                                method=shah_method,
                            )
                            fixed_estimates[name] = fixed_result.T_base_fixed_i

                    elif method in TABB_METHODS:
                        # Tabb: AX=ZB — cost function을 직접 최소화하는 iterative 해.
                        # eye-in-hand에서는 robot pose를 반전해 넣고(논문 B_i 정의가
                        # base->end-effector), eye-to-hand에서는 반전하지 않는다.
                        config, joint = TABB_METHODS[method]
                        needs_pixels = config.cost in PIXEL_COST_FUNCTIONS
                        wrist_pixels = build_pixel_observations(
                            wrist_corners, wrist_camera, board_points, train,
                        ) if needs_pixels else None
                        wrist_estimate = solve_tabb_eye_in_hand(
                            [wrist["T_base_gripper"][event] for event in train],
                            [wrist_poses[event] for event in train],
                            config, wrist_pixels,
                        ).T_gripper_wrist

                        names = sorted(fixed)
                        fixed_pixels = [
                            build_pixel_observations(
                                fixed_observations[name][1],
                                case.data.cameras[name], board_points, train,
                            ) for name in names
                        ] if needs_pixels else None
                        fixed_estimates = solve_tabb_eye_to_hand(
                            names,
                            [[fixed[name]["robot"][event] for event in train]
                             for name in names],
                            [[fixed_observations[name][0][event] for event in train]
                             for name in names],
                            config, fixed_pixels, joint=joint,
                        ).T_base_fixed

                    else:
                        # Tsai/Park/Horaud/Andreff/Daniilidis: AX=XB
                        wrist_estimate = solve_hand_eye(
                            [wrist["T_base_gripper"][event] for event in train],
                            [wrist_poses[event] for event in train],
                            eye_to_hand=False, method=method,
                        )
                        fixed_estimates = {
                            name: solve_hand_eye(
                                [item["robot"][event] for event in train],
                                [fixed_observations[name][0][event] for event in train],
                                eye_to_hand=True, method=method,
                            )
                            for name, item in fixed.items()
                        }

                except Exception as error:  # noqa: BLE001
                    # README 11-7: 실패한 trial을 버리지 않고 NaN + status로 남긴다.
                    records.append(failure_record(
                        method, trial, noise, train, heldout, camera_names,
                        f"solver_error: {type(error).__name__}",
                    ))
                    continue

                reference = heldout[0]
                estimated_cameras = {
                    "wrist": wrist["T_base_gripper"][reference] @ wrist_estimate,
                    **fixed_estimates,
                }
                truth_cameras = {
                    "wrist": wrist["T_base_wrist"][reference],
                    **case.truth.T_base_camera,
                }
                camera_errors = pose_error_table([
                    (estimated_cameras[name], truth_cameras[name])
                    for name in camera_names
                ])
                camera_t = float(camera_errors[:, 0].mean())
                camera_r = float(camera_errors[:, 1].mean())

                relative_pairs = []
                for first, second in combinations(sorted(estimated_cameras), 2):
                    estimate = (
                        invert_transform(estimated_cameras[first])
                        @ estimated_cameras[second]
                    )
                    truth = (
                        invert_transform(truth_cameras[first])
                        @ truth_cameras[second]
                    )
                    relative_pairs.append((estimate, truth))
                registration_t, registration_r = mean_pose_errors(relative_pairs)

                heldout_pairs = []
                heldout_by_camera = {name: [] for name in camera_names}
                for event in heldout:
                    wrist_pair = (
                        wrist["T_base_gripper"][event]
                        @ wrist_estimate @ wrist_poses[event],
                        wrist["T_base_board"],
                    )
                    heldout_pairs.append(wrist_pair)
                    heldout_by_camera["wrist"].append(wrist_pair)
                    for name, item in fixed.items():
                        fixed_pair = (
                            fixed_estimates[name]
                            @ fixed_observations[name][0][event],
                            item["robot"][event] @ case.truth.T_gripper_board,
                        )
                        heldout_pairs.append(fixed_pair)
                        heldout_by_camera[name].append(fixed_pair)
                heldout_t, heldout_r = mean_pose_errors(heldout_pairs)

                estimated_static_board = average_transforms([
                    wrist["T_base_gripper"][event]
                    @ wrist_estimate @ wrist_poses[event]
                    for event in train
                ])
                estimated_mounts = {
                    name: average_transforms([
                        invert_transform(item["robot"][event])
                        @ fixed_estimates[name]
                        @ fixed_observations[name][0][event]
                        for event in train
                    ])
                    for name, item in fixed.items()
                }
                reprojection_squared = []
                reprojection_by_camera = {name: [] for name in camera_names}
                for event in heldout:
                    predicted_wrist_board = (
                        invert_transform(
                            wrist["T_base_gripper"][event] @ wrist_estimate
                        ) @ estimated_static_board
                    )
                    rmse, count = reprojection_rmse(
                        wrist_camera, wrist_corners[event], predicted_wrist_board,
                        board_points,
                    )
                    reprojection_squared.extend([rmse * rmse] * count)
                    reprojection_by_camera["wrist"].extend([rmse * rmse] * count)
                    for name, item in fixed.items():
                        predicted_fixed_board = (
                            invert_transform(fixed_estimates[name])
                            @ item["robot"][event] @ estimated_mounts[name]
                        )
                        rmse, count = reprojection_rmse(
                            case.data.cameras[name],
                            fixed_observations[name][1][event],
                            predicted_fixed_board, board_points,
                        )
                        reprojection_squared.extend([rmse * rmse] * count)
                        reprojection_by_camera[name].extend([rmse * rmse] * count)
                reprojection = float(np.sqrt(np.mean(reprojection_squared)))

                record = {
                    "method": method,
                    "trial": trial,
                    "noise_sigma_mm": float(noise),
                    "status": "ok",
                    "calibration_pose_count": len(train),
                    "heldout_pose_count": len(heldout),
                    "heldout_translation_error_mm": heldout_t,
                    "heldout_rotation_error_deg": heldout_r,
                    "camera_pose_translation_error_mm": camera_t,
                    "camera_pose_rotation_error_deg": camera_r,
                    "pairwise_registration_translation_error_mm": registration_t,
                    "pairwise_registration_rotation_error_deg": registration_r,
                    "heldout_reprojection_rmse_px": reprojection,
                }
                # camera별 결과를 통합 지표와 같은 행에 남긴다 (README 8.2 / 11-9).
                for index, name in enumerate(camera_names):
                    camera_heldout_t, camera_heldout_r = mean_pose_errors(
                        heldout_by_camera[name]
                    )
                    record.update({
                        f"{name}_camera_pose_translation_error_mm":
                            float(camera_errors[index, 0]),
                        f"{name}_camera_pose_rotation_error_deg":
                            float(camera_errors[index, 1]),
                        f"{name}_heldout_translation_error_mm": camera_heldout_t,
                        f"{name}_heldout_rotation_error_deg": camera_heldout_r,
                        f"{name}_heldout_reprojection_rmse_px":
                            float(np.sqrt(np.mean(reprojection_by_camera[name]))),
                    })
                # solver가 예외 없이 NaN/Inf를 돌려주는 경우도 실패로 기록한다.
                if not all(np.isfinite(value) for key, value in record.items()
                           if key not in ("method", "status")):
                    record["status"] = "nonfinite"
                records.append(record)
    return records, train, heldout


def summarise(records, methods, noise_levels):
    """method/noise별 mean과 sample standard deviation (ddof=1).

    실패 trial은 통계에서 빼되 지우지 않고 ``failure_count``로 함께 보고한다
    (README 11-7).
    """
    output = {}
    for method in methods:
        method_output = {}
        for noise in noise_levels:
            selected = [record for record in records
                        if record["method"] == method
                        and record["noise_sigma_mm"] == float(noise)]
            usable = [record for record in selected if record["status"] == "ok"]
            values = {
                "trial_count": len(selected),
                "failure_count": len(selected) - len(usable),
            }
            for metric in INTEGRATED_METRICS:
                data = np.asarray([record[metric] for record in usable])
                values[f"{metric}_mean"] = float(data.mean()) if len(data) else float("nan")
                values[f"{metric}_std"] = float(data.std(ddof=1)) if len(data) > 1 else 0.0
            method_output[str(float(noise))] = values
        output[method] = method_output
    return output


def plot_metrics(summary, methods, noise_levels, output, rotation=False):
    if rotation:
        specs = [
            ("heldout_rotation_error_deg", "Held-out chain error [deg]"),
            ("camera_pose_rotation_error_deg", "Camera pose accuracy [deg]"),
            ("pairwise_registration_rotation_error_deg", "Pairwise registration [deg]"),
        ]
        fig, axes = plt.subplots(1, 3, figsize=(15, 4.3))
        title = "Multi-camera evaluation — rotation metrics"
    else:
        specs = [
            ("heldout_translation_error_mm", "Held-out chain error [mm]"),
            ("camera_pose_translation_error_mm", "Camera position accuracy [mm]"),
            ("pairwise_registration_translation_error_mm", "Pairwise registration [mm]"),
            ("heldout_reprojection_rmse_px", "Held-out reprojection RMSE [px]"),
        ]
        fig, axes = plt.subplots(2, 2, figsize=(12, 9))
        axes = axes.reshape(-1)
        title = "Multi-camera evaluation — integrated primary metrics"
    x = np.asarray(noise_levels, dtype=float)
    for axis, (metric, label) in zip(axes, specs):
        for method in methods:
            mean = np.asarray([summary[method][str(float(n))][f"{metric}_mean"] for n in noise_levels])
            std = np.asarray([summary[method][str(float(n))][f"{metric}_std"] for n in noise_levels])
            axis.errorbar(x, np.where(mean < 1e-6, 0.0, mean),
                          yerr=np.where(std < 1e-6, 0.0, std),
                          marker="o", capsize=3, label=method)
        axis.set_title(label)
        axis.set_xlabel("3D corner noise σ [mm]")
        axis.set_xticks(x)
        axis.grid(alpha=0.25)
    axes[0].legend(title="Method", fontsize=8)
    fig.suptitle(title)
    fig.tight_layout(rect=(0, 0, 1, 0.96))
    fig.savefig(output, dpi=180)
    return fig


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", default=str(Path(__file__).with_name("config.example.json")))
    parser.add_argument("--output", default=str(Path(__file__).with_name("outputs") / "opencv_multicam_metrics"))
    parser.add_argument("--methods", nargs="+", default=["all"])
    parser.add_argument("--noise-mm", nargs="+", type=float, default=list(DEFAULT_NOISE_MM))
    parser.add_argument("--trials", type=int, default=30)
    parser.add_argument("--seed", type=int, default=2026)
    parser.add_argument("--heldout-events", nargs="+", type=int, default=list(DEFAULT_HELDOUT))
    parser.add_argument("--show", action="store_true")
    args = parser.parse_args()
    # "all"은 기존 5개 OpenCV 방법만 뜻한다 (기존 결과 재현성 유지).
    # Shah/Tabb는 이름을 명시해야 실행된다. "tabb_all"은 Tabb ladder로 펼친다.
    methods = []
    for name in args.methods:
        name = name.lower()
        if name == "all":
            methods.extend(HAND_EYE_METHODS)
        elif name == "tabb_all":
            methods.extend(TABB_LADDER)
        else:
            methods.append(name)
    methods = list(dict.fromkeys(methods))
    unknown = sorted(set(methods) - ALL_KNOWN_METHODS)
    if unknown:
        raise SystemExit(
            f"unknown method(s): {unknown}; "
            f"choose from {sorted(ALL_KNOWN_METHODS)} or 'all'/'tabb_all'"
        )

    config, config_path = load_config(args.config)
    config["simulation"]["pixel_noise_sigma"] = 0.0
    config["simulation"]["corner_dropout_probability"] = 0.0
    config["simulation"]["camera_event_dropout_probability"] = 0.0
    case = generate_case(config, config_path)
    wrist = build_eye_in_hand_session(int(config["simulation"]["number_of_events"]))
    wrist_camera = load_wrist_camera(config_path)
    records, train, heldout = run_evaluation(
        case, wrist, wrist_camera, args.noise_mm, args.trials, args.seed,
        methods, args.heldout_events,
    )
    summary = summarise(records, methods, args.noise_mm)
    output = Path(args.output).resolve()
    output.mkdir(parents=True, exist_ok=True)
    report = {
        "methods": methods,
        "noise_levels_mm": args.noise_mm,
        "trials": args.trials,
        "seed": args.seed,
        "calibration_events": train,
        "heldout_events": heldout,
        "wrist_intrinsics": "intrinsics/cam2.npz",
        "provenance": provenance(),
        "metric_scope": {
            "heldout": "mean board-pose chain error over four cameras and held-out poses",
            "camera_pose": "equal-camera mean extrinsic GT error; wrist evaluated in base at the reference pose",
            "registration": "mean GT error of all six pairwise relative camera transforms",
            "reprojection": "pixel RMSE over all held-out corners and all four cameras",
            "per_camera": (
                "records.csv also carries <camera>_* columns for the same metrics "
                "evaluated on one camera; registration is a camera-pair metric and "
                "has no per-camera value"
            ),
            "status": "records.csv 'status' column: ok / nonfinite / solver_error:<type>",
        },
        "summary": summary,
    }
    with (output / "report.json").open("w", encoding="utf-8") as handle:
        json.dump(report, handle, indent=2, ensure_ascii=False)
    # 실패 record는 key 순서가 다를 수 있으므로 처음 본 순서대로 합집합을 만든다.
    fieldnames = []
    for record in records:
        for key in record:
            if key not in fieldnames:
                fieldnames.append(key)
    with (output / "records.csv").open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(records)
    figures = [
        plot_metrics(summary, methods, args.noise_mm, output / "figure1_integrated_metrics.png"),
        plot_metrics(summary, methods, args.noise_mm, output / "figure2_rotation_metrics.png", rotation=True),
    ]
    print(json.dumps(report, indent=2, ensure_ascii=False))
    if args.show:
        plt.show(block=True)
    for figure in figures:
        plt.close(figure)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
