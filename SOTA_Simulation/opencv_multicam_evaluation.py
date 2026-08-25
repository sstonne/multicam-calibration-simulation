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

# Shah는 AX=YB를 풀므로 HAND_EYE_METHODS에 섞지 않고 별도 set으로 관리
ROBOT_WORLD_METHODS = {"shah": "SHAH", "li": "LI"}
ALL_KNOWN_METHODS = set(HAND_EYE_METHODS) | set(ROBOT_WORLD_METHODS)

DEFAULT_NOISE_MM = (0.0, 1.0, 3.0, 5.0)
DEFAULT_HELDOUT = (2, 5, 9, 12)


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


def mean_pose_errors(pairs):
    errors = np.asarray([pose_error(estimate, truth) for estimate, truth in pairs])
    return float(errors[:, 0].mean()), float(errors[:, 1].mean())


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

                reference = heldout[0]
                estimated_cameras = {
                    "wrist": wrist["T_base_gripper"][reference] @ wrist_estimate,
                    **fixed_estimates,
                }
                truth_cameras = {
                    "wrist": wrist["T_base_wrist"][reference],
                    **case.truth.T_base_camera,
                }
                camera_t, camera_r = mean_pose_errors([
                    (estimated_cameras[name], truth_cameras[name])
                    for name in sorted(estimated_cameras)
                ])

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
                for event in heldout:
                    heldout_pairs.append((
                        wrist["T_base_gripper"][event]
                        @ wrist_estimate @ wrist_poses[event],
                        wrist["T_base_board"],
                    ))
                    for name, item in fixed.items():
                        heldout_pairs.append((
                            fixed_estimates[name]
                            @ fixed_observations[name][0][event],
                            item["robot"][event] @ case.truth.T_gripper_board,
                        ))
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
                reprojection = float(np.sqrt(np.mean(reprojection_squared)))

                records.append({
                    "method": method,
                    "trial": trial,
                    "noise_sigma_mm": float(noise),
                    "calibration_pose_count": len(train),
                    "heldout_pose_count": len(heldout),
                    "heldout_translation_error_mm": heldout_t,
                    "heldout_rotation_error_deg": heldout_r,
                    "camera_pose_translation_error_mm": camera_t,
                    "camera_pose_rotation_error_deg": camera_r,
                    "pairwise_registration_translation_error_mm": registration_t,
                    "pairwise_registration_rotation_error_deg": registration_r,
                    "heldout_reprojection_rmse_px": reprojection,
                })
    return records, train, heldout


def summarise(records, methods, noise_levels):
    metric_names = [
        "heldout_translation_error_mm",
        "heldout_rotation_error_deg",
        "camera_pose_translation_error_mm",
        "camera_pose_rotation_error_deg",
        "pairwise_registration_translation_error_mm",
        "pairwise_registration_rotation_error_deg",
        "heldout_reprojection_rmse_px",
    ]
    output = {}
    for method in methods:
        method_output = {}
        for noise in noise_levels:
            selected = [record for record in records
                        if record["method"] == method
                        and record["noise_sigma_mm"] == float(noise)]
            values = {}
            for metric in metric_names:
                data = np.asarray([record[metric] for record in selected])
                values[f"{metric}_mean"] = float(data.mean())
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
    axes[0].legend(title="OpenCV method")
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
    methods = list(HAND_EYE_METHODS) if args.methods == ["all"] else [name.lower() for name in args.methods]
    if args.methods == ["all"]:
        methods = list(HAND_EYE_METHODS)
    else:
        methods = [name.lower() for name in args.methods]
    unknown = sorted(set(methods) - ALL_KNOWN_METHODS)

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
        "calibration_events": train,
        "heldout_events": heldout,
        "wrist_intrinsics": "intrinsics/cam2.npz",
        "metric_scope": {
            "heldout": "mean board-pose chain error over four cameras and held-out poses",
            "camera_pose": "equal-camera mean extrinsic GT error; wrist evaluated in base at the reference pose",
            "registration": "mean GT error of all six pairwise relative camera transforms",
            "reprojection": "pixel RMSE over all held-out corners and all four cameras",
        },
        "summary": summary,
    }
    with (output / "report.json").open("w", encoding="utf-8") as handle:
        json.dump(report, handle, indent=2, ensure_ascii=False)
    with (output / "records.csv").open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(records[0]))
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
