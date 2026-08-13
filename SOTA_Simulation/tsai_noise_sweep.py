#!/usr/bin/env python3
"""Paired Tsai–Lenz perception-noise sweep with a frozen robot trajectory.

The wrist camera and every fixed camera are calibrated independently.  The
same robot poses, camera ground truth, board geometry and standard-normal
noise samples are reused at all noise levels. Noise is applied to perceived 3D
board corners, from which each T_camera_board is re-estimated.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import os
from pathlib import Path
import sys

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

os.environ.setdefault("QT_API", "PyQt5")

import matplotlib.pyplot as plt
import numpy as np

from SOTA_Simulation.sota_simulation import (
    generate_case,
    load_config,
    pose_error,
    transform_points,
)
from SOTA_Simulation.tsai_combined_demo import (
    HAND_EYE_METHODS,
    _draw_frame,
    _equal_limits,
    build_eye_in_hand_session,
    solve_hand_eye,
)


DEFAULT_NOISE_MM = (0.0, 1.0, 3.0, 5.0)


def estimate_rigid_transform(source_points: np.ndarray,
                             target_points: np.ndarray) -> np.ndarray:
    """Least-squares rigid transform mapping source points to target points."""
    source = np.asarray(source_points, dtype=float)
    target = np.asarray(target_points, dtype=float)
    if source.shape != target.shape or source.ndim != 2 or source.shape[1] != 3:
        raise ValueError("source_points and target_points must both have shape (N, 3)")
    source_centroid = source.mean(axis=0)
    target_centroid = target.mean(axis=0)
    covariance = (source - source_centroid).T @ (target - target_centroid)
    left, _, right_t = np.linalg.svd(covariance)
    rotation = right_t.T @ left.T
    if np.linalg.det(rotation) < 0.0:
        right_t[-1, :] *= -1.0
        rotation = right_t.T @ left.T
    output = np.eye(4)
    output[:3, :3] = rotation
    output[:3, 3] = target_centroid - rotation @ source_centroid
    return output


def add_corner_perception_noise(
    poses: list[np.ndarray],
    board_points: np.ndarray,
    standard_normal: np.ndarray,
    sigma_mm: float,
) -> list[np.ndarray]:
    """Perturb perceived 3D board corners and recover camera-to-board poses.

    ``sigma_mm`` is the standard deviation of each camera-frame x/y/z corner
    coordinate. Re-fitting a rigid board pose produces both translation and
    rotation error.
    """
    expected = (len(poses), len(board_points), 3)
    if standard_normal.shape != expected:
        raise ValueError(f"standard_normal must have shape {expected}")
    scale_m = float(sigma_mm) / 1000.0
    noisy = []
    for pose, sample in zip(poses, standard_normal):
        exact_corners_camera = transform_points(pose, board_points)
        perceived_corners_camera = exact_corners_camera + scale_m * sample
        noisy.append(estimate_rigid_transform(board_points, perceived_corners_camera))
    return noisy


def make_corner_observations(
    poses: list[np.ndarray],
    board_points: np.ndarray,
    standard_normal: np.ndarray,
    sigma_mm: float,
) -> tuple[list[np.ndarray], list[np.ndarray]]:
    """Return fitted poses and the underlying noisy 3D corner observations."""
    expected = (len(poses), len(board_points), 3)
    if standard_normal.shape != expected:
        raise ValueError(f"standard_normal must have shape {expected}")
    scale_m = float(sigma_mm) / 1000.0
    fitted_poses = []
    perceived_corners = []
    for pose, sample in zip(poses, standard_normal):
        exact = transform_points(pose, board_points)
        perceived = exact + scale_m * sample
        fitted_poses.append(estimate_rigid_transform(board_points, perceived))
        perceived_corners.append(perceived)
    return fitted_poses, perceived_corners


def trajectory_digest(poses: list[np.ndarray]) -> str:
    array = np.stack(poses).astype("<f8", copy=False)
    return hashlib.sha256(array.tobytes()).hexdigest()


def prepare_fixed_inputs(case):
    inputs = {}
    for camera_name in sorted(case.data.cameras):
        observations = sorted(
            (item for item in case.data.observations if item.camera == camera_name),
            key=lambda item: item.event,
        )
        inputs[camera_name] = {
            "robot": [case.data.T_base_gripper[item.event] for item in observations],
            "visual": [item.T_camera_board_exact for item in observations],
            "events": [item.event for item in observations],
        }
    return inputs


def run_sweep(case, wrist_session, noise_levels_mm, trials: int, seed: int,
              methods=("tsai",)):
    """Run a paired sweep; all experimental factors except perception noise stay fixed."""
    fixed_inputs = prepare_fixed_inputs(case)
    camera_names = ["wrist", *sorted(fixed_inputs)]
    records = []

    for trial in range(trials):
        rng = np.random.default_rng(seed + trial)
        # Draw once per trial and scale the same samples at every noise level.
        number_of_points = len(case.data.board_points)
        wrist_unit_noise = rng.standard_normal(
            (len(wrist_session["T_wrist_board"]), number_of_points, 3)
        )
        fixed_unit_noise = {
            name: rng.standard_normal(
                (len(fixed_inputs[name]["visual"]), number_of_points, 3)
            )
            for name in sorted(fixed_inputs)
        }
        for noise_mm in noise_levels_mm:
            wrist_visual = add_corner_perception_noise(
                wrist_session["T_wrist_board"], case.data.board_points,
                wrist_unit_noise, noise_mm
            )
            fixed_visual = {
                name: add_corner_perception_noise(
                    item["visual"], case.data.board_points,
                    fixed_unit_noise[name], noise_mm,
                )
                for name, item in fixed_inputs.items()
            }
            for method in methods:
                wrist_estimate = solve_hand_eye(
                    wrist_session["T_base_gripper"], wrist_visual,
                    eye_to_hand=False, method=method,
                )
                translation_mm, rotation_deg = pose_error(
                    wrist_estimate, wrist_session["T_gripper_wrist_truth"]
                )
                records.append({
                    "method": method,
                    "trial": trial,
                    "noise_sigma_mm": float(noise_mm),
                    "camera": "wrist",
                    "camera_type": "eye_in_hand",
                    "translation_error_mm": translation_mm,
                    "rotation_error_deg": rotation_deg,
                })
                for name in sorted(fixed_inputs):
                    item = fixed_inputs[name]
                    estimate = solve_hand_eye(
                        item["robot"], fixed_visual[name], eye_to_hand=True,
                        method=method,
                    )
                    translation_mm, rotation_deg = pose_error(
                        estimate, case.truth.T_base_camera[name]
                    )
                    records.append({
                        "method": method,
                        "trial": trial,
                        "noise_sigma_mm": float(noise_mm),
                        "camera": name,
                        "camera_type": "eye_to_hand",
                        "translation_error_mm": translation_mm,
                        "rotation_error_deg": rotation_deg,
                    })
    return records, camera_names, fixed_inputs


def summarise(records, camera_names, noise_levels_mm, methods):
    summary = {}
    for method in methods:
        method_summary = {}
        for noise in noise_levels_mm:
            level = {}
            for camera in camera_names:
                selected = [
                    record for record in records
                    if record["method"] == method
                    and record["noise_sigma_mm"] == float(noise)
                    and record["camera"] == camera
                ]
                translation = np.asarray(
                    [record["translation_error_mm"] for record in selected]
                )
                rotation = np.asarray(
                    [record["rotation_error_deg"] for record in selected]
                )
                level[camera] = {
                    "translation_mean_mm": float(translation.mean()),
                    "translation_std_mm": float(translation.std(ddof=1))
                        if len(translation) > 1 else 0.0,
                    "rotation_mean_deg": float(rotation.mean()),
                    "rotation_std_deg": float(rotation.std(ddof=1))
                        if len(rotation) > 1 else 0.0,
                }
            method_summary[str(float(noise))] = level
        summary[method] = method_summary
    return summary


def aggregate_system_results(records, camera_names, noise_levels_mm, methods):
    """Equal-camera macro average and worst-camera error for every trial."""
    integrated_records = []
    for method in methods:
        for noise in noise_levels_mm:
            trials = sorted({
                record["trial"] for record in records
                if record["method"] == method
                and record["noise_sigma_mm"] == float(noise)
            })
            for trial in trials:
                selected = [
                    record for record in records
                    if record["method"] == method
                    and record["noise_sigma_mm"] == float(noise)
                    and record["trial"] == trial
                ]
                if {record["camera"] for record in selected} != set(camera_names):
                    raise RuntimeError("A system aggregate requires every camera result")
                translation = np.asarray([
                    record["translation_error_mm"] for record in selected
                ])
                rotation = np.asarray([
                    record["rotation_error_deg"] for record in selected
                ])
                integrated_records.append({
                    "method": method,
                    "trial": trial,
                    "noise_sigma_mm": float(noise),
                    "number_of_cameras": len(selected),
                    "macro_translation_error_mm": float(translation.mean()),
                    "macro_rotation_error_deg": float(rotation.mean()),
                    "worst_translation_error_mm": float(translation.max()),
                    "worst_rotation_error_deg": float(rotation.max()),
                })

    integrated_summary = {}
    metrics = (
        "macro_translation_error_mm",
        "macro_rotation_error_deg",
        "worst_translation_error_mm",
        "worst_rotation_error_deg",
    )
    for method in methods:
        method_summary = {}
        for noise in noise_levels_mm:
            selected = [
                record for record in integrated_records
                if record["method"] == method
                and record["noise_sigma_mm"] == float(noise)
            ]
            values = {}
            for metric in metrics:
                data = np.asarray([record[metric] for record in selected])
                values[f"{metric}_mean"] = float(data.mean())
                values[f"{metric}_std"] = (
                    float(data.std(ddof=1)) if len(data) > 1 else 0.0
                )
            method_summary[str(float(noise))] = values
        integrated_summary[method] = method_summary
    return integrated_records, integrated_summary


def plot_setup(case, wrist_session, output):
    fig = plt.figure(figsize=(13.0, 5.8))

    # Eye-in-hand experiment: its own robot trajectory, reused by every method.
    wrist_axis = fig.add_subplot(121, projection="3d")
    _draw_frame(wrist_axis, np.eye(4), "base", 0.08)
    wrist_camera = np.asarray([
        transform[:3, 3] for transform in wrist_session["T_base_wrist"]
    ])
    wrist_gripper = np.asarray([
        transform[:3, 3] for transform in wrist_session["T_base_gripper"]
    ])
    static_board = wrist_session["T_base_board"][:3, 3]
    wrist_axis.plot(*wrist_camera.T, "o-", color="#4e79a7", markersize=3,
                    label="wrist-camera trajectory")
    wrist_axis.plot(*wrist_gripper.T, ".--", color="0.35", markersize=2,
                    label="eye-in-hand robot trajectory")
    wrist_axis.scatter(*static_board, marker="s", s=90, color="#f28e2b",
                       label="static board")
    for index in range(0, len(wrist_session["T_base_wrist"]), 4):
        _draw_frame(wrist_axis, wrist_session["T_base_wrist"][index], None,
                    0.035, 0.55)
    _equal_limits(
        wrist_axis,
        np.vstack([wrist_camera, wrist_gripper, static_board.reshape(1, 3)]),
    )
    wrist_axis.set_title("Eye-in-hand: wrist camera + static board")
    wrist_axis.legend(loc="upper right", fontsize=8)

    # Eye-to-hand experiment: a separate trajectory, likewise reused by methods.
    axis = fig.add_subplot(122, projection="3d")
    _draw_frame(axis, np.eye(4), "base", 0.08)
    positions = [np.zeros(3)]
    for name, transform in case.truth.T_base_camera.items():
        _draw_frame(axis, transform, name, 0.065)
        positions.append(transform[:3, 3])
    events = sorted(case.data.T_base_gripper)
    gripper = np.asarray([case.data.T_base_gripper[event][:3, 3] for event in events])
    board = np.asarray([
        (case.data.T_base_gripper[event] @ case.truth.T_gripper_board)[:3, 3]
        for event in events
    ])
    axis.plot(*gripper.T, ".--", color="0.35",
              label="eye-to-hand robot trajectory")
    axis.plot(*board.T, "o-", color="#f28e2b", markersize=3,
              label="frozen board trajectory")
    _equal_limits(axis, np.vstack([np.asarray(positions), gripper, board]))
    axis.set_title("Eye-to-hand: fixed cameras + moving board")
    axis.legend(loc="upper right", fontsize=8)
    fig.suptitle(
        "Two calibration trajectories — each is frozen across methods and noise levels"
    )
    fig.tight_layout(rect=(0, 0, 1, 0.985))
    fig.savefig(output, dpi=180)
    return fig


def plot_results(summary, camera_names, noise_levels_mm, methods, output):
    fig, axes = plt.subplots(len(methods), 2, figsize=(12, 3.7 * len(methods)),
                             squeeze=False)
    colours = plt.cm.tab10(np.linspace(0, 1, len(camera_names)))
    x = np.asarray(noise_levels_mm, dtype=float)
    for row, method in enumerate(methods):
        method_summary = summary[method]
        for camera, colour in zip(camera_names, colours):
            mean_t = np.asarray([method_summary[str(float(n))][camera]["translation_mean_mm"] for n in noise_levels_mm])
            std_t = np.asarray([method_summary[str(float(n))][camera]["translation_std_mm"] for n in noise_levels_mm])
            mean_r = np.asarray([method_summary[str(float(n))][camera]["rotation_mean_deg"] for n in noise_levels_mm])
            std_r = np.asarray([method_summary[str(float(n))][camera]["rotation_std_deg"] for n in noise_levels_mm])
            axes[row, 0].errorbar(x, mean_t, yerr=std_t, marker="o", capsize=2,
                                  label=camera, color=colour)
            axes[row, 1].errorbar(x, np.where(mean_r < 1e-6, 0.0, mean_r),
                                  yerr=np.where(std_r < 1e-6, 0.0, std_r),
                                  marker="o", capsize=2, label=camera, color=colour)
        axes[row, 0].set_ylabel(f"{method}\ntranslation [mm]")
        axes[row, 1].set_ylabel(f"{method}\nrotation [deg]")
        for axis in axes[row]:
            axis.set_xlabel("3D corner noise σ [mm]")
            axis.set_xticks(x)
            axis.grid(alpha=0.25)
    axes[0, 0].set_title("Extrinsic translation error: mean ± std")
    axes[0, 1].set_title("Extrinsic rotation error: mean ± std")
    axes[0, 1].legend(title="camera", ncol=2)
    fig.suptitle(
        "OpenCV hand-eye methods — identical observations and trajectories",
        y=0.998,
    )
    fig.tight_layout(rect=(0, 0, 1, 0.97))
    fig.savefig(output, dpi=180)
    return fig


def plot_distributions(records, camera_names, noise_levels_mm, output):
    fig, axes = plt.subplots(1, len(camera_names), figsize=(14, 3.8), sharey=True)
    for axis, camera in zip(axes, camera_names):
        groups = [
            [record["translation_error_mm"] for record in records
             if record["camera"] == camera
             and record["noise_sigma_mm"] == float(noise)]
            for noise in noise_levels_mm
        ]
        axis.boxplot(groups, tick_labels=[f"{noise:g}" for noise in noise_levels_mm],
                     showfliers=True)
        axis.set_title(camera)
        axis.set_xlabel("noise σ [mm]")
        axis.grid(axis="y", alpha=0.25)
    axes[0].set_ylabel("translation error [mm]")
    fig.suptitle("Trial distribution (the trajectory is never regenerated)")
    fig.tight_layout()
    fig.savefig(output, dpi=180)
    return fig


def plot_integrated_results(integrated_summary, methods, noise_levels_mm, output):
    fig, axes = plt.subplots(1, 2, figsize=(11.5, 4.7))
    x = np.asarray(noise_levels_mm, dtype=float)
    for method in methods:
        summary = integrated_summary[method]
        translation = np.asarray([
            summary[str(float(noise))]["macro_translation_error_mm_mean"]
            for noise in noise_levels_mm
        ])
        translation_std = np.asarray([
            summary[str(float(noise))]["macro_translation_error_mm_std"]
            for noise in noise_levels_mm
        ])
        rotation = np.asarray([
            summary[str(float(noise))]["macro_rotation_error_deg_mean"]
            for noise in noise_levels_mm
        ])
        rotation_std = np.asarray([
            summary[str(float(noise))]["macro_rotation_error_deg_std"]
            for noise in noise_levels_mm
        ])
        axes[0].errorbar(x, translation, yerr=translation_std, marker="o",
                         capsize=3, label=method)
        axes[1].errorbar(x, np.where(rotation < 1e-6, 0.0, rotation),
                         yerr=np.where(rotation_std < 1e-6, 0.0, rotation_std),
                         marker="o", capsize=3, label=method)
    axes[0].set_ylabel("4-camera macro translation error [mm]")
    axes[1].set_ylabel("4-camera macro rotation error [deg]")
    for axis in axes:
        axis.set_xlabel("3D corner noise σ [mm]")
        axis.set_xticks(x)
        axis.grid(alpha=0.25)
    axes[0].set_title("System translation: mean ± std")
    axes[1].set_title("System rotation: mean ± std")
    axes[1].legend(title="OpenCV method")
    fig.suptitle(
        "System-level aggregate — equal-weight mean of wrist, cam0, cam1 and cam3"
    )
    fig.tight_layout(rect=(0, 0, 1, 0.94))
    fig.savefig(output, dpi=180)
    return fig


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--config", default=str(Path(__file__).with_name("config.example.json"))
    )
    parser.add_argument(
        "--output",
        default=str(Path(__file__).with_name("outputs") / "tsai_noise_sweep"),
    )
    parser.add_argument("--noise-mm", nargs="+", type=float,
                        default=list(DEFAULT_NOISE_MM))
    parser.add_argument("--trials", type=int, default=30)
    parser.add_argument("--methods", nargs="+", default=["all"])
    parser.add_argument("--seed", type=int, default=2026)
    parser.add_argument("--show", action="store_true")
    args = parser.parse_args()
    if args.trials < 1 or any(noise < 0 for noise in args.noise_mm):
        parser.error("trials must be positive and noise levels must be non-negative")
    methods = list(HAND_EYE_METHODS) if args.methods == ["all"] else [m.lower() for m in args.methods]
    unknown = sorted(set(methods) - set(HAND_EYE_METHODS))
    if unknown:
        parser.error(f"unknown methods {unknown}; choose from {sorted(HAND_EYE_METHODS)} or all")

    config, config_path = load_config(args.config)
    config["simulation"]["pixel_noise_sigma"] = 0.0
    config["simulation"]["corner_dropout_probability"] = 0.0
    config["simulation"]["camera_event_dropout_probability"] = 0.0
    case = generate_case(config, config_path)
    wrist = build_eye_in_hand_session(int(config["simulation"]["number_of_events"]))

    records, camera_names, fixed_inputs = run_sweep(
        case, wrist, args.noise_mm, args.trials, args.seed, methods
    )
    summary = summarise(records, camera_names, args.noise_mm, methods)
    integrated_records, integrated_summary = aggregate_system_results(
        records, camera_names, args.noise_mm, methods
    )
    robot_poses = [case.data.T_base_gripper[event]
                   for event in sorted(case.data.T_base_gripper)]
    report = {
        "methods": methods,
        "noise_levels_mm": [float(value) for value in args.noise_mm],
        "perception_noise_model": {
            "target": "each board corner perceived in the camera 3D frame",
            "distribution": "zero-mean Gaussian",
            "sigma_definition": (
                "standard deviation per camera-frame x/y/z corner coordinate"
            ),
            "pose_recovery": (
                "least-squares rigid registration of board points to noisy corners"
            ),
            "effect": "the recovered T_camera_board has translation and rotation error",
        },
        "controlled_variables": {
            "trajectory_policy": (
                "eye-in-hand and eye-to-hand use separate trajectories; each "
                "trajectory is identical across methods, noise levels and trials"
            ),
            "robot_pose_noise": 0.0,
            "camera_ground_truth_noise": 0.0,
            "pixel_noise": 0.0,
            "paired_noise_samples": True,
            "robot_trajectory_sha256": trajectory_digest(robot_poses),
            "wrist_trajectory_sha256": trajectory_digest(
                wrist["T_base_gripper"]
            ),
        },
        "trials": args.trials,
        "seed_sequence": [args.seed, args.seed + args.trials - 1],
        "calibrations": {
            "wrist": "eye-in-hand, solved independently",
            **{name: "eye-to-hand, solved independently"
               for name in sorted(fixed_inputs)},
        },
        "summary": summary,
        "integrated_metric_definition": {
            "macro": "equal-weight arithmetic mean over wrist, cam0, cam1 and cam3 within each trial",
            "worst": "maximum camera error within each trial",
            "note": "system-level aggregation of independent calibrations; not a joint calibration estimate",
        },
        "integrated_summary": integrated_summary,
        "records": records,
    }

    output = Path(args.output).resolve()
    output.mkdir(parents=True, exist_ok=True)
    with (output / "report.json").open("w", encoding="utf-8") as handle:
        json.dump(report, handle, indent=2, ensure_ascii=False)
    with (output / "records.csv").open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(records[0]))
        writer.writeheader()
        writer.writerows(records)
    with (output / "integrated_records.csv").open(
        "w", encoding="utf-8", newline=""
    ) as handle:
        writer = csv.DictWriter(handle, fieldnames=list(integrated_records[0]))
        writer.writeheader()
        writer.writerows(integrated_records)
    figures = [
        plot_setup(case, wrist, output / "figure1_fixed_setup.png"),
        plot_results(summary, camera_names, args.noise_mm, methods,
                     output / "figure2_noise_results.png"),
        plot_distributions([r for r in records if r["method"] == methods[0]], camera_names, args.noise_mm,
                           output / "figure3_trial_distributions.png"),
        plot_integrated_results(
            integrated_summary, methods, args.noise_mm,
            output / "figure4_integrated_results.png",
        ),
    ]

    print(json.dumps({
        "output": str(output),
        "trajectory_sha256": report["controlled_variables"]["robot_trajectory_sha256"],
        "trials": args.trials,
        "summary": summary,
    }, indent=2, ensure_ascii=False))
    if args.show:
        print("Visualization windows opened. Close all windows to finish.")
        plt.show(block=True)
    for figure in figures:
        plt.close(figure)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
