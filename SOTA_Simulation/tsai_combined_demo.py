#!/usr/bin/env python3
"""Noiseless Tsai–Lenz demo: eye-in-hand + eye-to-hand + merged camera rig.

This is an educational example, not a new multi-camera algorithm.  Tsai–Lenz
is run once for the wrist camera and independently once per fixed camera.  The
estimated transforms are then expressed in the robot base frame and visualized.
"""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import sys

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

# OpenCV from conda-forge currently brings Qt6 as well.  Matplotlib must use the
# explicitly installed PyQt5 binding to avoid mixing the two plugin families.
os.environ.setdefault("QT_API", "PyQt5")

import cv2
import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial.transform import Rotation

from SOTA_Simulation.sota_simulation import (
    SyntheticCase,
    generate_case,
    invert_transform,
    load_config,
    look_at,
    pose_error,
    transform_points,
)


def make_transform(rpy_deg, translation_m) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = Rotation.from_euler("xyz", rpy_deg, degrees=True).as_matrix()
    T[:3, 3] = np.asarray(translation_m, dtype=float)
    return T


HAND_EYE_METHODS = {
    "tsai": cv2.CALIB_HAND_EYE_TSAI,
    "park": cv2.CALIB_HAND_EYE_PARK,
    "horaud": cv2.CALIB_HAND_EYE_HORAUD,
    "andreff": cv2.CALIB_HAND_EYE_ANDREFF,
    "daniilidis": cv2.CALIB_HAND_EYE_DANIILIDIS,
}


def solve_hand_eye(
    robot_poses: list[np.ndarray],
    T_camera_target: list[np.ndarray],
    eye_to_hand: bool,
    method: str = "tsai",
) -> np.ndarray:
    """Run an OpenCV hand-eye solver with explicit input conversion.

    Eye-in-hand:
      input robot poses = T_base_gripper
      output            = T_gripper_camera

    Eye-to-hand:
      input robot poses = T_gripper_base = inverse(T_base_gripper)
      output            = T_base_camera

    In both cases the visual input is T_camera_target.
    """
    method_name = method.lower()
    if method_name not in HAND_EYE_METHODS:
        raise ValueError(
            f"Unknown method {method!r}; choose from {sorted(HAND_EYE_METHODS)}"
        )
    robot_input = [invert_transform(T) for T in robot_poses] if eye_to_hand \
        else robot_poses
    rotations_robot = [T[:3, :3].copy() for T in robot_input]
    translations_robot = [T[:3, 3].reshape(3, 1).copy() for T in robot_input]
    rotations_target = [T[:3, :3].copy() for T in T_camera_target]
    translations_target = [T[:3, 3].reshape(3, 1).copy() for T in T_camera_target]
    rotation, translation = cv2.calibrateHandEye(
        rotations_robot,
        translations_robot,
        rotations_target,
        translations_target,
        method=HAND_EYE_METHODS[method_name],
    )
    output = np.eye(4)
    output[:3, :3] = rotation
    output[:3, 3] = np.asarray(translation).reshape(3)
    return output


def solve_tsai(robot_poses, T_camera_target, eye_to_hand):
    """Backward-compatible alias used by the focused Tsai teaching demo."""
    return solve_hand_eye(robot_poses, T_camera_target, eye_to_hand, "tsai")


def build_eye_in_hand_session(number_of_poses: int = 14):
    """Static horizontal board observed by a moving wrist camera."""
    T_base_board = make_transform([0.0, 0.0, 12.0], [0.0, 0.02, 0.08])
    T_gripper_wrist_truth = make_transform([4.0, -7.0, 3.0], [0.035, -0.008, 0.065])
    robot_poses = []
    camera_target_poses = []
    wrist_poses = []
    target = T_base_board[:3, 3]
    for index in range(number_of_poses):
        phase = 2.0 * np.pi * index / number_of_poses
        camera_position = target + np.array([
            0.22 * np.sin(phase),
            -0.04 + 0.18 * np.cos(phase),
            0.34 + 0.07 * np.sin(2.0 * phase + 0.3),
        ])
        aim = target + np.array([
            0.035 * np.sin(2.0 * phase),
            0.025 * np.cos(3.0 * phase),
            0.0,
        ])
        T_base_wrist = look_at(camera_position, aim)
        roll = Rotation.from_euler("z", 22.0 * np.sin(3.0 * phase), degrees=True)
        T_base_wrist[:3, :3] = T_base_wrist[:3, :3] @ roll.as_matrix()
        T_base_gripper = T_base_wrist @ invert_transform(T_gripper_wrist_truth)
        T_wrist_board = invert_transform(T_base_wrist) @ T_base_board
        robot_poses.append(T_base_gripper)
        wrist_poses.append(T_base_wrist)
        camera_target_poses.append(T_wrist_board)
    return {
        "T_base_board": T_base_board,
        "T_gripper_wrist_truth": T_gripper_wrist_truth,
        "T_base_gripper": robot_poses,
        "T_base_wrist": wrist_poses,
        "T_wrist_board": camera_target_poses,
    }


def solve_eye_to_hand(case: SyntheticCase):
    estimates = {}
    inputs = {}
    for camera_name in sorted(case.data.cameras):
        observations = sorted(
            (item for item in case.data.observations if item.camera == camera_name),
            key=lambda item: item.event,
        )
        robot_poses = [case.data.T_base_gripper[item.event] for item in observations]
        target_poses = [item.T_camera_board_exact for item in observations]
        estimates[camera_name] = solve_tsai(
            robot_poses, target_poses, eye_to_hand=True
        )
        inputs[camera_name] = {
            "events": [item.event for item in observations],
            "T_camera_board": target_poses,
        }
    return estimates, inputs


def _draw_frame(axis, T, label, length=0.055, alpha=1.0, linewidth=1.3):
    origin = T[:3, 3]
    for column, color in enumerate(("#d62728", "#2ca02c", "#1f77b4")):
        vector = T[:3, column] * length
        axis.quiver(
            *origin, *vector, color=color, alpha=alpha,
            arrow_length_ratio=0.2, linewidth=linewidth,
        )
    if label:
        axis.text(*origin, label, fontsize=8)


def _board_outline(board_points: np.ndarray) -> np.ndarray:
    minimum = board_points[:, :2].min(axis=0)
    maximum = board_points[:, :2].max(axis=0)
    return np.array([
        [minimum[0], minimum[1], 0.0],
        [maximum[0], minimum[1], 0.0],
        [maximum[0], maximum[1], 0.0],
        [minimum[0], maximum[1], 0.0],
        [minimum[0], minimum[1], 0.0],
    ])


def _equal_limits(axis, points: np.ndarray):
    centre = points.mean(axis=0)
    radius = max(0.22, 0.56 * np.ptp(points, axis=0).max())
    axis.set_xlim(centre[0] - radius, centre[0] + radius)
    axis.set_ylim(centre[1] - radius, centre[1] + radius)
    axis.set_zlim(max(0.0, centre[2] - radius), centre[2] + radius)
    axis.set_xlabel("base x [m]")
    axis.set_ylabel("base y [m]")
    axis.set_zlabel("base z [m]")


def plot_eye_in_hand(session, estimate, board_points, output):
    fig = plt.figure(figsize=(8.5, 6.8))
    axis = fig.add_subplot(111, projection="3d")
    _draw_frame(axis, np.eye(4), "base", 0.08)
    outline = transform_points(
        session["T_base_board"], _board_outline(board_points)
    )
    axis.plot(*outline.T, color="#f28e2b", linewidth=3, label="static board")
    camera_positions = np.asarray([T[:3, 3] for T in session["T_base_wrist"]])
    gripper_positions = np.asarray([T[:3, 3] for T in session["T_base_gripper"]])
    axis.plot(*camera_positions.T, "o-", color="#4e79a7", markersize=3,
              label="wrist-camera trajectory")
    axis.plot(*gripper_positions.T, ".--", color="0.35", markersize=2,
              label="gripper trajectory")
    for index in range(0, len(session["T_base_wrist"]), 3):
        _draw_frame(axis, session["T_base_wrist"][index], None, 0.035, 0.65)
    # Show the recovered mounting transform at pose 0 in the same base frame.
    T_base_wrist_estimate = session["T_base_gripper"][0] @ estimate
    _draw_frame(axis, session["T_base_wrist"][0], "wrist (GT = estimate)", 0.065)
    _draw_frame(axis, T_base_wrist_estimate, None, 0.045, 0.45)
    points = np.vstack([camera_positions, gripper_positions, outline])
    _equal_limits(axis, points)
    axis.set_title(
        "Step 1 — Eye-in-hand Tsai–Lenz\n"
        "Static board + moving wrist camera → T_gripper_wrist"
    )
    axis.legend(loc="upper right")
    fig.tight_layout()
    fig.savefig(output, dpi=180)
    return fig


def plot_eye_to_hand(case, estimates, output):
    fig = plt.figure(figsize=(8.5, 6.8))
    axis = fig.add_subplot(111, projection="3d")
    _draw_frame(axis, np.eye(4), "base", 0.08)
    for camera_name, truth in case.truth.T_base_camera.items():
        _draw_frame(axis, truth, f"{camera_name} GT", 0.065)
        _draw_frame(axis, estimates[camera_name], None, 0.045, 0.4)
    outline_local = _board_outline(case.data.board_points)
    events = sorted(case.data.T_base_gripper)
    board_centres = []
    for event in events:
        T_base_board = (
            case.data.T_base_gripper[event] @ case.truth.T_gripper_board
        )
        board_centres.append(T_base_board[:3, 3])
    board_centres = np.asarray(board_centres)
    axis.plot(*board_centres.T, "k.-", linewidth=0.8, markersize=3,
              label="moving board trajectory")
    for event in events[::max(1, len(events) // 6)]:
        T_base_board = (
            case.data.T_base_gripper[event] @ case.truth.T_gripper_board
        )
        outline = transform_points(T_base_board, outline_local)
        axis.plot(*outline.T, color="#f28e2b", alpha=0.65)
    camera_positions = np.asarray([
        T[:3, 3] for T in case.truth.T_base_camera.values()
    ])
    _equal_limits(axis, np.vstack([camera_positions, board_centres]))
    axis.set_title(
        "Step 2 — Eye-to-hand Tsai–Lenz, once per fixed camera\n"
        "Board on gripper + fixed camera → T_base_fixed_i"
    )
    axis.legend(loc="upper right")
    fig.tight_layout()
    fig.savefig(output, dpi=180)
    return fig


def plot_merged(case, fixed_estimates, wrist_session, wrist_estimate, output):
    reference_event = 0
    T_base_gripper = wrist_session["T_base_gripper"][reference_event]
    T_base_wrist_estimate = T_base_gripper @ wrist_estimate
    fig = plt.figure(figsize=(8.5, 6.8))
    axis = fig.add_subplot(111, projection="3d")
    _draw_frame(axis, np.eye(4), "base", 0.085)
    positions = [np.zeros(3)]
    for camera_name, estimate in fixed_estimates.items():
        _draw_frame(axis, estimate, camera_name, 0.07)
        positions.append(estimate[:3, 3])
    _draw_frame(axis, T_base_wrist_estimate, "wrist@pose0", 0.07)
    positions.append(T_base_wrist_estimate[:3, 3])
    _equal_limits(axis, np.asarray(positions))
    axis.set_title(
        "Step 3 — All calibrated cameras in the robot base frame\n"
        "Fixed: T_base_fixed_i | Wrist at pose 0: T_base_gripper(0) @ T_gripper_wrist"
    )
    fig.tight_layout()
    fig.savefig(output, dpi=180)
    return fig


def plot_summary(wrist_error, fixed_errors, output):
    names = ["wrist"] + list(fixed_errors)
    translation = [wrist_error[0]] + [fixed_errors[name][0] for name in fixed_errors]
    rotation = [wrist_error[1]] + [fixed_errors[name][1] for name in fixed_errors]
    fig, axes = plt.subplots(1, 2, figsize=(10, 4))
    axes[0].bar(names, translation, color="#4e79a7")
    axes[0].set_ylabel("translation error [mm]")
    axes[0].set_title("GT translation error")
    axes[1].bar(names, rotation, color="#f28e2b")
    axes[1].set_ylabel("rotation error [deg]")
    axes[1].set_title("GT rotation error")
    for axis in axes:
        axis.grid(axis="y", alpha=0.25)
    fig.suptitle("Noiseless Tsai–Lenz recovery (machine-precision zero expected)")
    fig.tight_layout()
    fig.savefig(output, dpi=180)
    return fig


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--config",
        default=str(Path(__file__).with_name("config.example.json")),
    )
    parser.add_argument(
        "--output",
        default=str(Path(__file__).with_name("outputs") / "tsai_combined_noiseless"),
    )
    parser.add_argument("--show", action="store_true")
    args = parser.parse_args()

    config, config_path = load_config(args.config)
    # This demo is deliberately noiseless.  Do not inherit noise settings from
    # an edited robustness config.
    config["simulation"]["pixel_noise_sigma"] = 0.0
    config["simulation"]["corner_dropout_probability"] = 0.0
    config["simulation"]["camera_event_dropout_probability"] = 0.0
    fixed_case = generate_case(config, config_path)

    wrist_session = build_eye_in_hand_session(
        int(config["simulation"]["number_of_events"])
    )
    T_gripper_wrist_estimate = solve_tsai(
        wrist_session["T_base_gripper"],
        wrist_session["T_wrist_board"],
        eye_to_hand=False,
    )
    fixed_estimates, fixed_inputs = solve_eye_to_hand(fixed_case)

    wrist_error = pose_error(
        T_gripper_wrist_estimate,
        wrist_session["T_gripper_wrist_truth"],
    )
    fixed_errors = {
        name: pose_error(estimate, fixed_case.truth.T_base_camera[name])
        for name, estimate in fixed_estimates.items()
    }
    max_translation = max([wrist_error[0], *(v[0] for v in fixed_errors.values())])
    max_rotation = max([wrist_error[1], *(v[1] for v in fixed_errors.values())])
    passed = max_translation < 1e-5 and max_rotation < 1e-6

    output = Path(args.output).resolve()
    output.mkdir(parents=True, exist_ok=True)
    report = {
        "demo": "Tsai-Lenz eye-in-hand + eye-to-hand + base-frame merge",
        "noise": 0.0,
        "transform_convention": "T_destination_source",
        "eye_in_hand": {
            "input_robot_pose": "T_base_gripper",
            "input_visual_pose": "T_wrist_board",
            "output": "T_gripper_wrist",
            "translation_error_mm": wrist_error[0],
            "rotation_error_deg": wrist_error[1],
        },
        "eye_to_hand": {
            "input_robot_pose": "T_gripper_base = inverse(T_base_gripper)",
            "input_visual_pose": "T_fixed_board",
            "output": "T_base_fixed",
            "per_camera": {
                name: {
                    "events": fixed_inputs[name]["events"],
                    "translation_error_mm": fixed_errors[name][0],
                    "rotation_error_deg": fixed_errors[name][1],
                }
                for name in fixed_errors
            },
        },
        "merge": {
            "fixed_camera_in_base": "T_base_fixed_i",
            "wrist_camera_in_base_at_event_k":
                "T_base_gripper(k) @ T_gripper_wrist",
        },
        "passed": passed,
    }
    with (output / "report.json").open("w", encoding="utf-8") as handle:
        json.dump(report, handle, indent=2, ensure_ascii=False)

    figures = [
        plot_eye_in_hand(
            wrist_session, T_gripper_wrist_estimate,
            fixed_case.data.board_points, output / "step1_eye_in_hand.png",
        ),
        plot_eye_to_hand(
            fixed_case, fixed_estimates, output / "step2_eye_to_hand.png",
        ),
        plot_merged(
            fixed_case, fixed_estimates, wrist_session,
            T_gripper_wrist_estimate, output / "step3_merged.png",
        ),
        plot_summary(
            wrist_error, fixed_errors, output / "step4_errors.png",
        ),
    ]

    print(json.dumps(report, indent=2, ensure_ascii=False))
    print(f"\nOutput: {output}")
    print("Noiseless Tsai combined gate:", "PASS" if passed else "FAIL")
    if args.show:
        print("Visualization windows opened. Close all windows to finish.")
        plt.show(block=True)
    for figure in figures:
        plt.close(figure)
    return 0 if passed else 2


if __name__ == "__main__":
    raise SystemExit(main())
