"""Diagnostic visualizations shared by all calibration methods."""

from __future__ import annotations

from pathlib import Path

import matplotlib
import matplotlib.pyplot as plt
import numpy as np

from .sota_simulation import (
    CalibrationResult,
    SyntheticCase,
    invert_transform,
    project_points,
    transform_points,
)


def _draw_frame(axis, T, label, length=0.06, alpha=1.0):
    origin = T[:3, 3]
    colors = ("#d62728", "#2ca02c", "#1f77b4")
    for index, color in enumerate(colors):
        direction = T[:3, index] * length
        axis.quiver(*origin, *direction, color=color, alpha=alpha,
                    arrow_length_ratio=0.2, linewidth=1.2)
    if label:
        axis.text(*origin, label, fontsize=8)


def plot_scene(case: SyntheticCase, result: CalibrationResult, output: Path) -> None:
    fig = plt.figure(figsize=(9, 7))
    axis = fig.add_subplot(111, projection="3d")
    _draw_frame(axis, np.eye(4), "base", length=0.09)
    for camera_name, truth in case.truth.T_base_camera.items():
        _draw_frame(axis, truth, f"{camera_name} GT", length=0.07)
        if camera_name in result.T_base_camera:
            _draw_frame(
                axis, result.T_base_camera[camera_name],
                None, length=0.05, alpha=0.45,
            )

    events = sorted(case.data.T_base_gripper)
    trajectory = np.asarray([
        case.data.T_base_gripper[event][:3, 3] for event in events
    ])
    axis.plot(*trajectory.T, "k.-", linewidth=0.8, markersize=3,
              label="gripper trajectory")
    board_xy = case.data.board_points[:, :2]
    x0, y0 = board_xy.min(axis=0)
    x1, y1 = board_xy.max(axis=0)
    outline = np.array([
        [x0, y0, 0], [x1, y0, 0], [x1, y1, 0], [x0, y1, 0], [x0, y0, 0]
    ])
    stride = max(1, len(events) // 6)
    for event in events[::stride]:
        T_base_board = (
            case.data.T_base_gripper[event] @ case.truth.T_gripper_board
        )
        world_outline = transform_points(T_base_board, outline)
        axis.plot(*world_outline.T, color="#ff7f0e", alpha=0.55)

    all_positions = np.vstack([
        trajectory,
        np.asarray([T[:3, 3] for T in case.truth.T_base_camera.values()]),
    ])
    centre = all_positions.mean(axis=0)
    radius = 0.55 * np.ptp(all_positions, axis=0).max()
    radius = max(radius, 0.25)
    axis.set_xlim(centre[0] - radius, centre[0] + radius)
    axis.set_ylim(centre[1] - radius, centre[1] + radius)
    axis.set_zlim(max(0.0, centre[2] - radius), centre[2] + radius)
    axis.set_xlabel("base x [m]")
    axis.set_ylabel("base y [m]")
    axis.set_zlabel("base z [m]")
    axis.set_title("Synthetic board-on-end-effector setup\nGT frames: opaque, estimates: transparent")
    axis.legend(loc="upper right")
    fig.tight_layout()
    fig.savefig(output, dpi=180)
    return fig


def plot_reprojection(
    case: SyntheticCase,
    result: CalibrationResult,
    output: Path,
) -> None:
    camera_names = sorted(case.data.cameras)
    fig, axes = plt.subplots(
        1, len(camera_names), figsize=(5 * len(camera_names), 4), squeeze=False
    )
    for axis, camera_name in zip(axes[0], camera_names):
        candidates = [o for o in case.data.observations if o.camera == camera_name]
        observation = max(candidates, key=lambda item: len(item.point_indices))
        camera = case.data.cameras[camera_name]
        axis.scatter(
            observation.image_points[:, 0], observation.image_points[:, 1],
            s=20, facecolors="none", edgecolors="#1f77b4", label="observed",
        )
        if result.T_gripper_board is not None and camera_name in result.T_base_camera:
            T_base_board = (
                case.data.T_base_gripper[observation.event]
                @ result.T_gripper_board
            )
            T_camera_board = (
                invert_transform(result.T_base_camera[camera_name]) @ T_base_board
            )
            predicted, _ = project_points(
                transform_points(
                    T_camera_board,
                    case.data.board_points[observation.point_indices],
                ),
                camera,
            )
            axis.scatter(predicted[:, 0], predicted[:, 1], s=8,
                         color="#d62728", label="estimated")
            for observed, estimate in zip(observation.image_points, predicted):
                axis.plot([observed[0], estimate[0]], [observed[1], estimate[1]],
                          color="0.6", linewidth=0.5)
        axis.set_xlim(0, camera.width)
        axis.set_ylim(camera.height, 0)
        axis.set_aspect("equal", adjustable="box")
        axis.set_title(f"{camera_name}, event {observation.event}")
        axis.set_xlabel("u [px]")
        axis.set_ylabel("v [px]")
        axis.grid(alpha=0.2)
        axis.legend()
    fig.suptitle("Observed vs. reprojected board corners")
    fig.tight_layout()
    fig.savefig(output, dpi=180)
    return fig


def plot_errors(report: dict, output: Path) -> None:
    names = [name for name, value in report["per_camera"].items()
             if value["available"]]
    translations = [report["per_camera"][name]["translation_error_mm"]
                    for name in names]
    rotations = [report["per_camera"][name]["rotation_error_deg"]
                 for name in names]
    fig, axes = plt.subplots(1, 2, figsize=(9, 3.8))
    axes[0].bar(names, translations, color="#4c78a8")
    axes[0].set_ylabel("translation error [mm]")
    axes[0].set_title("Camera translation")
    axes[1].bar(names, rotations, color="#f58518")
    axes[1].set_ylabel("rotation error [deg]")
    axes[1].set_title("Camera rotation")
    for axis in axes:
        axis.grid(axis="y", alpha=0.25)
    fig.suptitle(
        f"{report['method']} | reprojection RMSE "
        f"{report['reprojection_rmse_px']:.3g} px"
    )
    fig.tight_layout()
    fig.savefig(output, dpi=180)
    return fig


def create_visualizations(
    case: SyntheticCase,
    result: CalibrationResult,
    report: dict,
    output_directory: Path,
    show: bool = False,
) -> None:
    output_directory.mkdir(parents=True, exist_ok=True)
    figures = [
        plot_scene(case, result, output_directory / "scene_3d.png"),
        plot_reprojection(case, result, output_directory / "reprojection.png"),
        plot_errors(report, output_directory / "calibration_errors.png"),
    ]
    if show:
        print("Visualization windows opened. Close all windows to finish.")
        plt.show(block=True)
    for figure in figures:
        plt.close(figure)
