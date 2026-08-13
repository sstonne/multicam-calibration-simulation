"""Ground-truth and reprojection evaluation for a calibration result."""

from __future__ import annotations

from typing import Any

import numpy as np

from .sota_simulation import (
    CalibrationResult,
    SyntheticCase,
    invert_transform,
    pose_error,
    project_points,
    transform_points,
)


def evaluate_result(case: SyntheticCase, result: CalibrationResult) -> dict[str, Any]:
    per_camera = {}
    residuals = []
    for camera_name, truth in case.truth.T_base_camera.items():
        if camera_name not in result.T_base_camera:
            per_camera[camera_name] = {"available": False}
            continue
        translation_mm, rotation_deg = pose_error(
            result.T_base_camera[camera_name], truth
        )
        per_camera[camera_name] = {
            "available": True,
            "translation_error_mm": translation_mm,
            "rotation_error_deg": rotation_deg,
        }

    if result.T_gripper_board is not None:
        board_translation_mm, board_rotation_deg = pose_error(
            result.T_gripper_board, case.truth.T_gripper_board
        )
        board_error = {
            "available": True,
            "translation_error_mm": board_translation_mm,
            "rotation_error_deg": board_rotation_deg,
        }
        for observation in case.data.observations:
            if observation.camera not in result.T_base_camera:
                continue
            T_base_board = (
                case.data.T_base_gripper[observation.event]
                @ result.T_gripper_board
            )
            T_camera_board = (
                invert_transform(result.T_base_camera[observation.camera])
                @ T_base_board
            )
            predicted, _ = project_points(
                transform_points(
                    T_camera_board,
                    case.data.board_points[observation.point_indices],
                ),
                case.data.cameras[observation.camera],
            )
            residuals.append((predicted - observation.image_points).reshape(-1))
    else:
        board_error = {"available": False}

    available = [value for value in per_camera.values() if value["available"]]
    reprojection = np.concatenate(residuals) if residuals else np.empty(0)
    return {
        "method": result.method,
        "success": bool(result.success),
        "number_of_cameras": len(case.data.cameras),
        "number_of_observations": len(case.data.observations),
        "per_camera": per_camera,
        "camera_translation_rmse_mm": float(np.sqrt(np.mean([
            value["translation_error_mm"] ** 2 for value in available
        ]))) if available else None,
        "camera_rotation_rmse_deg": float(np.sqrt(np.mean([
            value["rotation_error_deg"] ** 2 for value in available
        ]))) if available else None,
        "T_gripper_board_error": board_error,
        "reprojection_rmse_px": float(np.sqrt(np.mean(reprojection ** 2)))
        if reprojection.size else None,
        "diagnostics": result.diagnostics,
    }


def noiseless_pass(report: dict[str, Any], tolerance: dict[str, float]) -> bool:
    """Predeclared numerical gate; literal floating-point zero is not required."""
    values = [
        report.get("camera_translation_rmse_mm"),
        report.get("camera_rotation_rmse_deg"),
        report.get("reprojection_rmse_px"),
    ]
    if not report.get("success") or any(value is None for value in values):
        return False
    board = report["T_gripper_board_error"]
    return bool(
        report["camera_translation_rmse_mm"] <= tolerance["translation_mm"]
        and report["camera_rotation_rmse_deg"] <= tolerance["rotation_deg"]
        and board["translation_error_mm"] <= tolerance["translation_mm"]
        and board["rotation_error_deg"] <= tolerance["rotation_deg"]
        and report["reprojection_rmse_px"] <= tolerance["reprojection_px"]
    )

