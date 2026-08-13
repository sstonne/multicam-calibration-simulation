"""Method interface, reference implementations, and adapter loading."""

from __future__ import annotations

from importlib import import_module
from typing import Protocol

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

from .sota_simulation import (
    CalibrationInput,
    CalibrationResult,
    invert_transform,
    project_points,
    transform_points,
    transform_to_vector,
    vector_to_transform,
)


class CalibrationMethod(Protocol):
    """Every external SOTA adapter implements this one method."""

    name: str

    def calibrate(self, data: CalibrationInput) -> CalibrationResult:
        ...


def _pixel_residual(
    camera_poses: dict[str, np.ndarray],
    T_gripper_board: np.ndarray,
    data: CalibrationInput,
    observations=None,
) -> np.ndarray:
    selected = data.observations if observations is None else observations
    blocks = []
    for observation in selected:
        T_base_board = data.T_base_gripper[observation.event] @ T_gripper_board
        T_camera_board = invert_transform(camera_poses[observation.camera]) @ T_base_board
        points = data.board_points[observation.point_indices]
        predicted, positive = project_points(
            transform_points(T_camera_board, points),
            data.cameras[observation.camera],
        )
        residual = predicted - observation.image_points
        if not np.all(positive):
            residual[~positive] = 1e3
        blocks.append(residual.reshape(-1))
    return np.concatenate(blocks) if blocks else np.empty(0)


class JointReprojectionReference:
    """Small reference BA used to verify the scaffold, not a paper baseline."""

    name = "joint_reprojection_reference"

    def __init__(self, max_nfev: int = 300):
        self.max_nfev = int(max_nfev)

    def calibrate(self, data: CalibrationInput) -> CalibrationResult:
        camera_names = sorted(data.cameras)
        x0 = np.concatenate([
            *(transform_to_vector(data.initial_T_base_camera[name])
              for name in camera_names),
            transform_to_vector(data.initial_T_gripper_board),
        ])

        def unpack(vector):
            camera_poses = {
                name: vector_to_transform(vector[6 * index:6 * (index + 1)])
                for index, name in enumerate(camera_names)
            }
            T_gripper_board = vector_to_transform(vector[6 * len(camera_names):])
            return camera_poses, T_gripper_board

        def residual(vector):
            return _pixel_residual(*unpack(vector), data)

        solution = least_squares(
            residual,
            x0,
            method="trf",
            loss="linear",
            x_scale="jac",
            ftol=1e-13,
            xtol=1e-13,
            gtol=1e-13,
            max_nfev=self.max_nfev,
        )
        camera_poses, T_gripper_board = unpack(solution.x)
        return CalibrationResult(
            method=self.name,
            T_base_camera=camera_poses,
            T_gripper_board=T_gripper_board,
            success=bool(solution.success),
            diagnostics={
                "message": solution.message,
                "nfev": int(solution.nfev),
                "cost": float(solution.cost),
                "optimality": float(solution.optimality),
            },
        )


def _average_transforms(transforms: list[np.ndarray]) -> np.ndarray:
    output = np.eye(4)
    output[:3, :3] = Rotation.from_matrix(
        [T[:3, :3] for T in transforms]
    ).mean().as_matrix()
    output[:3, 3] = np.mean([T[:3, 3] for T in transforms], axis=0)
    return output


class IndependentReprojectionReference:
    """Run one calibration per camera and assemble results in the base frame."""

    name = "independent_reprojection_reference"

    def __init__(self, max_nfev: int = 300):
        self.max_nfev = int(max_nfev)

    def calibrate(self, data: CalibrationInput) -> CalibrationResult:
        camera_results = {}
        board_results = []
        per_camera = {}
        overall_success = True
        for camera_name in sorted(data.cameras):
            selected = tuple(o for o in data.observations if o.camera == camera_name)
            x0 = np.r_[
                transform_to_vector(data.initial_T_base_camera[camera_name]),
                transform_to_vector(data.initial_T_gripper_board),
            ]

            def residual(vector):
                camera_pose = vector_to_transform(vector[:6])
                board_pose = vector_to_transform(vector[6:12])
                return _pixel_residual(
                    {camera_name: camera_pose}, board_pose, data, selected
                )

            solution = least_squares(
                residual,
                x0,
                method="trf",
                loss="linear",
                x_scale="jac",
                ftol=1e-13,
                xtol=1e-13,
                gtol=1e-13,
                max_nfev=self.max_nfev,
            )
            camera_results[camera_name] = vector_to_transform(solution.x[:6])
            board_results.append(vector_to_transform(solution.x[6:12]))
            overall_success &= bool(solution.success)
            per_camera[camera_name] = {
                "success": bool(solution.success),
                "nfev": int(solution.nfev),
                "cost": float(solution.cost),
                "message": solution.message,
            }

        return CalibrationResult(
            method=self.name,
            T_base_camera=camera_results,
            T_gripper_board=_average_transforms(board_results),
            success=overall_success,
            diagnostics={"per_camera": per_camera},
        )


def load_method(specification: str) -> CalibrationMethod:
    """Load built-ins or ``module.path:factory_or_class`` external adapters."""
    if specification == "joint_reference":
        return JointReprojectionReference()
    if specification == "independent_reference":
        return IndependentReprojectionReference()
    if ":" not in specification:
        raise ValueError(
            "Method must be joint_reference, independent_reference, or module:factory"
        )
    module_name, attribute_name = specification.split(":", 1)
    attribute = getattr(import_module(module_name), attribute_name)
    method = attribute() if callable(attribute) else attribute
    if not hasattr(method, "calibrate"):
        raise TypeError(f"{specification} does not provide calibrate(data)")
    return method

