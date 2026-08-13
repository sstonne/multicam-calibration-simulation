"""Synthetic board-on-end-effector dataset for calibration method adapters.

Transform notation follows ``T_destination_source``.  For example,
``T_base_camera`` maps points from a camera frame into the robot base frame.
All distances are stored in metres and image measurements in pixels.
"""

from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path
from typing import Any

import numpy as np
from scipy.spatial.transform import Rotation


@dataclass(frozen=True)
class CameraModel:
    name: str
    K: np.ndarray
    distortion: np.ndarray
    width: int
    height: int


@dataclass(frozen=True)
class BoardObservation:
    camera: str
    event: int
    point_indices: np.ndarray
    image_points: np.ndarray
    T_camera_board_exact: np.ndarray


@dataclass(frozen=True)
class CalibrationInput:
    """Only this object is passed to a student/SOTA method; it contains no GT."""

    board_points: np.ndarray
    cameras: dict[str, CameraModel]
    T_base_gripper: dict[int, np.ndarray]
    observations: tuple[BoardObservation, ...]
    initial_T_base_camera: dict[str, np.ndarray]
    initial_T_gripper_board: np.ndarray
    metadata: dict[str, Any]


@dataclass(frozen=True)
class GroundTruth:
    T_base_camera: dict[str, np.ndarray]
    T_gripper_board: np.ndarray


@dataclass(frozen=True)
class SyntheticCase:
    data: CalibrationInput
    truth: GroundTruth


@dataclass(frozen=True)
class CalibrationResult:
    method: str
    T_base_camera: dict[str, np.ndarray]
    T_gripper_board: np.ndarray | None
    success: bool
    diagnostics: dict[str, Any]


def invert_transform(T: np.ndarray) -> np.ndarray:
    R = np.asarray(T, dtype=float)[:3, :3]
    t = np.asarray(T, dtype=float)[:3, 3]
    output = np.eye(4)
    output[:3, :3] = R.T
    output[:3, 3] = -R.T @ t
    return output


def transform_points(T_destination_source: np.ndarray, points: np.ndarray) -> np.ndarray:
    points = np.asarray(points, dtype=float)
    return points @ T_destination_source[:3, :3].T + T_destination_source[:3, 3]


def transform_to_vector(T: np.ndarray) -> np.ndarray:
    return np.r_[Rotation.from_matrix(T[:3, :3]).as_rotvec(), T[:3, 3]]


def vector_to_transform(vector: np.ndarray) -> np.ndarray:
    vector = np.asarray(vector, dtype=float)
    T = np.eye(4)
    T[:3, :3] = Rotation.from_rotvec(vector[:3]).as_matrix()
    T[:3, 3] = vector[3:6]
    return T


def pose_error(estimate: np.ndarray, truth: np.ndarray) -> tuple[float, float]:
    delta = invert_transform(truth) @ estimate
    translation_mm = 1000.0 * np.linalg.norm(delta[:3, 3])
    rotation_deg = np.degrees(np.linalg.norm(
        Rotation.from_matrix(delta[:3, :3]).as_rotvec()))
    return float(translation_mm), float(rotation_deg)


def look_at(camera_position: np.ndarray, target: np.ndarray) -> np.ndarray:
    """Return T_base_camera using the OpenCV camera convention (+z optical axis)."""
    position = np.asarray(camera_position, dtype=float)
    z_axis = np.asarray(target, dtype=float) - position
    z_axis /= np.linalg.norm(z_axis)
    world_up = np.array([0.0, 0.0, 1.0])
    x_axis = np.cross(world_up, z_axis)
    if np.linalg.norm(x_axis) < 1e-8:
        world_up = np.array([0.0, 1.0, 0.0])
        x_axis = np.cross(world_up, z_axis)
    x_axis /= np.linalg.norm(x_axis)
    y_axis = np.cross(z_axis, x_axis)
    T = np.eye(4)
    T[:3, :3] = np.column_stack([x_axis, y_axis, z_axis])
    T[:3, 3] = position
    return T


def project_points(
    points_camera: np.ndarray,
    camera: CameraModel,
) -> tuple[np.ndarray, np.ndarray]:
    """OpenCV-compatible five-parameter radial/tangential projection."""
    points = np.asarray(points_camera, dtype=float)
    z = points[:, 2]
    valid = z > 1e-8
    x = np.zeros_like(z)
    y = np.zeros_like(z)
    x[valid] = points[valid, 0] / z[valid]
    y[valid] = points[valid, 1] / z[valid]

    d = np.zeros(5, dtype=float)
    source = np.asarray(camera.distortion, dtype=float).reshape(-1)
    d[: min(5, len(source))] = source[:5]
    k1, k2, p1, p2, k3 = d
    r2 = x * x + y * y
    radial = 1.0 + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2
    x_distorted = x * radial + 2.0 * p1 * x * y + p2 * (r2 + 2.0 * x * x)
    y_distorted = y * radial + p1 * (r2 + 2.0 * y * y) + 2.0 * p2 * x * y
    u = camera.K[0, 0] * x_distorted + camera.K[0, 1] * y_distorted + camera.K[0, 2]
    v = camera.K[1, 1] * y_distorted + camera.K[1, 2]
    image = np.column_stack([u, v])
    valid &= (u >= 0) & (u < camera.width) & (v >= 0) & (v < camera.height)
    return image, valid


def board_points_from_config(board: dict[str, Any]) -> np.ndarray:
    squares_x = int(board["squares_x"])
    squares_y = int(board["squares_y"])
    square = float(board["square_length_m"])
    if squares_x < 2 or squares_y < 2 or square <= 0:
        raise ValueError("Board requires squares_x/y >= 2 and positive square_length_m")
    # ChArUco inner chessboard corners, centred at the physical board centre.
    points = [
        [ix * square - squares_x * square / 2.0,
         iy * square - squares_y * square / 2.0,
         0.0]
        for iy in range(1, squares_y)
        for ix in range(1, squares_x)
    ]
    return np.asarray(points, dtype=float)


def _load_camera(config: dict[str, Any], root: Path) -> CameraModel:
    name = str(config["name"])
    if "intrinsics_npz" in config:
        path = (root / config["intrinsics_npz"]).resolve()
        payload = np.load(path)
        K = np.asarray(payload["color_K"], dtype=float)
        distortion = np.asarray(payload["color_D"], dtype=float).reshape(-1)
        width = int(payload["color_w"])
        height = int(payload["color_h"])
    else:
        K = np.asarray(config["K"], dtype=float)
        distortion = np.asarray(config.get("distortion", np.zeros(5)), dtype=float)
        width, height = map(int, config["image_size"])
    return CameraModel(name, K, distortion, width, height)


def _pose(rotation_matrix: np.ndarray, translation: np.ndarray) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = rotation_matrix
    T[:3, 3] = translation
    return T


def _perturb_transform(
    T: np.ndarray,
    rng: np.random.Generator,
    translation_mm: float,
    rotation_deg: float,
) -> np.ndarray:
    direction = rng.normal(size=3)
    direction /= np.linalg.norm(direction)
    delta = _pose(
        Rotation.from_rotvec(direction * np.deg2rad(rotation_deg)).as_matrix(),
        rng.normal(0.0, translation_mm / 1000.0, size=3),
    )
    return T @ delta


def generate_case(config: dict[str, Any], config_path: Path) -> SyntheticCase:
    seed = int(config["simulation"].get("seed", 7))
    rng = np.random.default_rng(seed)
    board_points = board_points_from_config(config["board"])
    root = config_path.parent

    cameras: dict[str, CameraModel] = {}
    T_base_camera: dict[str, np.ndarray] = {}
    for camera_config in config["cameras"]:
        camera = _load_camera(camera_config, root)
        cameras[camera.name] = camera
        T_base_camera[camera.name] = look_at(
            np.asarray(camera_config["position_base_m"], dtype=float),
            np.asarray(camera_config["look_at_base_m"], dtype=float),
        )

    gripper_board_config = config["T_gripper_board"]
    T_gripper_board = _pose(
        Rotation.from_euler(
            "xyz", gripper_board_config["rpy_deg"], degrees=True).as_matrix(),
        np.asarray(gripper_board_config["translation_m"], dtype=float),
    )

    number_of_events = int(config["simulation"]["number_of_events"])
    centre = np.asarray(config["simulation"]["workspace_center_base_m"], dtype=float)
    span = np.asarray(config["simulation"]["workspace_span_m"], dtype=float)
    # Board +z roughly faces the fixed cameras on the -y side of the table.
    board_nominal_rotation = np.column_stack([
        np.array([1.0, 0.0, 0.0]),
        np.array([0.0, 0.28734789, 0.95782629]),
        np.array([0.0, -0.95782629, 0.28734789]),
    ])
    T_base_gripper: dict[int, np.ndarray] = {}
    for event in range(number_of_events):
        phase = 2.0 * np.pi * event / number_of_events
        translation = centre + np.array([
            0.5 * span[0] * np.sin(phase),
            0.5 * span[1] * np.sin(2.0 * phase + 0.3),
            0.5 * span[2] * np.cos(phase + 0.4),
        ])
        deterministic = np.array([
            18.0 * np.sin(phase + 0.2),
            24.0 * np.cos(2.0 * phase),
            30.0 * np.sin(3.0 * phase + 0.5),
        ])
        jitter = rng.normal(0.0, 2.0, size=3)
        board_rotation = board_nominal_rotation @ Rotation.from_euler(
            "xyz", deterministic + jitter, degrees=True).as_matrix()
        T_base_board = _pose(board_rotation, translation)
        T_base_gripper[event] = T_base_board @ invert_transform(T_gripper_board)

    simulation = config["simulation"]
    noise_px = float(simulation.get("pixel_noise_sigma", 0.0))
    dropout = float(simulation.get("corner_dropout_probability", 0.0))
    camera_dropout = float(simulation.get("camera_event_dropout_probability", 0.0))
    minimum_corners = int(simulation.get("minimum_visible_corners", 8))
    observations: list[BoardObservation] = []
    for event, T_base_gripper_event in T_base_gripper.items():
        T_base_board = T_base_gripper_event @ T_gripper_board
        for camera_name, camera in cameras.items():
            if camera_dropout > 0 and rng.random() < camera_dropout:
                continue
            T_camera_board = invert_transform(T_base_camera[camera_name]) @ T_base_board
            points_camera = transform_points(T_camera_board, board_points)
            pixels, visible = project_points(points_camera, camera)
            visible &= rng.random(len(board_points)) >= dropout
            indices = np.flatnonzero(visible)
            if len(indices) < minimum_corners:
                continue
            observed = pixels[indices].copy()
            if noise_px > 0:
                observed += rng.normal(0.0, noise_px, size=observed.shape)
            observations.append(BoardObservation(
                camera=camera_name,
                event=event,
                point_indices=indices,
                image_points=observed,
                T_camera_board_exact=T_camera_board,
            ))

    missing = [name for name in cameras if not any(o.camera == name for o in observations)]
    if missing:
        raise RuntimeError(f"No visible board observations for cameras: {missing}")

    initial = config["initialization"]
    initial_cameras = {
        name: _perturb_transform(
            transform, rng,
            float(initial["translation_sigma_mm"]),
            float(initial["rotation_sigma_deg"]),
        )
        for name, transform in T_base_camera.items()
    }
    initial_gripper_board = _perturb_transform(
        T_gripper_board,
        rng,
        float(initial["translation_sigma_mm"]),
        float(initial["rotation_sigma_deg"]),
    )
    metadata = {
        "transform_convention": "T_destination_source",
        "length_unit": "metre",
        "image_unit": "pixel",
        "board": config["board"],
        "seed": seed,
    }
    data = CalibrationInput(
        board_points=board_points,
        cameras=cameras,
        T_base_gripper=T_base_gripper,
        observations=tuple(observations),
        initial_T_base_camera=initial_cameras,
        initial_T_gripper_board=initial_gripper_board,
        metadata=metadata,
    )
    return SyntheticCase(data, GroundTruth(T_base_camera, T_gripper_board))


def load_config(path: str | Path) -> tuple[dict[str, Any], Path]:
    config_path = Path(path).resolve()
    with config_path.open("r", encoding="utf-8") as handle:
        return json.load(handle), config_path


def export_input_npz(data: CalibrationInput, path: str | Path) -> None:
    """Export the common input without GT for external repositories/adapters."""
    camera_names = list(data.cameras)
    camera_index = {name: index for index, name in enumerate(camera_names)}
    offsets = [0]
    point_indices: list[np.ndarray] = []
    image_points: list[np.ndarray] = []
    for observation in data.observations:
        point_indices.append(observation.point_indices)
        image_points.append(observation.image_points)
        offsets.append(offsets[-1] + len(observation.point_indices))
    np.savez_compressed(
        path,
        board_points=data.board_points,
        camera_names=np.asarray(camera_names),
        camera_K=np.stack([data.cameras[name].K for name in camera_names]),
        camera_distortion=np.stack([
            np.pad(data.cameras[name].distortion.reshape(-1), (0, max(
                0, 5 - data.cameras[name].distortion.size)))[:5]
            for name in camera_names
        ]),
        image_size=np.asarray([
            [data.cameras[name].width, data.cameras[name].height]
            for name in camera_names
        ]),
        event_ids=np.asarray(sorted(data.T_base_gripper)),
        T_base_gripper=np.stack([
            data.T_base_gripper[event] for event in sorted(data.T_base_gripper)
        ]),
        observation_camera_index=np.asarray([
            camera_index[o.camera] for o in data.observations
        ]),
        observation_event=np.asarray([o.event for o in data.observations]),
        observation_T_camera_board_exact=np.stack([
            o.T_camera_board_exact for o in data.observations
        ]),
        observation_offsets=np.asarray(offsets),
        observation_point_indices=np.concatenate(point_indices),
        observation_image_points=np.concatenate(image_points),
    )
