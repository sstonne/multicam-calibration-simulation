"""Tabb & Ahmad Yousef (2017) iterative robot-world/hand-eye(s) calibration.

논문: A. Tabb and K. M. Ahmad Yousef, "Solving the robot-world hand-eye(s)
calibration problem with iterative methods", Machine Vision and Applications
28(5-6):569-590, 2017.  doi:10.1007/s00138-017-0841-7  (arXiv:1907.12425)
저자 구현: https://github.com/amy-tabb/RWHEC-Tabb-AhmadYousef

앞선 방법들과의 결정적 차이
---------------------------------------------------------------------------
- Tsai/Park/Horaud/Andreff/Daniilidis : AX = XB, 미지수 1개, closed-form.
- Shah(2013)                          : AX = YB, 미지수 2개, closed-form
                                        (Kronecker product + SVD, separable).
- Tabb(2017)                          : AX = ZB, 미지수 2개, **iterative**.
                                        cost function을 직접 정의하고
                                        Levenberg-Marquardt로 최소화한다.

즉 Tabb는 "새로운 대수적 해법"이 아니라 **무엇을 최소화할 것인가(cost
function)를 바꾸는 방법론 모음**이다. 논문은 2개 class의 cost function,
3가지 rotation parameterization, separable/simultaneous 두 형태를 조합한다.

논문의 표기 (Sec. 2)
---------------------------------------------------------------------------
    A_i X = Z B_i                                             (Eq. 2)

    A_i : world(=calibration board) -> camera   변환
    B_i : robot base -> end-effector            변환
    X   : robot base -> world                   (robot-world)
    Z   : end-effector -> camera                (hand-eye)
    X~  := X^{-1}  (c2 / rp1 / rp2 에서 실제로 추정하는 파라미터)

구현된 cost function
---------------------------------------------------------------------------
  class 1 (pose-level, A·B 를 직접 사용):
    c1 : sum_i ||A_i X - Z B_i||_F^2                          (Eq. 5, 6)
         separable 버전은 rotation(Eq. 7) -> translation(Eq. 8) 순서.
    c2 : sum_i ||A_i - Z B_i X~||_F^2                         (Eq. 9, 10)
         separable 버전은 rotation(Eq. 11) -> translation(Eq. 12) 순서.
  class 2 (pixel-level, A 를 쓰지 않고 corner 관측을 직접 사용):
    rp1: sum_i sum_j ||x_ij - f(k, [Z B_i X~]_{3x4} X_j)||^2  (Eq. 16)
    rp2: rp1 + intrinsic k 까지 동시 추정                      (Eq. 17)

separable 버전의 translation 단계(Eq. 8, Eq. 12)는 미지수에 대해 선형이므로
반복 없이 linear least squares로 정확히 푼다.

초기값 (논문 Sec. 3.1)
---------------------------------------------------------------------------
  - c1 / c2 : R = I, t = 0.  (논문 각주 2: 1st class는 초기값에 둔감)
  - rp1     : c2 simultaneous 해를 초기값으로 사용.
  - rp2     : rp1 해를 초기값으로 사용.

multi-eye (논문 Sec. 2.3, Eq. 19-23)
---------------------------------------------------------------------------
    A_{i,d} X = Z_d B_i      (d = 0..q-1)
  X 하나를 q대의 카메라가 **공유**하고 Z_d만 카메라마다 따로 둔다.
  카메라별 관측 수가 다를 때는 가중치 w_d = min_e |S_e| / |S_d| 로
  각 카메라의 영향력을 같게 맞춘다 (Eq. 23).

이 저장소 transform으로의 매핑은 아래 solve_tabb_eye_in_hand() /
solve_tabb_eye_to_hand() docstring 참고.  self_test()로 zero-noise
복원을 항상 재검증할 수 있다.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Iterable, Sequence

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

ROTATION_PARAMETERIZATIONS = ("euler", "axis_angle", "quaternion")
COST_FUNCTIONS = ("c1", "c2", "rp1", "rp2")

# 논문은 Ceres의 Levenberg-Marquardt를 쓴다. scipy에서 같은 알고리즘 계열은
# MINPACK LM인 method="lm" 이고, "trf"는 trust-region reflective다.
_DEFAULT_OPTIMIZER = "lm"


# ----------------------------------------------------------------------
# 4x4 <-> (R, t) 유틸  (shah_solver.py와 동일 규칙)
# ----------------------------------------------------------------------

def _to_h(R: np.ndarray, t: np.ndarray) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = np.asarray(t, dtype=float).reshape(3)
    return T


def _inv(T: np.ndarray) -> np.ndarray:
    R = np.asarray(T, dtype=float)[:3, :3]
    t = np.asarray(T, dtype=float)[:3, 3]
    output = np.eye(4)
    output[:3, :3] = R.T
    output[:3, 3] = -R.T @ t
    return output


def _stack_Rt(transforms: Sequence[np.ndarray]) -> tuple[np.ndarray, np.ndarray]:
    array = np.asarray(transforms, dtype=float)
    return array[:, :3, :3].copy(), array[:, :3, 3].copy()


# ----------------------------------------------------------------------
# rotation parameterization (논문 Sec. 2 첫 문단)
#   euler      : p in R^3, R = Rz Ry Rx  (논문이 고른 x, y, z 순서)
#   axis_angle : p in R^3, Rodrigues
#   quaternion : p in R^4, 내부에서 정규화
# ----------------------------------------------------------------------

def _rotation_from_parameters(p: np.ndarray, kind: str) -> np.ndarray:
    if kind == "euler":
        # scipy의 소문자 "xyz"는 extrinsic 회전이라 R = Rz @ Ry @ Rx 가 된다.
        return Rotation.from_euler("xyz", np.asarray(p, dtype=float)).as_matrix()
    if kind == "axis_angle":
        return Rotation.from_rotvec(np.asarray(p, dtype=float)).as_matrix()
    if kind == "quaternion":
        q = np.asarray(p, dtype=float)
        norm = np.linalg.norm(q)
        if norm < 1e-12:
            return np.eye(3)
        return Rotation.from_quat(q / norm).as_matrix()
    raise ValueError(f"unknown rotation parameterization {kind!r}")


def _parameters_from_rotation(R: np.ndarray, kind: str) -> np.ndarray:
    rotation = Rotation.from_matrix(np.asarray(R, dtype=float))
    if kind == "euler":
        return rotation.as_euler("xyz")
    if kind == "axis_angle":
        return rotation.as_rotvec()
    if kind == "quaternion":
        return rotation.as_quat()
    raise ValueError(f"unknown rotation parameterization {kind!r}")


def _identity_parameters(kind: str) -> np.ndarray:
    if kind == "quaternion":
        return np.array([0.0, 0.0, 0.0, 1.0])  # scipy는 [x, y, z, w] 순서
    return np.zeros(3)


def _rotation_size(kind: str) -> int:
    return 4 if kind == "quaternion" else 3


# ----------------------------------------------------------------------
# pixel 관측 컨테이너 (rp1 / rp2 전용)
# ----------------------------------------------------------------------

@dataclass
class PixelObservations:
    """카메라 1대의 corner 관측. 포즈 순서는 A_list / B_list와 같아야 한다."""

    board_points: np.ndarray            # (m, 3) board frame corner 좌표
    K: np.ndarray                       # (3, 3)
    distortion: np.ndarray              # (>=5,)
    image_points: list[np.ndarray]      # 포즈별 (k_i, 2) 관측 픽셀
    point_indices: list[np.ndarray]     # 포즈별 (k_i,) board_points 인덱스

    def flat(self) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """(pose_index, point_index, image_point) 를 1차원으로 펴서 돌려준다."""
        poses, indices, pixels = [], [], []
        for pose_index, (index, pixel) in enumerate(
            zip(self.point_indices, self.image_points)
        ):
            poses.append(np.full(len(index), pose_index, dtype=int))
            indices.append(np.asarray(index, dtype=int))
            pixels.append(np.asarray(pixel, dtype=float))
        if not poses:
            return (np.empty(0, int), np.empty(0, int), np.empty((0, 2), float))
        return (
            np.concatenate(poses),
            np.concatenate(indices),
            np.concatenate(pixels),
        )


def _project_pixels(points_camera: np.ndarray, K: np.ndarray,
                    distortion: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """sota_simulation.project_points와 같은 5-parameter 모델.

    다만 최적화 중에는 예측점이 화면 밖으로 나가도 residual이 이어져야 하므로
    image bound gate는 적용하지 않고, depth 양수 여부만 함께 돌려준다.
    """
    points = np.asarray(points_camera, dtype=float)
    z = points[:, 2]
    positive = z > 1e-8
    safe_z = np.where(positive, z, 1.0)
    x = points[:, 0] / safe_z
    y = points[:, 1] / safe_z

    d = np.zeros(5, dtype=float)
    source = np.asarray(distortion, dtype=float).reshape(-1)
    d[: min(5, len(source))] = source[:5]
    k1, k2, p1, p2, k3 = d
    r2 = x * x + y * y
    radial = 1.0 + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2
    x_distorted = x * radial + 2.0 * p1 * x * y + p2 * (r2 + 2.0 * x * x)
    y_distorted = y * radial + p1 * (r2 + 2.0 * y * y) + 2.0 * p2 * x * y
    u = K[0, 0] * x_distorted + K[0, 1] * y_distorted + K[0, 2]
    v = K[1, 1] * y_distorted + K[1, 2]
    return np.column_stack([u, v]), positive


# ----------------------------------------------------------------------
# 설정
# ----------------------------------------------------------------------

@dataclass(frozen=True)
class TabbConfig:
    cost: str = "c2"
    separable: bool = False
    rotation: str = "axis_angle"
    optimizer: str = _DEFAULT_OPTIMIZER
    max_nfev: int | None = None

    def __post_init__(self):
        if self.cost not in COST_FUNCTIONS:
            raise ValueError(f"cost must be one of {COST_FUNCTIONS}")
        if self.rotation not in ROTATION_PARAMETERIZATIONS:
            raise ValueError(f"rotation must be one of {ROTATION_PARAMETERIZATIONS}")
        if self.separable and self.cost in ("rp1", "rp2"):
            # 논문에 reprojection cost의 separable 버전은 없다.
            raise ValueError("rp1/rp2 have no separable formulation in the paper")

    @property
    def label(self) -> str:
        if self.cost in ("rp1", "rp2"):
            return f"{self.cost}_{self.rotation}"
        form = "sep" if self.separable else "sim"
        return f"{self.cost}_{form}_{self.rotation}"


@dataclass
class TabbSolution:
    """A_i X = Z B_i 를 만족하는 X, Z (그리고 multi-eye면 Z 여러 개)."""

    X: np.ndarray
    Z: list[np.ndarray]
    diagnostics: dict = field(default_factory=dict)

    @property
    def Z0(self) -> np.ndarray:
        return self.Z[0]


def _run_least_squares(residual, x0, config: TabbConfig):
    kwargs = dict(xtol=1e-14, ftol=1e-14, gtol=1e-14)
    if config.max_nfev is not None:
        kwargs["max_nfev"] = int(config.max_nfev)
    if config.optimizer == "lm":
        return least_squares(residual, x0, method="lm", **kwargs)
    return least_squares(residual, x0, method="trf", x_scale="jac", **kwargs)


# ----------------------------------------------------------------------
# 파라미터 pack / unpack
#   simultaneous : [p_X, t_X] + per camera [p_Zd, t_Zd]
#   separable    : rotation 단계는 [p_X] + per camera [p_Zd] 만 사용
# ----------------------------------------------------------------------

def _unpack(vector: np.ndarray, kind: str, camera_count: int,
            with_translation: bool):
    size = _rotation_size(kind)
    block = size + 3 if with_translation else size
    p_x = vector[:size]
    t_x = vector[size:size + 3] if with_translation else np.zeros(3)
    R_x = _rotation_from_parameters(p_x, kind)
    cameras = []
    for index in range(camera_count):
        start = block + index * block
        p_z = vector[start:start + size]
        t_z = vector[start + size:start + size + 3] if with_translation \
            else np.zeros(3)
        cameras.append((_rotation_from_parameters(p_z, kind), t_z))
    return R_x, t_x, cameras


def _initial_vector(kind: str, camera_count: int, with_translation: bool,
                    seed_X=None, seed_Z=None) -> np.ndarray:
    """논문 Sec. 3.1: 기본 초기값은 R = I, t = 0."""
    parts = []
    if seed_X is None:
        p_x, t_x = _identity_parameters(kind), np.zeros(3)
    else:
        p_x = _parameters_from_rotation(seed_X[:3, :3], kind)
        t_x = seed_X[:3, 3]
    parts.append(p_x)
    if with_translation:
        parts.append(t_x)
    for index in range(camera_count):
        if seed_Z is None:
            p_z, t_z = _identity_parameters(kind), np.zeros(3)
        else:
            p_z = _parameters_from_rotation(seed_Z[index][:3, :3], kind)
            t_z = seed_Z[index][:3, 3]
        parts.append(p_z)
        if with_translation:
            parts.append(t_z)
    return np.concatenate([np.atleast_1d(part) for part in parts])


# ----------------------------------------------------------------------
# class 1: c1 / c2
# ----------------------------------------------------------------------

def _pose_residual(vector, blocks, kind, cost, with_translation):
    """c1 / c2 residual. blocks는 카메라별 (RA, tA, RB, tB, sqrt_weight)."""
    R_x, t_x, cameras = _unpack(vector, kind, len(blocks), with_translation)
    output = []
    for (RA, tA, RB, tB, scale, _), (R_z, t_z) in zip(blocks, cameras):
        if cost == "c1":
            # A_i X - Z B_i  (Eq. 5 / Eq. 7)
            rotation = RA @ R_x - R_z @ RB
            translation = (RA @ t_x + tA) - (R_z @ tB.T).T - t_z
        else:
            # A_i - Z B_i X~   (Eq. 9 / Eq. 11); 여기서 R_x, t_x 는 X~ 성분
            rotation = RA - (R_z @ RB) @ R_x
            translation = tA - np.einsum("nij,j->ni", R_z @ RB, t_x) \
                - (R_z @ tB.T).T - t_z
        block = [scale * rotation.reshape(len(rotation), 9)]
        if with_translation:
            block.append(scale * translation)
        output.append(np.concatenate(block, axis=1).reshape(-1))
    return np.concatenate(output) if output else np.empty(0)


def _solve_translation_c1(blocks, R_z_list) -> tuple[np.ndarray, list[np.ndarray]]:
    """Eq. 8: min sum ||R_A,i t_X + t_A,i - R_Z t_B,i - t_Z||^2.

    미지수 (t_X, t_Z0..t_Zq-1)에 대해 선형이므로 정규방정식으로 정확히 푼다.
    """
    camera_count = len(blocks)
    columns = 3 + 3 * camera_count
    rows_A, rows_b = [], []
    for index, ((RA, tA, RB, tB, scale, _), R_z) in enumerate(
        zip(blocks, R_z_list)
    ):
        n = len(RA)
        design = np.zeros((n, 3, columns))
        design[:, :, :3] = RA
        design[:, :, 3 + 3 * index:6 + 3 * index] = -np.eye(3)
        target = (R_z @ tB.T).T - tA
        rows_A.append(scale * design.reshape(-1, columns))
        rows_b.append(scale * target.reshape(-1))
    solution, *_ = np.linalg.lstsq(
        np.concatenate(rows_A), np.concatenate(rows_b), rcond=None
    )
    t_x = solution[:3]
    t_z = [solution[3 + 3 * i:6 + 3 * i] for i in range(camera_count)]
    return t_x, t_z


def _solve_translation_c2(blocks, R_z_list) -> tuple[np.ndarray, list[np.ndarray]]:
    """Eq. 12: min sum ||t_A,i - R_Z R_B,i t_X~ - R_Z t_B,i - t_Z||^2."""
    camera_count = len(blocks)
    columns = 3 + 3 * camera_count
    rows_A, rows_b = [], []
    for index, ((RA, tA, RB, tB, scale, _), R_z) in enumerate(
        zip(blocks, R_z_list)
    ):
        n = len(RA)
        design = np.zeros((n, 3, columns))
        design[:, :, :3] = R_z @ RB
        design[:, :, 3 + 3 * index:6 + 3 * index] = np.eye(3)
        target = tA - (R_z @ tB.T).T
        rows_A.append(scale * design.reshape(-1, columns))
        rows_b.append(scale * target.reshape(-1))
    solution, *_ = np.linalg.lstsq(
        np.concatenate(rows_A), np.concatenate(rows_b), rcond=None
    )
    t_x = solution[:3]
    t_z = [solution[3 + 3 * i:6 + 3 * i] for i in range(camera_count)]
    return t_x, t_z


# ----------------------------------------------------------------------
# class 2: rp1 / rp2
# ----------------------------------------------------------------------

def _reprojection_residual(vector, blocks, pixel_blocks, kind, refine_intrinsics):
    """Eq. 16 / Eq. 17: x_ij - f(k, [Z B_i X~]_{3x4} X_j).

    벡터 앞부분은 X~, Z_d 파라미터이고, rp2(refine_intrinsics=True)일 때만
    카메라마다 [fx, fy, cx, cy, k1, k2, p1, p2] 8개 intrinsic이 뒤에 붙는다.
    """
    camera_count = len(blocks)
    size = _rotation_size(kind)
    block = size + 3
    pose_length = block * (camera_count + 1)
    R_x, t_x, cameras = _unpack(vector[:pose_length], kind, camera_count, True)
    X_tilde = _to_h(R_x, t_x)

    output = []
    for index, ((_, _, _, _, scale, B_h), (R_z, t_z), pixels) in enumerate(
        zip(blocks, cameras, pixel_blocks)
    ):
        K = pixels.K.copy()
        distortion = np.asarray(pixels.distortion, dtype=float).reshape(-1).copy()
        if refine_intrinsics:
            start = pose_length + 8 * index
            fx, fy, cx, cy, k1, k2, p1, p2 = vector[start:start + 8]
            K[0, 0], K[1, 1], K[0, 2], K[1, 2] = fx, fy, cx, cy
            distortion = np.pad(distortion, (0, max(0, 5 - distortion.size)))[:5]
            distortion[:4] = [k1, k2, p1, p2]

        Z = _to_h(R_z, t_z)
        pose_indices, point_indices, observed = pixels.flat()
        if len(pose_indices) == 0:
            continue
        # T_camera_board(i) = Z @ B_i @ X~
        chains = Z @ B_h @ X_tilde
        selected = chains[pose_indices]
        points = pixels.board_points[point_indices]
        camera_points = np.einsum(
            "nij,nj->ni", selected[:, :3, :3], points
        ) + selected[:, :3, 3]
        predicted, positive = _project_pixels(camera_points, K, distortion)
        residual = predicted - observed
        # 카메라 뒤로 넘어간 점은 큰 상수 penalty (methods.py와 같은 처리)
        residual[~positive] = 1e3
        output.append(scale * residual.reshape(-1))
    return np.concatenate(output) if output else np.empty(0)


# ----------------------------------------------------------------------
# 메인 solver
# ----------------------------------------------------------------------

def _build_blocks(A_lists, B_lists, weights):
    blocks = []
    for A_list, B_list, weight in zip(A_lists, B_lists, weights):
        if len(A_list) != len(B_list):
            raise ValueError("A_list and B_list must have the same length")
        RA, tA = _stack_Rt(A_list)
        RB, tB = _stack_Rt(B_list)
        B_h = np.asarray(B_list, dtype=float)
        blocks.append((RA, tA, RB, tB, float(np.sqrt(weight)), B_h))
    return blocks


def _equal_influence_weights(A_lists) -> list[float]:
    """Eq. 23: w_d = min_e |S_e| / |S_d| 로 카메라별 영향력을 같게 만든다."""
    sizes = [len(item) for item in A_lists]
    smallest = min(sizes)
    return [smallest / size for size in sizes]


def solve_tabb_multi(
    A_lists: Sequence[Sequence[np.ndarray]],
    B_lists: Sequence[Sequence[np.ndarray]],
    config: TabbConfig = TabbConfig(),
    pixel_blocks: Sequence[PixelObservations] | None = None,
) -> TabbSolution:
    """A_{i,d} X = Z_d B_i 를 푼다 (카메라 1대면 Eq. 2와 동일).

    반환하는 X, Z는 항상 논문 정의 그대로다 (X: base->world, Z: hand->camera).
    c2/rp1/rp2는 내부적으로 X~ = X^{-1}을 추정하지만 반환 직전에 역변환한다.
    """
    camera_count = len(A_lists)
    if camera_count == 0:
        raise ValueError("need at least one camera")
    for A_list in A_lists:
        if len(A_list) < 3:
            raise ValueError("Tabb needs at least three robot positions per camera")

    weights = _equal_influence_weights(A_lists)
    blocks = _build_blocks(A_lists, B_lists, weights)
    kind = config.rotation
    diagnostics: dict = {"config": config.label, "weights": weights}

    if config.cost in ("c1", "c2"):
        if config.separable:
            # 1단계: rotation만 (Eq. 7 / Eq. 11)
            x0 = _initial_vector(kind, camera_count, with_translation=False)
            solution = _run_least_squares(
                lambda v: _pose_residual(v, blocks, kind, config.cost, False),
                x0, config,
            )
            R_x, _, cameras = _unpack(solution.x, kind, camera_count, False)
            R_z_list = [item[0] for item in cameras]
            # 2단계: translation은 선형이므로 정확해 (Eq. 8 / Eq. 12)
            solver = _solve_translation_c1 if config.cost == "c1" \
                else _solve_translation_c2
            t_x, t_z_list = solver(blocks, R_z_list)
            diagnostics.update(
                rotation_cost=float(solution.cost),
                nfev=int(solution.nfev),
                message=str(solution.message),
            )
        else:
            x0 = _initial_vector(kind, camera_count, with_translation=True)
            solution = _run_least_squares(
                lambda v: _pose_residual(v, blocks, kind, config.cost, True),
                x0, config,
            )
            R_x, t_x, cameras = _unpack(solution.x, kind, camera_count, True)
            R_z_list = [item[0] for item in cameras]
            t_z_list = [item[1] for item in cameras]
            diagnostics.update(
                cost=float(solution.cost),
                nfev=int(solution.nfev),
                message=str(solution.message),
            )
        estimated = _to_h(R_x, t_x)
        Z_list = [_to_h(R, t) for R, t in zip(R_z_list, t_z_list)]
        # c1은 X를, c2는 X~ = X^{-1}을 추정한다.
        X = estimated if config.cost == "c1" else _inv(estimated)
        return TabbSolution(X=X, Z=Z_list, diagnostics=diagnostics)

    # ---- class 2: rp1 / rp2 ----
    if pixel_blocks is None or len(pixel_blocks) != camera_count:
        raise ValueError("rp1/rp2 need one PixelObservations per camera")

    # 논문 Sec. 3.1: rp1의 초기값은 c2 simultaneous 해.
    seed = solve_tabb_multi(
        A_lists, B_lists,
        TabbConfig(cost="c2", separable=False, rotation=kind,
                   optimizer=config.optimizer),
    )
    seed_X_tilde = _inv(seed.X)
    x0 = _initial_vector(kind, camera_count, True, seed_X_tilde, seed.Z)

    if config.cost == "rp2":
        # 논문 Sec. 3.1: rp2의 초기값은 rp1 해.
        rp1 = solve_tabb_multi(
            A_lists, B_lists,
            TabbConfig(cost="rp1", rotation=kind, optimizer=config.optimizer),
            pixel_blocks,
        )
        x0 = _initial_vector(kind, camera_count, True, _inv(rp1.X), rp1.Z)
        intrinsics = []
        for pixels in pixel_blocks:
            distortion = np.asarray(
                pixels.distortion, dtype=float).reshape(-1)
            distortion = np.pad(distortion, (0, max(0, 5 - distortion.size)))[:5]
            intrinsics.append(np.array([
                pixels.K[0, 0], pixels.K[1, 1], pixels.K[0, 2], pixels.K[1, 2],
                distortion[0], distortion[1], distortion[2], distortion[3],
            ]))
        x0 = np.concatenate([x0, *intrinsics])

    refine = config.cost == "rp2"
    solution = _run_least_squares(
        lambda v: _reprojection_residual(
            v, blocks, pixel_blocks, kind, refine),
        x0, config,
    )
    size = _rotation_size(kind)
    pose_length = (size + 3) * (camera_count + 1)
    R_x, t_x, cameras = _unpack(solution.x[:pose_length], kind, camera_count, True)
    diagnostics.update(
        cost=float(solution.cost),
        nfev=int(solution.nfev),
        message=str(solution.message),
        seed=seed.diagnostics,
    )
    if refine:
        diagnostics["intrinsics"] = [
            solution.x[pose_length + 8 * i:pose_length + 8 * (i + 1)].tolist()
            for i in range(camera_count)
        ]
    return TabbSolution(
        X=_inv(_to_h(R_x, t_x)),
        Z=[_to_h(R, t) for R, t in cameras],
        diagnostics=diagnostics,
    )


def solve_tabb(
    A_list: Sequence[np.ndarray],
    B_list: Sequence[np.ndarray],
    config: TabbConfig = TabbConfig(),
    pixels: PixelObservations | None = None,
) -> TabbSolution:
    """카메라 1대 버전 (Eq. 2)."""
    return solve_tabb_multi(
        [A_list], [B_list], config,
        None if pixels is None else [pixels],
    )


# ----------------------------------------------------------------------
# 이 저장소의 transform 규칙으로의 매핑
# ----------------------------------------------------------------------

@dataclass
class TabbEyeInHandResult:
    T_gripper_wrist: np.ndarray   # 주 출력 (다른 방법과 비교 가능)
    T_base_board: np.ndarray      # 추가 출력: 정지 board의 base 상 위치
    diagnostics: dict = field(default_factory=dict)


@dataclass
class TabbEyeToHandResult:
    T_base_fixed: dict[str, np.ndarray]   # 주 출력: 카메라별 extrinsic
    T_gripper_board: np.ndarray           # 추가 출력: gripper 상 board 오프셋
    diagnostics: dict = field(default_factory=dict)


def solve_tabb_eye_in_hand(
    T_base_gripper_list: Sequence[np.ndarray],
    T_wrist_board_list: Sequence[np.ndarray],
    config: TabbConfig = TabbConfig(),
    pixels: PixelObservations | None = None,
) -> TabbEyeInHandResult:
    """Eye-in-hand: board는 base에 고정, camera는 gripper에 고정.

        T_base_board = T_base_gripper(k) @ T_gripper_wrist @ T_wrist_board(k)

    논문 A_i X = Z B_i 로 재배열하면 (논문 정의 그대로):
        A_i = T_wrist_board(k)                  <- world(board) -> camera
        B_i = inverse(T_base_gripper(k))        <- base -> end-effector
        X   = T_board_base                       (robot-world), X~ = T_base_board
        Z   = T_wrist_gripper                    (hand-eye), 주 출력은 Z^{-1}

    ※ 논문 정의가 "base -> end-effector"이므로 eye-in-hand에서 robot pose를
      **반전해서** 넣는다.  Tsai 계열(AX=XB)은 반대로 eye-to-hand에서만
      반전하고, Shah(AX=YB)는 두 경우 모두 반전하지 않는다.  세 규칙이 모두
      다르므로 self_test()로 항상 재확인할 것.
    """
    A_list = list(T_wrist_board_list)
    B_list = [_inv(T) for T in T_base_gripper_list]
    solution = solve_tabb(A_list, B_list, config, pixels)
    return TabbEyeInHandResult(
        T_gripper_wrist=_inv(solution.Z0),
        T_base_board=_inv(solution.X),
        diagnostics=solution.diagnostics,
    )


def solve_tabb_eye_to_hand(
    camera_names: Sequence[str],
    T_base_gripper_lists: Sequence[Sequence[np.ndarray]],
    T_fixed_board_lists: Sequence[Sequence[np.ndarray]],
    config: TabbConfig = TabbConfig(),
    pixel_blocks: Sequence[PixelObservations] | None = None,
    joint: bool = False,
) -> TabbEyeToHandResult:
    """Eye-to-hand: board는 gripper에 고정, camera는 base에 고정.

        T_base_gripper(k) @ T_gripper_board = T_base_fixed @ T_fixed_board(k)

    논문 A_i X = Z B_i 로 재배열하면:
        A_i = T_fixed_board(k)                  <- world(board) -> camera
        B_i = T_base_gripper(k)                 <- (반전하지 않음, 아래 주석)
        X   = T_board_gripper                    X~ = T_gripper_board (추가 출력)
        Z   = T_fixed_base                       주 출력은 Z^{-1} = T_base_fixed

    논문은 "camera가 end-effector에 달리고 board가 정지"한 배치를 가정한다.
    이 저장소의 fixed camera는 반대 배치(board가 gripper에, camera가 정지)라서
    base와 end-effector의 역할이 서로 바뀐다.  그 결과 eye-to-hand에서는
    robot pose를 반전하지 않는다.

    joint=True면 논문 Sec. 2.3(Eq. 19-23)의 multi-eye 형태로, 여러 카메라가
    X(= board가 gripper에 붙은 고정 오프셋) 하나를 **공유**하도록 동시에 푼다.
    joint=False면 카메라마다 독립적으로 풀고 X는 평균으로 합친다
    (Shah/Tsai 계열이 하는 것과 같은 방식).
    """
    names = list(camera_names)
    if joint:
        solution = solve_tabb_multi(
            list(T_fixed_board_lists), list(T_base_gripper_lists),
            config, pixel_blocks,
        )
        return TabbEyeToHandResult(
            T_base_fixed={name: _inv(Z) for name, Z in zip(names, solution.Z)},
            T_gripper_board=_inv(solution.X),
            diagnostics=solution.diagnostics,
        )

    cameras: dict[str, np.ndarray] = {}
    boards: list[np.ndarray] = []
    per_camera: dict[str, dict] = {}
    for index, name in enumerate(names):
        pixels = None if pixel_blocks is None else pixel_blocks[index]
        solution = solve_tabb(
            list(T_fixed_board_lists[index]),
            list(T_base_gripper_lists[index]),
            config, pixels,
        )
        cameras[name] = _inv(solution.Z0)
        boards.append(_inv(solution.X))
        per_camera[name] = solution.diagnostics
    return TabbEyeToHandResult(
        T_base_fixed=cameras,
        T_gripper_board=_average_transforms(boards),
        diagnostics={"per_camera": per_camera},
    )


def _average_transforms(transforms: Iterable[np.ndarray]) -> np.ndarray:
    items = list(transforms)
    output = np.eye(4)
    output[:3, :3] = Rotation.from_matrix(
        [T[:3, :3] for T in items]).mean().as_matrix()
    output[:3, 3] = np.mean([T[:3, 3] for T in items], axis=0)
    return output


# ----------------------------------------------------------------------
# self test: 이 파일만 단독 실행해서 convention이 맞는지 항상 재검증 가능
# ----------------------------------------------------------------------

def _random_rotation(rng) -> np.ndarray:
    q = rng.normal(size=4)
    return Rotation.from_quat(q / np.linalg.norm(q)).as_matrix()


def _rotation_error_deg(R1, R2) -> float:
    cosine = np.clip((np.trace(R1.T @ R2) - 1.0) / 2.0, -1.0, 1.0)
    return float(np.degrees(np.arccos(cosine)))


def _pose_error(estimate, truth) -> tuple[float, float]:
    delta = _inv(truth) @ estimate
    return (
        float(1000.0 * np.linalg.norm(delta[:3, 3])),
        float(np.degrees(np.linalg.norm(
            Rotation.from_matrix(delta[:3, :3]).as_rotvec()))),
    )


def self_test(n_poses: int = 12, seed: int = 42, verbose: bool = True) -> bool:
    """noise=0 합성 데이터로 모든 cost/parameterization 조합을 검증한다."""
    rng = np.random.default_rng(seed)
    ok = True

    # ---- eye-in-hand ----
    T_gripper_wrist = _to_h(_random_rotation(rng), rng.uniform(-0.3, 0.3, 3))
    T_base_board = _to_h(_random_rotation(rng), rng.uniform(-0.5, 0.5, 3))
    robot, visual = [], []
    for _ in range(n_poses):
        T_base_gripper = _to_h(_random_rotation(rng), rng.uniform(-1, 1, 3))
        robot.append(T_base_gripper)
        visual.append(_inv(T_gripper_wrist) @ _inv(T_base_gripper) @ T_base_board)

    for cost in ("c1", "c2"):
        for separable in (False, True):
            for rotation in ROTATION_PARAMETERIZATIONS:
                config = TabbConfig(cost=cost, separable=separable,
                                    rotation=rotation)
                result = solve_tabb_eye_in_hand(robot, visual, config)
                t_err, r_err = _pose_error(result.T_gripper_wrist, T_gripper_wrist)
                b_err, _ = _pose_error(result.T_base_board, T_base_board)
                passed = t_err < 1e-3 and r_err < 1e-4 and b_err < 1e-3
                ok &= passed
                if verbose:
                    print(f"[eye-in-hand {config.label:>22}] "
                          f"T_gripper_wrist {t_err:.3e} mm / {r_err:.3e} deg  "
                          f"T_base_board {b_err:.3e} mm  "
                          f"{'PASS' if passed else 'FAIL'}")

    # ---- eye-to-hand, 카메라 3대 multi-eye ----
    T_gripper_board = _to_h(_random_rotation(rng), rng.uniform(-0.2, 0.2, 3))
    names = ["cam0", "cam1", "cam3"]
    T_base_fixed = {name: _to_h(_random_rotation(rng), rng.uniform(-0.8, 0.8, 3))
                    for name in names}
    robot_lists, visual_lists = [], []
    robot2 = [_to_h(_random_rotation(rng), rng.uniform(-1, 1, 3))
              for _ in range(n_poses)]
    for name in names:
        robot_lists.append(robot2)
        visual_lists.append([
            _inv(T_base_fixed[name]) @ T @ T_gripper_board for T in robot2
        ])

    for joint in (False, True):
        for cost in ("c1", "c2"):
            for separable in (False, True):
                config = TabbConfig(cost=cost, separable=separable,
                                    rotation="axis_angle")
                result = solve_tabb_eye_to_hand(
                    names, robot_lists, visual_lists, config, joint=joint)
                errors = [
                    _pose_error(result.T_base_fixed[name], T_base_fixed[name])
                    for name in names
                ]
                t_err = max(error[0] for error in errors)
                r_err = max(error[1] for error in errors)
                b_err, _ = _pose_error(result.T_gripper_board, T_gripper_board)
                passed = t_err < 1e-3 and r_err < 1e-4 and b_err < 1e-3
                ok &= passed
                if verbose:
                    tag = "joint" if joint else "indep"
                    print(f"[eye-to-hand {tag} {config.label:>16}] "
                          f"T_base_fixed {t_err:.3e} mm / {r_err:.3e} deg  "
                          f"T_gripper_board {b_err:.3e} mm  "
                          f"{'PASS' if passed else 'FAIL'}")

    # ---- class 2 (rp1 / rp2): pixel 관측이 필요하므로 합성 카메라를 만든다 ----
    board_points = np.array([
        [x * 0.025 - 0.1, y * 0.025 - 0.06, 0.0]
        for y in range(6) for x in range(10)
    ])
    K = np.array([[900.0, 0.0, 640.0], [0.0, 900.0, 360.0], [0.0, 0.0, 1.0]])
    distortion = np.array([-0.08, 0.02, 0.0005, -0.0003, 0.0])
    pixel_blocks, robot3, visual3 = [], [], []
    T_gripper_board3 = _to_h(
        Rotation.from_euler("xyz", [3.0, -2.0, 5.0], degrees=True).as_matrix(),
        [0.01, -0.02, 0.10])
    T_base_fixed3 = _to_h(
        Rotation.from_euler("xyz", [-110.0, 0.0, 8.0], degrees=True).as_matrix(),
        [0.05, -0.62, 0.40])
    for index in range(n_poses):
        phase = 2.0 * np.pi * index / n_poses
        T_base_gripper = _to_h(
            Rotation.from_euler(
                "xyz",
                [14.0 * np.sin(phase), 16.0 * np.cos(2.0 * phase),
                 22.0 * np.sin(3.0 * phase)],
                degrees=True).as_matrix(),
            [0.12 * np.sin(phase), 0.07 * np.cos(phase), 0.27])
        robot3.append(T_base_gripper)
        visual3.append(_inv(T_base_fixed3) @ T_base_gripper @ T_gripper_board3)
    image_points, point_indices = [], []
    for T_fixed_board in visual3:
        camera_points = board_points @ T_fixed_board[:3, :3].T + T_fixed_board[:3, 3]
        pixels, positive = _project_pixels(camera_points, K, distortion)
        index = np.flatnonzero(positive)
        point_indices.append(index)
        image_points.append(pixels[index])
    pixel_blocks.append(PixelObservations(
        board_points=board_points, K=K, distortion=distortion,
        image_points=image_points, point_indices=point_indices))

    for cost in ("rp1", "rp2"):
        config = TabbConfig(cost=cost, rotation="axis_angle")
        result = solve_tabb_eye_to_hand(
            ["cam0"], [robot3], [visual3], config, pixel_blocks)
        t_err, r_err = _pose_error(result.T_base_fixed["cam0"], T_base_fixed3)
        b_err, _ = _pose_error(result.T_gripper_board, T_gripper_board3)
        passed = t_err < 1e-2 and r_err < 1e-3 and b_err < 1e-2
        ok &= passed
        if verbose:
            print(f"[eye-to-hand pixel {config.label:>16}] "
                  f"T_base_fixed {t_err:.3e} mm / {r_err:.3e} deg  "
                  f"T_gripper_board {b_err:.3e} mm  "
                  f"{'PASS' if passed else 'FAIL'}")

    if verbose:
        print("PASS" if ok else "FAIL")
    return ok


if __name__ == "__main__":
    self_test()
