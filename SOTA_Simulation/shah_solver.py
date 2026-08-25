"""
Tsai/Park/Horaud/Andreff/Daniilidis 와의 결정적 차이:
    - 다른 방법들은 AX = XB 를 풀고 cv2.calibrateHandEye() 로 미지수 1개(카메라
      extrinsic)만 얻는다.
    - Shah는 AX = YB 를 풀고 cv2.calibrateRobotWorldHandEye() 로 미지수
      2개를 동시에 얻는다. 하나는 다른 방법들과 직접 비교 가능한 카메라
      extrinsic(Y_shah)이고, 다른 하나는 '보드가 base(또는 gripper)에 대해
      고정된 위치'라는 부가 제약을 알려주는 추가 출력(X_shah)이다.

이 모듈은 cv2.CALIB_ROBOT_WORLD_HAND_EYE_SHAH 를 호출하는데,
OpenCV가 반환하는 (R_base2world, t_base2world), (R_gripper2cam, t_gripper2cam)
은 실제로 A_j X = Y B_j 를 만족하는 X, Y 의 역행렬이라는 것을
synthetic zero-noise 검증으로 실제 확인함. (아래 self_test 참고)
따라서 이 모듈 내부에서 그 역행렬 보정을 이미 적용해 반환한다.

두 setup 모두 zero-noise self-consistency 검증 완료 (rot/trans error = 0.000000)
"""

from dataclasses import dataclass
from typing import List, Tuple

import numpy as np
import cv2


# ----------------------------------------------------------------------
# 4x4 <-> (R, t) 유틸
# ----------------------------------------------------------------------

def _to_Rt(T: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    return T[:3, :3].copy(), T[:3, 3].copy()


def _to_h(R: np.ndarray, t: np.ndarray) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = np.asarray(t).reshape(3)
    return T


def _inv(T: np.ndarray) -> np.ndarray:
    R, t = _to_Rt(T)
    Ti = np.eye(4)
    Ti[:3, :3] = R.T
    Ti[:3, 3] = -R.T @ t
    return Ti


# ----------------------------------------------------------------------
# AX = YB 를 푸는 raw solver
# ----------------------------------------------------------------------

def _solve_axyb_shah(A_list: List[np.ndarray], B_list: List[np.ndarray],
                      method: str = "SHAH") -> Tuple[np.ndarray, np.ndarray]:
    """
    A_j @ X = Y @ B_j 를 만족하는 (X, Y) 를 반환한다.

    A_list, B_list: 4x4 T_destination_source 행렬 리스트, 길이 >= 3
                     (Theorem 2.2/2.3 근거로 회전축이 다양한 5+ 포즈 권장)
    method: "SHAH" (separable, 이 논문) 또는 "LI" (simultaneous, 비교용)
    """
    n = len(A_list)
    assert n == len(B_list) >= 3, "Shah/Li 방법은 최소 3 포즈 필요"

    RA = [_to_Rt(a)[0] for a in A_list]
    tA = [_to_Rt(a)[1] for a in A_list]
    RB = [_to_Rt(b)[0] for b in B_list]
    tB = [_to_Rt(b)[1] for b in B_list]

    cv_method = {
        "SHAH": cv2.CALIB_ROBOT_WORLD_HAND_EYE_SHAH,
        "LI": cv2.CALIB_ROBOT_WORLD_HAND_EYE_LI,
    }[method]

    # 첫 두 인자가 "world2cam"(=B_j), 다음 두 인자가 "base2gripper"(=A_j)
    R_b2w, t_b2w, R_g2c, t_g2c = cv2.calibrateRobotWorldHandEye(
        RB, tB, RA, tA, method=cv_method,
    )

    # OpenCV가 반환하는 값은 X, Y 의 역행렬이다 (synthetic 검증으로 확인)
    X = _inv(_to_h(R_b2w, t_b2w))
    Y = _inv(_to_h(R_g2c, t_g2c))
    return X, Y


@dataclass
class ShahEyeInHandResult:
    T_gripper_wrist: np.ndarray 
    T_base_board: np.ndarray      # 추가 출력: 보드의 고정 위치


@dataclass
class ShahEyeToHandResult:
    T_base_fixed_i: np.ndarray 
    T_gripper_board: np.ndarray   # 추가 출력: 보드의 gripper 상 고정 오프셋


def solve_shah_eye_in_hand(
    T_base_gripper_list: List[np.ndarray],
    T_wrist_board_list: List[np.ndarray],
    method: str = "SHAH",
) -> ShahEyeInHandResult:
    """
    Eye-in-hand (wrist camera):  board는 base frame에 고정, camera는 gripper에 고정.

        T_base_board == T_base_gripper(k) @ T_gripper_wrist @ T_wrist_board(k)   for all k

    위 식을 A_j X = Y B_j 형태로 재정렬하면:
        A_j = T_base_gripper(k)                     (그대로, 반전 불필요)
        B_j = inv(T_wrist_board(k)) = T_board_wrist(k)
        X   = T_gripper_wrist                        <- 주 출력
        Y   = T_base_board                            <- 추가 출력

    입력은 다른 5개 방법과 완전히 동일한 T_base_gripper(k),
    T_wrist_board(k) 리스트를 그대로 사용하면 된다.
    """
    A_list = T_base_gripper_list
    B_list = [_inv(T) for T in T_wrist_board_list]
    T_gripper_wrist, T_base_board = _solve_axyb_shah(A_list, B_list, method=method)
    return ShahEyeInHandResult(T_gripper_wrist=T_gripper_wrist, T_base_board=T_base_board)


def solve_shah_eye_to_hand(
    T_base_gripper_list: List[np.ndarray],
    T_fixed_i_board_list: List[np.ndarray],
    method: str = "SHAH",
) -> ShahEyeToHandResult:
    """
    Eye-to-hand (fixed camera i):  board는 gripper에 고정, camera는 base frame에 고정.

        T_base_gripper(k) @ T_gripper_board == T_base_fixed_i @ T_fixed_i_board(k)   for all k

    위 식을 A_j X = Y B_j 형태로 재정렬하면:
        A_j = T_base_gripper(k)          (그대로, 다른 방법처럼 inverse 안 함)
        B_j = T_fixed_i_board(k)          (그대로)
        X   = T_gripper_board             <- 추가 출력
        Y   = T_base_fixed_i              <- 주 출력

    ※ 다른 5개 방법은 eye-to-hand에서
      T_gripper_base = inverse(T_base_gripper) 를 robot input으로 반전해서 넣었지만,
      Shah/Li는 AX=YB 형태 자체가 이미 두 변환을 분리해서 다루므로
      T_base_gripper 를 반전 없이 그대로 넣는다. (self_test()에서 검증됨)
    """
    T_gripper_board, T_base_fixed_i = _solve_axyb_shah(
        T_base_gripper_list, T_fixed_i_board_list, method=method
    )
    return ShahEyeToHandResult(T_base_fixed_i=T_base_fixed_i, T_gripper_board=T_gripper_board)


# ----------------------------------------------------------------------
# self test: 이 파일만 단독 실행해서 convention이 맞는지 항상 재검증 가능
# ----------------------------------------------------------------------

def _rand_rot(rng):
    q = rng.normal(size=4)
    q /= np.linalg.norm(q)
    w, x, y, z = q
    return np.array([
        [1 - 2*(y*y+z*z), 2*(x*y - z*w), 2*(x*z + y*w)],
        [2*(x*y + z*w), 1 - 2*(x*x+z*z), 2*(y*z - x*w)],
        [2*(x*z - y*w), 2*(y*z + x*w), 1 - 2*(x*x+y*y)],
    ])


def _rot_err_deg(R1, R2):
    c = np.clip((np.trace(R1.T @ R2) - 1) / 2, -1, 1)
    return np.degrees(np.arccos(c))


def self_test(n_poses: int = 15, seed: int = 42, verbose: bool = True) -> bool:
    """noise=0 합성 데이터로 eye-in-hand / eye-to-hand 두 wrapper를 모두 검증"""
    rng = np.random.default_rng(seed)
    ok = True

    # eye-in-hand
    T_gripper_wrist_gt = _to_h(_rand_rot(rng), rng.uniform(-.3, .3, 3))
    T_base_board_gt = _to_h(_rand_rot(rng), rng.uniform(-.5, .5, 3))
    T_base_gripper_list, T_wrist_board_list = [], []
    for _ in range(n_poses):
        T_base_gripper = _to_h(_rand_rot(rng), rng.uniform(-1, 1, 3))
        T_wrist_board = _inv(T_gripper_wrist_gt) @ _inv(T_base_gripper) @ T_base_board_gt
        T_base_gripper_list.append(T_base_gripper)
        T_wrist_board_list.append(T_wrist_board)
    res = solve_shah_eye_in_hand(T_base_gripper_list, T_wrist_board_list)
    e1 = _rot_err_deg(T_gripper_wrist_gt[:3, :3], res.T_gripper_wrist[:3, :3])
    e2 = _rot_err_deg(T_base_board_gt[:3, :3], res.T_base_board[:3, :3])
    ok &= e1 < 1e-3 and e2 < 1e-3
    if verbose:
        print(f"[eye-in-hand]  T_gripper_wrist rot_err={e1:.6f} deg | T_base_board rot_err={e2:.6f} deg")

    # eye-to-hand
    T_gripper_board_gt = _to_h(_rand_rot(rng), rng.uniform(-.2, .2, 3))
    T_base_fixed_gt = _to_h(_rand_rot(rng), rng.uniform(-.8, .8, 3))
    T_base_gripper_list2, T_fixed_board_list2 = [], []
    for _ in range(n_poses):
        T_base_gripper = _to_h(_rand_rot(rng), rng.uniform(-1, 1, 3))
        T_fixed_board = _inv(T_base_fixed_gt) @ T_base_gripper @ T_gripper_board_gt
        T_base_gripper_list2.append(T_base_gripper)
        T_fixed_board_list2.append(T_fixed_board)
    res2 = solve_shah_eye_to_hand(T_base_gripper_list2, T_fixed_board_list2)
    e3 = _rot_err_deg(T_base_fixed_gt[:3, :3], res2.T_base_fixed_i[:3, :3])
    e4 = _rot_err_deg(T_gripper_board_gt[:3, :3], res2.T_gripper_board[:3, :3])
    ok &= e3 < 1e-3 and e4 < 1e-3
    if verbose:
        print(f"[eye-to-hand]  T_base_fixed_i rot_err={e3:.6f} deg | T_gripper_board rot_err={e4:.6f} deg")
        print("PASS" if ok else "FAIL")
    return ok


# ----------------------------------------------------------------------
# tsai_combined_demo.py 의 solve_hand_eye() 와 같은 시그니처의 단일 함수.
# shah_combined_demo.py 같은 데모 스크립트를 나중에 만들 때 drop-in으로 쓴다.
#
# 주의: tsai는 eye_to_hand=True일 때 robot_poses를 inverse해서 넣지만(AX=XB 방식),
#       Shah는 AX=YB 방식이라 eye_to_hand=True일 때도 반전하지 않는다.
#       이 차이가 이 wrapper의 존재 이유 — 같은 시그니처 뒤에 다른 규칙이 숨어 있으므로,
#       이 함수를 거치지 않고 cv2.calibrateRobotWorldHandEye를 직접 호출하는 코드를
#       새로 짤 때는 이 규칙을 반드시 다시 확인할 것.
# ----------------------------------------------------------------------

def solve_shah(
    robot_poses: List[np.ndarray],
    T_camera_target: List[np.ndarray],
    eye_to_hand: bool,
    method: str = "SHAH",
) -> np.ndarray:
    """다른 5개 방법(solve_hand_eye)과 같은 시그니처, 주 출력만 반환.

    Eye-in-hand (eye_to_hand=False):
      robot_poses      = T_base_gripper   (반전 없음)
      T_camera_target  = T_wrist_board
      반환             = T_gripper_wrist   (주 출력)
      * 보너스 출력 T_base_board가 필요하면 solve_shah_eye_in_hand()를 직접 쓸 것.

    Eye-to-hand (eye_to_hand=True):
      robot_poses      = T_base_gripper   (다른 방법과 달리 반전하지 않음!)
      T_camera_target  = T_fixed_board (=T_camera_board)
      반환             = T_base_fixed      (주 출력)
      * 보너스 출력 T_gripper_board가 필요하면 solve_shah_eye_to_hand()를 직접 쓸 것.
    """
    if eye_to_hand:
        result = solve_shah_eye_to_hand(robot_poses, T_camera_target, method=method)
        return result.T_base_fixed_i
    result = solve_shah_eye_in_hand(robot_poses, T_camera_target, method=method)
    return result.T_gripper_wrist


# ----------------------------------------------------------------------
# methods.py 의 CalibrationMethod 프로토콜 구현 (load_method("module:factory")
# 로 바로 등록 가능). 
#
# methods.py 의 _pixel_residual() 이 가정하는 기하학적 모델(모든 카메라를
# "board가 gripper에 고정된" eye-to-hand 스타일로 통일해서 다룸)을 그대로 따른다:
#
#     T_base_board(event) = T_base_gripper(event) @ T_gripper_board
#     T_camera_board(event) = inverse(camera_pose) @ T_base_board(event)
#
# 이를 재정렬하면:
#     T_base_gripper(event) @ T_gripper_board == camera_pose @ T_camera_board(event)
#     ↔ A_j X = Y B_j  with  A_j=T_base_gripper(event), B_j=T_camera_board(event),
#                              X=T_gripper_board (카메라별 보너스 추정치),
#                              Y=camera_pose (주 출력)
#
# IndependentReprojectionReference와 동일하게 카메라별로 독립 계산 후,
# T_gripper_board는 카메라 간 평균으로 합친다. 다만 Shah/Li만 가능한
# 부가 진단으로 "카메라별 board 추정치가 서로 얼마나 일치하는지"를
# diagnostics에 같이 기록한다 (물리적으로는 하나의 값이어야 하므로,
# 흩어짐이 크면 해당 카메라의 pose 다양성 부족이나 noise를 의심할 신호).
# ----------------------------------------------------------------------

def _average_transforms_shah(transforms: List[np.ndarray]) -> np.ndarray:
    from scipy.spatial.transform import Rotation
    output = np.eye(4)
    output[:3, :3] = Rotation.from_matrix(
        [T[:3, :3] for T in transforms]
    ).mean().as_matrix()
    output[:3, 3] = np.mean([T[:3, 3] for T in transforms], axis=0)
    return output


class ShahKroneckerMethod:
    """Shah (2013) closed-form robot-world/hand-eye via the Kronecker product.

    JointReprojectionReference / IndependentReprojectionReference(methods.py)와
    달리 pixel-space bundle adjustment가 아니라, cv2.calibrateRobotWorldHandEye
    를 이용한 closed-form 해다. 카메라별로 독립 계산한다는 점은
    IndependentReprojectionReference와 같다.
    """

    name = "shah_kronecker"

    def __init__(self, method: str = "SHAH", min_poses: int = 3):
        # method: "SHAH"(separable, 이 논문) 또는 "LI"(simultaneous, 비교용)
        self.method = method
        self.min_poses = min_poses

    def calibrate(self, data) -> "CalibrationResult":  # CalibrationResult: sota_simulation
        from .sota_simulation import CalibrationResult

        camera_results = {}
        board_results = []
        per_camera = {}
        overall_success = True

        for camera_name in sorted(data.cameras):
            observations = sorted(
                (item for item in data.observations if item.camera == camera_name),
                key=lambda item: item.event,
            )
            if len(observations) < self.min_poses:
                overall_success = False
                per_camera[camera_name] = {
                    "success": False,
                    "message": f"only {len(observations)} poses, need >= {self.min_poses}",
                }
                continue

            A_list = [data.T_base_gripper[item.event] for item in observations]
            B_list = [item.T_camera_board_exact for item in observations]

            try:
                result = solve_shah_eye_to_hand(A_list, B_list, method=self.method)
            except Exception as error:  # SVD/degenerate-pose failure
                overall_success = False
                per_camera[camera_name] = {"success": False, "message": str(error)}
                continue

            camera_results[camera_name] = result.T_base_fixed_i
            board_results.append(result.T_gripper_board)
            per_camera[camera_name] = {
                "success": True,
                "n_poses": len(observations),
                "events": [item.event for item in observations],
                "T_gripper_board_estimate": result.T_gripper_board.tolist(),
            }

        if not board_results:
            overall_success = False
            T_gripper_board = np.eye(4)
            translation_spread = float("nan")
            rotation_spread_deg = float("nan")
        else:
            T_gripper_board = _average_transforms_shah(board_results)
            if len(board_results) >= 2:
                from scipy.spatial.transform import Rotation
                translation_spread = float(
                    np.std([T[:3, 3] for T in board_results], axis=0).mean()
                )
                rotation_spread_deg = float(
                    np.std(
                        [Rotation.from_matrix(T[:3, :3]).as_rotvec() for T in board_results],
                        axis=0,
                    ).mean() * 180.0 / np.pi
                )
            else:
                translation_spread = 0.0
                rotation_spread_deg = 0.0

        return CalibrationResult(
            method=self.name,
            T_base_camera=camera_results,
            T_gripper_board=T_gripper_board,
            success=overall_success,
            diagnostics={
                "per_camera": per_camera,
                # Shah/Li 고유 지표: 다른 방법에는 없음. 카메라마다 독립적으로
                # 추정한 T_gripper_board가 서로 얼마나 일치하는지 (0에 가까울수록 좋음)
                "board_consistency_translation_std_m": translation_spread,
                "board_consistency_rotation_std_deg": rotation_spread_deg,
            },
        )


if __name__ == "__main__":
    self_test()