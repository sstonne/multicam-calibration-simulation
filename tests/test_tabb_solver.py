"""Tabb & Ahmad Yousef (2017) zero-noise 검증.

README 6.1 "2단계 — Tabb" 완료 조건: 무잡음 데이터에서 ground truth를 복원하고,
기존 방법과 동일한 trajectory·split·평가 코드로 결과가 나와야 한다.
"""

from pathlib import Path

import numpy as np
import pytest

from SOTA_Simulation.opencv_multicam_evaluation import (
    TABB_LADDER,
    TABB_METHODS,
    load_wrist_camera,
    run_evaluation,
)
from SOTA_Simulation.sota_simulation import generate_case, load_config
from SOTA_Simulation.tabb_solver import (
    ROTATION_PARAMETERIZATIONS,
    TabbConfig,
    solve_tabb_eye_in_hand,
    solve_tabb_eye_to_hand,
    self_test,
)
from SOTA_Simulation.tsai_combined_demo import build_eye_in_hand_session

ROOT = Path(__file__).resolve().parents[1]
CONFIG = ROOT / "SOTA_Simulation" / "config.example.json"


def test_solver_self_test_passes():
    """c1/c2 × simultaneous/separable × 3 parameterization + rp1/rp2 전부 통과."""
    assert self_test(verbose=False)


@pytest.mark.parametrize("rotation", ROTATION_PARAMETERIZATIONS)
@pytest.mark.parametrize("cost", ["c1", "c2"])
@pytest.mark.parametrize("separable", [False, True])
def test_eye_in_hand_recovers_ground_truth(cost, separable, rotation):
    rng = np.random.default_rng(11)
    from scipy.spatial.transform import Rotation

    def transform(rotation_matrix, translation):
        output = np.eye(4)
        output[:3, :3] = rotation_matrix
        output[:3, 3] = translation
        return output

    def inverse(T):
        return transform(T[:3, :3].T, -T[:3, :3].T @ T[:3, 3])

    T_gripper_wrist = transform(
        Rotation.from_rotvec([0.05, -0.12, 0.3]).as_matrix(),
        [0.035, -0.008, 0.065],
    )
    T_base_board = transform(
        Rotation.from_rotvec([0.0, 0.0, 0.21]).as_matrix(), [0.0, 0.02, 0.08]
    )
    robot, visual = [], []
    for _ in range(10):
        q = rng.normal(size=4)
        T_base_gripper = transform(
            Rotation.from_quat(q / np.linalg.norm(q)).as_matrix(),
            rng.uniform(-0.5, 0.5, 3),
        )
        robot.append(T_base_gripper)
        visual.append(
            inverse(T_gripper_wrist) @ inverse(T_base_gripper) @ T_base_board
        )

    result = solve_tabb_eye_in_hand(
        robot, visual,
        TabbConfig(cost=cost, separable=separable, rotation=rotation),
    )
    assert np.allclose(result.T_gripper_wrist, T_gripper_wrist, atol=1e-9)
    assert np.allclose(result.T_base_board, T_base_board, atol=1e-9)


def test_joint_multi_eye_shares_one_board_transform():
    """Sec. 2.3: 카메라 3대가 X(=board의 gripper 오프셋) 하나를 공유해도 복원된다."""
    from scipy.spatial.transform import Rotation

    rng = np.random.default_rng(5)

    def transform(rotation_matrix, translation):
        output = np.eye(4)
        output[:3, :3] = rotation_matrix
        output[:3, 3] = translation
        return output

    def inverse(T):
        return transform(T[:3, :3].T, -T[:3, :3].T @ T[:3, 3])

    names = ["cam0", "cam1", "cam3"]
    T_gripper_board = transform(np.eye(3), [0.0, 0.0, 0.10])
    T_base_fixed = {}
    for index, name in enumerate(names):
        q = rng.normal(size=4)
        T_base_fixed[name] = transform(
            Rotation.from_quat(q / np.linalg.norm(q)).as_matrix(),
            rng.uniform(-0.6, 0.6, 3),
        )
    robot = []
    for _ in range(10):
        q = rng.normal(size=4)
        robot.append(transform(
            Rotation.from_quat(q / np.linalg.norm(q)).as_matrix(),
            rng.uniform(-0.4, 0.4, 3),
        ))
    visual = [[inverse(T_base_fixed[name]) @ T @ T_gripper_board for T in robot]
              for name in names]

    result = solve_tabb_eye_to_hand(
        names, [robot] * len(names), visual,
        TabbConfig(cost="c2", separable=False), joint=True,
    )
    for name in names:
        assert np.allclose(result.T_base_fixed[name], T_base_fixed[name], atol=1e-9)
    assert np.allclose(result.T_gripper_board, T_gripper_board, atol=1e-9)


def test_tabb_ladder_passes_noiseless_evaluation():
    """공통 evaluation runner에서도 noise 0이면 네 지표가 모두 0에 수렴한다."""
    config, config_path = load_config(CONFIG)
    case = generate_case(config, config_path)
    wrist = build_eye_in_hand_session(14)
    records, train, heldout = run_evaluation(
        case,
        wrist,
        load_wrist_camera(config_path),
        noise_levels=[0.0],
        trials=1,
        seed=7,
        methods=tuple(TABB_LADDER),
        heldout_events=(2, 5, 9, 12),
    )

    assert set(train).isdisjoint(heldout)
    assert len(records) == len(TABB_LADDER)
    for record in records:
        assert record["camera_pose_translation_error_mm"] < 1e-4
        assert record["camera_pose_rotation_error_deg"] < 1e-5
        assert record["pairwise_registration_translation_error_mm"] < 1e-4
        assert record["heldout_reprojection_rmse_px"] < 1e-3


def test_rp_variants_reject_missing_pixel_observations():
    with pytest.raises(ValueError):
        solve_tabb_eye_in_hand(
            [np.eye(4)] * 3, [np.eye(4)] * 3, TabbConfig(cost="rp1"),
        )


def test_separable_reprojection_is_not_a_paper_method():
    with pytest.raises(ValueError):
        TabbConfig(cost="rp1", separable=True)


def test_registry_labels_match_configs():
    assert TABB_METHODS["tabb"][0].label == "c2_sim_axis_angle"
    assert TABB_METHODS["tabb_c1_sep"][0].label == "c1_sep_axis_angle"
    assert TABB_METHODS["tabb_rp1_joint"][1] is True
