from pathlib import Path

from SOTA_Simulation.opencv_multicam_evaluation import (
    load_wrist_camera,
    run_evaluation,
)
from SOTA_Simulation.sota_simulation import generate_case, load_config
from SOTA_Simulation.tsai_combined_demo import (
    HAND_EYE_METHODS,
    build_eye_in_hand_session,
)


ROOT = Path(__file__).resolve().parents[1]
CONFIG = ROOT / "SOTA_Simulation" / "config.example.json"


def test_all_opencv_methods_have_disjoint_heldout_evaluation():
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
        methods=tuple(HAND_EYE_METHODS),
        heldout_events=(2, 5, 9, 12),
    )

    assert set(train).isdisjoint(heldout)
    assert len(train) == 10
    assert len(heldout) == 4
    assert len(records) == len(HAND_EYE_METHODS)
    for record in records:
        assert record["camera_pose_translation_error_mm"] < 1e-4
        assert record["pairwise_registration_translation_error_mm"] < 1e-4
        assert record["heldout_reprojection_rmse_px"] < 1e-3
