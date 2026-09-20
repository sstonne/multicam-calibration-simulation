"""공통 evaluation runner가 README 11절 기록 규칙을 지키는지 검증한다.

- 11-7: 실패한 trial을 삭제하지 않고 failure로 기록한다
- 11-9: camera별 결과와 system-level 결과를 함께 보관한다
- 11-10: 사용한 구현과 dependency version을 기록한다
"""

import math

import numpy as np
import pytest

from SOTA_Simulation import opencv_multicam_evaluation as ev
from SOTA_Simulation.sota_simulation import generate_case, load_config
from SOTA_Simulation.tsai_combined_demo import build_eye_in_hand_session

CAMERAS = ("cam0", "cam1", "cam3", "wrist")
CONFIG = ev.Path(__file__).resolve().parents[1] / "SOTA_Simulation" / "config.example.json"


@pytest.fixture(scope="module")
def session():
    config, config_path = load_config(CONFIG)
    config["simulation"]["pixel_noise_sigma"] = 0.0
    config["simulation"]["corner_dropout_probability"] = 0.0
    config["simulation"]["camera_event_dropout_probability"] = 0.0
    case = generate_case(config, config_path)
    wrist = build_eye_in_hand_session(
        int(config["simulation"]["number_of_events"]))
    return case, wrist, ev.load_wrist_camera(config_path)


def evaluate(session, methods, trials=1, noise=(1.0,)):
    case, wrist, wrist_camera = session
    return ev.run_evaluation(
        case, wrist, wrist_camera, list(noise), trials, 2026,
        list(methods), list(ev.DEFAULT_HELDOUT),
    )


def test_records_carry_per_camera_metrics(session):
    """camera별 열이 있고, camera pose 평균이 통합 값과 일치한다 (README 8.2/11-9)."""
    records, _, _ = evaluate(session, ["park"])
    record = records[0]
    for camera in CAMERAS:
        for metric in ev.PER_CAMERA_METRICS:
            assert f"{camera}_{metric}" in record

    macro = np.mean([record[f"{c}_camera_pose_translation_error_mm"] for c in CAMERAS])
    assert macro == pytest.approx(record["camera_pose_translation_error_mm"], abs=1e-12)
    macro_rotation = np.mean(
        [record[f"{c}_camera_pose_rotation_error_deg"] for c in CAMERAS])
    assert macro_rotation == pytest.approx(
        record["camera_pose_rotation_error_deg"], abs=1e-12)


def test_failed_trial_is_kept_as_failure(session, monkeypatch):
    """solver가 던져도 trial을 버리지 않고 NaN + status로 남긴다 (README 11-7)."""
    original = ev.solve_hand_eye
    calls = {"count": 0}

    def flaky(*args, **kwargs):
        calls["count"] += 1
        if calls["count"] == 1:
            raise RuntimeError("synthetic solver failure")
        return original(*args, **kwargs)

    monkeypatch.setattr(ev, "solve_hand_eye", flaky)
    records, _, _ = evaluate(session, ["park"], trials=2)

    assert len(records) == 2, "실패한 trial이 사라지면 안 된다"
    failed = [r for r in records if r["status"] != "ok"]
    healthy = [r for r in records if r["status"] == "ok"]
    assert len(failed) == 1 and len(healthy) == 1
    assert failed[0]["status"].startswith("solver_error")
    assert set(failed[0]) == set(healthy[0])
    for metric in ev.INTEGRATED_METRICS:
        assert math.isnan(failed[0][metric])

    summary = ev.summarise(records, ["park"], [1.0])["park"]["1.0"]
    assert summary["trial_count"] == 2
    assert summary["failure_count"] == 1
    # 통계는 살아남은 trial만 쓴다
    assert summary["heldout_translation_error_mm_mean"] == pytest.approx(
        healthy[0]["heldout_translation_error_mm"])


def test_provenance_records_versions():
    """README 11-10: 구현 출처와 dependency version을 남긴다."""
    recorded = ev.provenance()
    for key in ("python", "numpy", "scipy", "opencv"):
        assert recorded[key]
    assert "calibration_handeye.cpp" in recorded[
        "tsai_park_horaud_andreff_daniilidis_shah"]
    assert "tabb_solver.py" in recorded["tabb"]
