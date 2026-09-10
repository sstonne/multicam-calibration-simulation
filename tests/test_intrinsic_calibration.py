import sys
from pathlib import Path

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "capture"))
import calibrate_realsense_intrinsics as calibration


def canonical_payload(serial):
    K = np.array([[900.0, 0.0, 640.0], [0.0, 900.0, 360.0], [0.0, 0.0, 1.0]])
    D = np.zeros((5, 1), dtype=np.float64)
    return {
        "serial": np.asarray(serial), "is_gripper": np.asarray(False),
        "color_K": K, "color_D": D, "depth_K": K, "depth_D": D,
        "depth_scale_m_per_unit": np.asarray(0.001, dtype=np.float64),
        "color_w": np.asarray(1280, dtype=np.int64),
        "color_h": np.asarray(720, dtype=np.int64), "fps": np.asarray(15, dtype=np.int64),
        "R_depth_to_color": np.eye(3), "t_depth_to_color": np.zeros((3, 1)),
        "factory_color_K": K, "factory_color_D": D,
        "intrinsics_source": np.asarray("charuco"),
        "charuco_reproj_error_px": np.asarray(0.3, dtype=np.float64),
        "charuco_num_views": np.asarray(20, dtype=np.int64),
        "charuco_calibrated_at": np.asarray("2026-09-04 12:00:00"),
    }


def test_candidate_schema_matches_existing_reader_contract(tmp_path):
    reference_path = tmp_path / "reference.npz"
    np.savez(reference_path, **canonical_payload("314522062542"))
    candidate_path = tmp_path / "cam0_candidate_039422061216.npz"
    np.savez(candidate_path, **canonical_payload("039422061216"))

    assert calibration.validate_candidate(
        candidate_path, reference_path, "039422061216") == []


def test_invalid_serial_is_detected_after_reload(tmp_path):
    reference_path = tmp_path / "reference.npz"
    np.savez(reference_path, **canonical_payload("314522062542"))
    candidate_path = tmp_path / "invalid_candidate.npz"
    np.savez(candidate_path, **canonical_payload("314522062542"))

    errors = calibration.validate_candidate(candidate_path, reference_path, "039422061216")
    assert any("serial 불일치" in error for error in errors)
