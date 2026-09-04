import sys
from pathlib import Path

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "capture"))
import calibrate_realsense_intrinsics as calibration


def test_candidate_schema_matches_existing_reader_contract(tmp_path):
    reference_path = ROOT / "intrinsics" / "cam0.npz"
    with np.load(reference_path, allow_pickle=False) as reference:
        payload = {key: reference[key].copy() for key in reference.files}
    payload["serial"] = np.asarray("039422061216")
    candidate_path = tmp_path / "cam0_candidate_039422061216.npz"
    np.savez(candidate_path, **payload)

    assert calibration.validate_candidate(
        candidate_path, reference_path, "039422061216") == []


def test_invalid_serial_is_detected_after_reload(tmp_path):
    reference_path = ROOT / "intrinsics" / "cam0.npz"
    with np.load(reference_path, allow_pickle=False) as reference:
        payload = {key: reference[key].copy() for key in reference.files}
    candidate_path = tmp_path / "invalid_candidate.npz"
    np.savez(candidate_path, **payload)

    errors = calibration.validate_candidate(candidate_path, reference_path, "039422061216")
    assert any("serial 불일치" in error for error in errors)
