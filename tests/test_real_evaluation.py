import json

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from SOTA_Simulation import real_evaluation as real


def make_pose(rotation, translation):
    result = np.eye(4)
    result[:3, :3] = Rotation.from_rotvec(rotation).as_matrix()
    result[:3, 3] = translation
    return result


@pytest.fixture
def noiseless_case():
    rng = np.random.default_rng(129)
    camera = dict(K=np.array([[900., 0, 640], [0, 910, 360], [0, 0, 1]]), D=np.zeros(5))
    y = make_pose([.1, -.2, .05], [.1, -.05, -1.2])
    x = make_pose([-.12, .04, .08], [.02, -.03, .06])
    points = np.array([[i*.02, j*.02, 0] for j in range(4) for i in range(6)])
    records = []
    for i in range(16):
        robot = make_pose(rng.uniform(-.8, .8, 3), rng.uniform(-.12, .12, 3))
        visual = real.inv(y) @ robot @ x
        pixels = real.project(visual, points, camera)
        records.append(dict(id=i, robot=robot, cams={"0":dict(visual=visual, pixels=pixels, points=points)}))
    return records, camera, y, x


@pytest.mark.parametrize("method", real.METHODS)
def test_noiseless_extrinsic_mount_and_heldout_recovery(noiseless_case, method):
    records, camera, true_y, true_x = noiseless_case
    y, x = real.fit(records[:12], "0", method)
    assert real.errors(y, true_y)[0] < 1e-4
    assert real.errors(y, true_y)[1] < 1e-4
    assert real.errors(x, true_x)[0] < 1e-4
    scores = real.aggregate(real.score(records[12:], "0", camera, y, x))
    assert scores["reprojection_rmse_px"] < 1e-5
    assert scores["chain_translation_mm"] < 1e-4


def test_near_duplicate_groups_never_cross_split(noiseless_case):
    records = noiseless_case[0]
    repeated = dict(records[3], id=100)
    records = records + [repeated]
    train, test, groups = real.make_split(records, dict(near_translation_mm=1.5, near_rotation_deg=1.))
    train_ids, test_ids = {r["id"] for r in train}, {r["id"] for r in test}
    assert train_ids.isdisjoint(test_ids)
    assert {3, 100}.issubset(test_ids)
    assert sorted(train_ids | test_ids) == sorted(r["id"] for r in records)


def test_heldout_corruption_does_not_change_fit(noiseless_case):
    records, camera, _, _ = noiseless_case
    y, x = real.fit(records[:12], "0", "park")
    baseline = real.aggregate(real.score(records[12:], "0", camera, y, x))
    for record in records[12:]:
        record["cams"]["0"]["pixels"] += 50
        record["cams"]["0"]["visual"][:3, 3] += .01
    y_after, x_after = real.fit(records[:12], "0", "park")
    assert np.array_equal(y, y_after)
    assert np.array_equal(x, x_after)
    changed = real.aggregate(real.score(records[12:], "0", camera, y_after, x_after))
    assert changed["reprojection_rmse_px"] > baseline["reprojection_rmse_px"] + 50


def test_missing_pixels_not_silently_scored_as_complete(noiseless_case):
    records, camera, y, x = noiseless_case
    records[0]["cams"]["0"]["pixels"] = np.empty((0, 2))
    score = real.aggregate(real.score(records, "0", camera, y, x))
    assert score["reprojection_rmse_px"] is None
    assert not score["reprojection_complete"]


def test_tsai_small_rotation_failure_not_identity_success(noiseless_case):
    records, _, y, x = noiseless_case
    for i, record in enumerate(records):
        record["robot"] = make_pose([i * .001, 0, 0], [i * .001, 0, 0])
        record["cams"]["0"]["visual"] = real.inv(y) @ record["robot"] @ x
    with pytest.raises(ValueError, match="tsai_insufficient_informative_motion_pairs:0"):
        real.fit(records, "0", "tsai")


def test_invalid_transform_and_path_rejected(tmp_path):
    invalid = np.eye(4)
    invalid[0, 0] = -1
    with pytest.raises(ValueError, match="SO3"):
        real.transform(invalid)
    invalid[0, 0] = np.nan
    with pytest.raises(ValueError, match="nonfinite"):
        real.transform(invalid)
    with pytest.raises(ValueError, match="outside_session"):
        real.relative_file(tmp_path, "../outside.png")


def test_manifest_cannot_include_excluded_backup(tmp_path):
    manifest = json.loads((real.ROOT / "datasets/real_analysis_manifest.json").read_text())
    assert all("20260904" not in s["meta"] for s in manifest["sessions"])
    manifest["sessions"][0]["name"] = "session_20260904_173735"
    path = tmp_path / "manifest.json"
    path.write_text(json.dumps(manifest))
    with pytest.raises(SystemExit):
        real.main(["--manifest", str(path), "--output", str(tmp_path / "output")])
    assert not (tmp_path / "output").exists()
