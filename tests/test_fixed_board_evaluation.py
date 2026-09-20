import numpy as np
from scipy.spatial.transform import Rotation

from SOTA_Simulation import fixed_board_evaluation as f


def transform(rotvec, xyz):
    value = np.eye(4)
    value[:3, :3] = Rotation.from_rotvec(rotvec).as_matrix()
    value[:3, 3] = xyz
    return value


def test_camera_mapping_matches_confirmed_realsense_order():
    assert f.CAMERA_FILES == {
        "0": "cam_039422061216",
        "1": "cam_fixed2",
        "2": "cam_gripper",
        "3": "cam_fixed3",
    }
    assert f.SERIALS["2"] == "752112070297"


def test_robot_pose_uses_rz_ry_rx_and_metre_translation():
    pose = f.robot_pose([100, -200, 300, 30, -20, 10])
    assert np.allclose(pose[:3, 3], [.1, -.2, .3])
    assert np.allclose(
        pose[:3, :3], Rotation.from_euler("ZYX", [30, -20, 10], degrees=True).as_matrix()
    )


def test_all_eye_in_hand_methods_recover_noiseless_case():
    rng = np.random.default_rng(710)
    gripper_camera = transform([.12, -.2, .09], [.04, -.02, .08])
    base_board = transform([-.3, .1, .2], [.25, .16, .04])
    records = []
    for event_id in range(15):
        base_gripper = transform(rng.uniform(-1, 1, 3), rng.uniform(-.3, .3, 3))
        camera_board = f.r.inv(gripper_camera) @ f.r.inv(base_gripper) @ base_board
        records.append(dict(
            id=event_id,
            robot=base_gripper,
            cams={"2": dict(visual=camera_board)},
        ))
    for method in f.r.METHODS:
        estimated_camera, estimated_board = f.fit_wrist(records, method)
        assert f.r.errors(estimated_camera, gripper_camera)[0] < 1e-4
        assert f.r.errors(estimated_board, base_board)[0] < 1e-4


def test_camera_pose_metric_is_equal_camera_macro():
    rows = []
    for camera, values in (("0", [1.0, 3.0]), ("1", [10.0])):
        for value in values:
            rows.append(dict(
                camera=camera,
                chain_translation_mm=value,
                chain_rotation_deg=value,
                camera_pose_translation_mm=value,
                camera_pose_rotation_deg=value,
                pixel_sse=1.0,
                corner_count=1,
            ))
    result = f.aggregate_scores(rows)
    assert result["heldout_translation_mm"] == 14 / 3
    assert result["camera_pose_translation_mm"] == 6.0


def test_fixed_camera_estimate_from_static_board():
    base_board = transform([.1, -.2, .05], [.3, .2, .1])
    base_cameras = {
        name: transform([.04 * index, -.02, .03], [.2 * index, -.1, .4])
        for index, name in enumerate(f.FIXED_CAMERAS)
    }
    records = []
    for event_id in range(5):
        records.append(dict(
            id=event_id,
            cams={
                name: dict(visual=f.r.inv(value) @ base_board)
                for name, value in base_cameras.items()
            },
        ))
    estimated = f.fixed_estimate_matrices(records, base_board)
    for name in f.FIXED_CAMERAS:
        assert f.r.errors(estimated[name], base_cameras[name])[0] < 1e-6
