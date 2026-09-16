import copy
import cv2
import numpy as np
import pytest
from scipy.spatial.transform import Rotation
from SOTA_Simulation import unified_real_evaluation as u


def pose(rot, xyz):
    t = np.eye(4)
    t[:3, :3] = Rotation.from_rotvec(rot).as_matrix()
    t[:3, 3] = xyz
    return t


def test_unicode_rgb_path(tmp_path):
    image = np.full((24,32,3), 127, dtype=np.uint8)
    ok, data = cv2.imencode('.png', image)
    assert ok
    path = tmp_path / '\uBB38\uC11C.png'
    path.write_bytes(data.tobytes())
    assert np.array_equal(u.read_rgb(path), image)


@pytest.fixture
def merged_case():
    rng = np.random.default_rng(129)
    mount = pose([-.12, .04, .08], [.02, -.03, .06])
    ys = {name: pose([.1, -.2, .05], [i*.1, -.05, -1.2]) for i, name in enumerate(u.CAMERAS)}
    cameras = {name:dict(K=np.array([[900.,0,640],[0,910,360],[0,0,1]]), D=np.zeros(5)) for name in u.CAMERAS}
    points = np.array([[i*.02,j*.02,0] for j in range(4) for i in range(6)])
    records = []
    for i in range(16):
        robot = pose(rng.uniform(-.8,.8,3), rng.uniform(-.12,.12,3))
        cams = {}
        for name in u.CAMERAS:
            visual = u.r.inv(ys[name]) @ robot @ mount
            cams[name] = dict(visual=visual, points=points.copy(), pixels=u.r.project(visual,points,cameras[name]))
        records.append(dict(id=i,key=f'session:{i}',source_session='session',source_event_id=i,robot=robot,cams=cams))
    return records,cameras,ys,mount


def test_low_quality_pose_included_and_partner_missing_independent():
    raw = dict(robot_pose_matrix_4x4=np.eye(4), capture_gate=dict(pass_=False), cams={
        '0':dict(saved=False,charuco=dict(ok=False,n_corners=2,reproj_error_px=200,
            foreign_marker_ids=[5],T_cam_board_4x4=np.eye(4))), '1':dict(charuco={})})
    a,b = u.usable_observation(raw,'0')
    assert np.array_equal(a,b)
    with pytest.raises(ValueError):
        u.usable_observation(raw,'1')
    raw['cams']['0']['charuco']['T_cam_board_4x4'][0,0] = np.nan
    with pytest.raises(ValueError,match='nonfinite'):
        u.usable_observation(raw,'0')


@pytest.mark.parametrize('method',u.r.METHODS)
def test_merged_noiseless_recovery_and_registration(merged_case,method):
    records,cameras,ys,mount = merged_case
    # Camera-specific missing measurements must not force a shared-camera mask.
    del records[0]['cams']['1']
    fits = u.fit_cameras(records[:12],method)
    assert fits['0']['count']==12 and fits['1']['count']==11
    assert all(x['success'] for x in fits.values())
    for name in u.CAMERAS:
        assert u.r.errors(np.array(fits[name]['T_base_camera']),ys[name])[0] < 1e-4
        assert u.r.errors(np.array(fits[name]['T_gripper_board']),mount)[0] < 1e-4
    metrics = u.r.aggregate(u.score(records[12:],cameras,fits,method,'heldout'))
    assert metrics['reprojection_rmse_px'] < 1e-5
    assert metrics['chain_translation_mm'] < 1e-4
    reg = u.registration(records[12:],fits)
    assert len(reg)==12 and max(x['translation_mm'] for x in reg)<1e-4


def test_groups_keep_every_observation_and_do_not_split_duplicate(merged_case):
    records = merged_case[0]
    duplicate = copy.deepcopy(records[3])
    duplicate.update(id=100,key='other_session:3')
    records.append(duplicate)
    train,held,_ = u.split_records(records)
    assert len(train)+len(held)==17
    assert {3,100} <= {x['id'] for x in held}
    assert not {x['id'] for x in train} & {x['id'] for x in held}


def test_heldout_mutations_do_not_change_frozen_fit(merged_case):
    records,cameras,_,_ = merged_case
    fits = u.fit_cameras(records[:12],'park')
    baseline = u.r.aggregate(u.score(records[12:],cameras,fits,'park','heldout'))
    for record in records[12:]:
        for cam in record['cams'].values():
            cam['pixels'] += 50
            cam['visual'][:3,3] += .01
    assert fits == u.fit_cameras(records[:12],'park')
    changed = u.r.aggregate(u.score(records[12:],cameras,fits,'park','heldout'))
    assert changed['reprojection_rmse_px'] > baseline['reprojection_rmse_px']+50


def test_missing_pixels_keeps_pose_but_marks_metric_incomplete(merged_case):
    records,cameras,_,_ = merged_case
    records[15]['cams']['0']['pixels'] = np.empty((0,2))
    fits = u.fit_cameras(records[:12],'shah')
    metrics = u.r.aggregate(u.score(records[12:],cameras,fits,'shah','heldout'))
    assert metrics['chain_translation_mm'] < 1e-4
    assert metrics['reprojection_rmse_px'] is None
    assert not metrics['reprojection_complete']
