"""Bounded automatic collection tests. No robot network or camera access."""
import copy
import json
from pathlib import Path
import sys
import tempfile
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'capture'))
from assisted_search import AutoCollector, assess, coverage, distinct
from assisted_capture import RobotLink
from robot.zeus_capture_core import Guard, distance, angle, offset_pose


def record(i, pose=None, passed=True):
    p=pose or [0.,0.,300.,0.,20.,180.]
    matrix=np.eye(4)
    matrix[2,3]=.5
    cams={name:dict(saved=True,frame_sequence=i,gate_reason=None if passed else 'too_few_corners',
                    charuco=dict(n_corners=16 if passed else 6,T_cam_board_4x4=matrix.tolist()))
          for name in ('1','3')}
    state=dict(flange_pose_6dof=list(p),joints_6dof=[0.]*6)
    return dict(event_id=i,cams=cams,capture_gate={'pass':passed},quality_score=.5 if passed else .1,
                robot_pose_6dof=list(p),robot_before=state,robot_after=copy.deepcopy(state),
                eligible_for_calibration=False,evaluation_only=True)


class QualityTests(unittest.TestCase):
    def test_isolated_pass_and_duplicate_frames_not_accepted(self):
        rows=[record(i,passed=(i==0)) for i in range(5)]
        self.assertFalse(assess(rows,['1','3'])['robust'])
        rows=[record(0) for _ in range(5)]
        self.assertFalse(assess(rows,['1','3'])['robust'])

    def test_four_joint_passes_required_and_pose_jitter_rejected(self):
        rows=[record(i,passed=(i!=2)) for i in range(5)]
        self.assertTrue(assess(rows,['1','3'])['robust'])
        rows[4]['cams']['3']['charuco']['T_cam_board_4x4'][0][3]=.01
        self.assertFalse(assess(rows,['1','3'])['robust'])

    def test_good_scores_do_not_override_pair_gate(self):
        rows=[record(i,passed=False) for i in range(5)]
        for r in rows:
            r['quality_score']=.99
        self.assertFalse(assess(rows,['1','3'])['robust'])

    def test_identical_and_translation_only_data_not_diverse(self):
        p=record(0)['robot_pose_6dof']
        self.assertFalse(distinct(p,[record(0)]))
        rows=[record(i,offset_pose(p,[i*2,0,0,0,0,0])) for i in range(10)]
        self.assertFalse(coverage(rows)['diverse'])
        rows=[record(i,offset_pose(p,[0,0,0,i,0,0])) for i in range(5)]
        self.assertFalse(coverage(rows)['diverse'])
        rows=[record(i,offset_pose(p,d)) for i,d in enumerate(
            ([0,0,0,0,0,0],[0,0,0,4,0,0],[0,0,0,0,4,0],[0,0,0,0,0,4]))]
        self.assertTrue(coverage(rows)['diverse'])


class SearchTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory()
        self.pose=[0.,0.,300.,0.,20.,180.]
        self.anchor=dict(pose=self.pose[:],joints=[0.]*6)
        self.cancel=threading.Event()
        self.moves=[]
        self.commands=[]
        self.armed=True
        self.fail_move=False
        self.lost=False
        self.low_quality=False
        self.simulation=False
        outer=self
        class Robot:
            mode='simulate'
            def state(self):
                return dict(anchor=copy.deepcopy(outer.anchor),armed=outer.armed,
                            fault=None,moving=False,pending=False,
                            state=dict(flange_pose_6dof=outer.pose[:]))
            def operate(self,cmd):
                outer.commands.append(cmd)
                if cmd=='disarm':
                    outer.armed=False
        class Rig:
            cameras=[SimpleNamespace(index=1),SimpleNamespace(index=3)]
            def fresh(self,*args,**kwargs):
                return {n:dict(quality={'charuco':{'n_corners':0 if outer.lost else 16}})
                        for n in ('cam1','cam3')}
        args=SimpleNamespace(cameras=['cam1','cam3'],auto_radius_mm=5.,auto_angle_deg=5.,
                             auto_max_candidates=20,auto_max_minutes=2.,min_corners=12)
        class Session:
            def __init__(self):
                self.args=args
                self.rig=Rig()
                self.path=Path(outer.tmp.name)
                self.meta={'captures':[]}
            def capture(self,label,evaluation_only=False):
                r=record(len(self.meta['captures']),outer.pose,not outer.low_quality)
                r['evaluation_only']=evaluation_only
                self.meta['captures'].append(r)
                return r
            def flush(self):
                pass
            def event(self,*args,**kwargs):
                pass
        def move(p):
            if outer.fail_move:
                raise ValueError('SDK rejected before motion')
            Guard().path(outer.pose,p,outer.anchor['pose'])
            outer.moves.append(p[:])
            outer.pose=p[:]
        self.op=SimpleNamespace(robot=Robot(),session=Session(),cancel=self.cancel,move=move)
        self.auto=AutoCollector(self.op)

    def tearDown(self):
        self.tmp.cleanup()

    def test_collection_progress_stays_bounded_and_simulation_never_exported(self):
        original=copy.deepcopy(self.anchor)
        self.auto.run(3)
        self.assertEqual(self.commands,['disarm'])
        self.assertEqual(self.anchor,original)
        self.assertGreater(len(self.auto.accepted),1)
        self.assertTrue(self.moves)
        for p in self.moves:
            self.assertLessEqual(distance(p,original['pose']),5.00001)
            self.assertLessEqual(angle(p,original['pose']),5.00001)
        selected=json.loads(next(Path(self.tmp.name).glob('auto_valid_meta*')).read_text())
        self.assertEqual(selected['captures'],[])
        self.assertTrue(all(r['auto_validation']['joint_passes']==5 for r in self.auto.accepted))

    def test_rejection_stops_without_retry_or_anchor_reset(self):
        self.fail_move=True
        with self.assertRaises(ValueError):
            self.auto.run(3)
        self.assertEqual(self.commands,['disarm'])
        self.assertEqual(self.moves,[])
        self.assertEqual(self.auto.report['status'],'aborted')

    def test_default_goal_reaches_thirty_distinct_poses_with_multiaxis_rotation(self):
        self.op.session.args.auto_max_candidates=120
        self.op.robot.mode='live'  # fake Robot only; never accesses a controller
        self.auto.run(30)
        self.assertEqual(self.auto.report['status'],'complete')
        self.assertGreaterEqual(len(self.auto.accepted),30)
        self.assertTrue(self.auto.report['coverage']['diverse'])
        selected=json.loads(next(Path(self.tmp.name).glob('auto_valid_meta*')).read_text())
        self.assertEqual(len(selected['captures']),len(self.auto.accepted))
        for i,r in enumerate(self.auto.accepted):
            self.assertTrue(distinct(r['robot_pose_6dof'],self.auto.accepted[:i]))

    def test_camera_loss_halts_without_automatic_return(self):
        self.lost=True
        self.auto.run(3)
        self.assertEqual(len(self.moves),1)
        self.assertEqual(self.auto.report['status'],'paused')
        self.assertFalse(self.armed)

    def test_cancel_disarms_without_moving(self):
        self.cancel.set()
        self.auto.run(3)
        self.assertEqual(self.moves,[])
        self.assertEqual(self.auto.report['status'],'paused')
        self.assertFalse(self.armed)

    def test_unsuccessful_search_reports_incomplete_and_keeps_evidence(self):
        self.low_quality=True
        self.op.session.args.auto_max_candidates=3
        self.auto.run(3)
        self.assertEqual(self.auto.report['status'],'incomplete')
        self.assertEqual(self.auto.accepted,[])
        self.assertEqual(len(self.op.session.meta['captures']),15)

    def test_cam2_rejected_before_motion(self):
        self.op.session.args.cameras=['cam1','cam2']
        with self.assertRaises(ValueError):
            self.auto.run(3)
        self.assertFalse(self.moves)


class RejectionTests(unittest.TestCase):
    def test_explicit_rejection_does_not_mark_link_broken(self):
        link=RobotLink.__new__(RobotLink)
        link.broken=threading.Event()
        link.seq=0
        with patch.object(link,'raw',side_effect=ValueError('FAULT latched')), patch.object(link,'stop') as stop:
            with self.assertRaises(ValueError):
                link.operate('anchor')
            self.assertFalse(link.broken.is_set())
            stop.assert_not_called()


if __name__=='__main__':
    unittest.main()
