"""Hardware-free safety/retention tests. Run with unittest, no pytest required."""
import copy
import json
from pathlib import Path
import sys
import tempfile
import threading
import time
import unittest
from unittest.mock import patch
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'capture'))
sys.path.insert(0,str(ROOT/'capture'/'robot'))
from zeus_capture_core import Guard, angle, clock, offset_pose, quality_plan, vector
from zeus_capture_server import Executor, sdk_ok, json_number
from assisted_capture import CaptureSession, parse_args
from assisted_camera import SyntheticCamera, analyze
from shah_capture_client import BoardDetector
from board_config import ROBOT_BOARD


class GuardTests(unittest.TestCase):
    def setUp(self):
        self.p=[400.,0.,300.,0.,20.,180.]

    def test_nonfinite_and_bool_rejected(self):
        for x in (float('nan'),float('inf'),True):
            with self.assertRaises(ValueError):
                vector([x,0,0,0,0,0])
        for text in ('NaN','Infinity','1e999'):
            with self.assertRaises(ValueError):
                json_number(text)

    def test_diagonal_translation_uses_norm(self):
        with self.assertRaises(ValueError):
            Guard().path(self.p,offset_pose(self.p,[2,2,0,0,0,0]),self.p)

    def test_world_rotation_and_euler_wrap_continuity(self):
        p=self.p[:]
        p[3]=179.9
        q=offset_pose(p,[0,0,0,.5,0,0])
        self.assertAlmostEqual(q[3],180.4)
        self.assertAlmostEqual(angle(p,q),.5)
        self.assertEqual(q[:3],p[:3])

    def test_intermediate_ik_jump_and_multiturn_not_wrapped(self):
        for path in ([[0]*6,[1.5,0,0,0,0,0],[0]*6],[[360,0,0,0,0,0]]):
            with self.assertRaises(ValueError):
                Guard().check_joints(path,[0]*6,[0]*6)

    def test_plan_preserves_anchor_and_checks_each_transition(self):
        plan=quality_plan(self.p)
        self.assertEqual(len(plan),24)
        prev=self.p
        for i,item in enumerate(plan):
            Guard().path(prev,item['target'],self.p)
            prev=item['target']
            if i%2:
                self.assertEqual(item['target'],self.p)

    def test_sdk_false_return_is_not_success(self):
        for r in (False,[False,3,16]):
            with self.assertRaises(RuntimeError):
                sdk_ok(r,'line')

    def test_verified_joint_limits_enforced_on_intermediate_points(self):
        guard=Guard(dict(verified=True,min_deg=[-1.]*6,max_deg=[1.]*6,margin_deg=.2))
        with self.assertRaises(ValueError):
            guard.check_joints([[0.]*6,[.9,0,0,0,0,0]],[0.]*6,[0.]*6)
        with self.assertRaises(ValueError):
            Guard(dict(verified=False,min_deg=[-1.]*6,max_deg=[1.]*6))


class ExecutorTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory()
        self.e=Executor('simulate',str(Path(self.tmp.name)/'log.jsonl'))
        self.session=self.e.rpc({'command':'connect'})['session']

    def tearDown(self):
        self.e.shutdown.set()
        self.e.log_file.close()
        self.tmp.cleanup()

    def rpc(self,c,**kw):
        return self.e.rpc(dict(command=c,session=self.session,**kw))

    def test_single_owner_and_no_replay(self):
        with self.assertRaises(ValueError):
            self.e.rpc({'command':'connect'})
        self.rpc('anchor',seq=1)
        with self.assertRaises(ValueError):
            self.rpc('anchor',seq=1)
        self.assertEqual(self.e.tasks.qsize(),1)

    def test_no_motion_queue_and_invalid_payload(self):
        self.rpc('anchor',seq=1)
        with self.assertRaises(ValueError):
            self.rpc('move',seq=2,target=self.e.snapshot['flange_pose_6dof'])

    def test_watchdog_latches_and_disarms(self):
        self.e.armed=True
        self.e.last_seen=clock()-3.
        t=threading.Thread(target=self.e.watchdog)
        t.start()
        time.sleep(.2)
        self.e.shutdown.set()
        t.join(1.)
        self.assertTrue(self.e.stop_event.is_set())
        self.assertFalse(self.e.armed)
        self.assertEqual(self.e.fault['reason'],'heartbeat_timeout')

    def test_guard_rejection_does_not_move(self):
        self.e.execute({'command':'anchor'})
        self.e.execute({'command':'arm'})
        before=self.e.backend.pose[:]
        target=before[:]
        target[0]+=3.
        with self.assertRaises(ValueError):
            self.e.execute({'command':'move','target':target})
        self.assertEqual(self.e.backend.pose,before)

    def test_stop_inhibits_move_and_keeps_first_reason(self):
        self.e.execute({'command':'anchor'})
        self.e.execute({'command':'arm'})
        before=self.e.backend.pose[:]
        self.e.stop('first failure')
        self.e.stop('later failure')
        with self.assertRaises(ValueError):
            self.rpc('move',seq=1,target=before)
        self.assertEqual(self.e.fault['reason'],'first failure')
        self.assertEqual(before,self.e.backend.pose)

    def test_logging_failure_cannot_prevent_stop_dispatch(self):
        self.e.mode='live'
        self.e.armed=True
        dispatched=[]
        self.e.start_stop_thread=lambda:dispatched.append(True)
        def full_disk(*a,**kw):
            raise OSError('disk full')
        self.e.log=full_disk
        self.e.stop('operator_stop')
        self.assertTrue(dispatched)
        self.assertTrue(self.e.stop_event.is_set())

    def test_midmove_exception_faults_and_records_original(self):
        self.e.execute({'command':'anchor'})
        self.e.execute({'command':'arm'})
        def broken(p):
            raise ValueError('injected SDK speed error')
        self.e.backend.move=broken
        t=threading.Thread(target=self.e.run)
        t.start()
        target=self.e.backend.pose[:]
        target[0]+=1.
        jid=self.rpc('move',seq=1,target=target)['job_id']
        deadline=time.monotonic()+2
        while not self.e.jobs[jid]['done'] and time.monotonic()<deadline:
            time.sleep(.02)
        self.e.shutdown.set()
        t.join(1.)
        self.assertFalse(self.e.jobs[jid]['ok'])
        self.assertFalse(self.e.jobs[jid]['rejected'])
        self.assertIn('injected SDK speed error',self.e.fault['reason'])


class CaptureTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory()
        self.args=parse_args(['--dataset-root',self.tmp.name,'--simulate-cameras','--no-preview'])
        self.camera=SyntheticCamera('cam0',1280,720,0)
        frame=self.camera.grab()
        self.sample=dict(frame=frame,sequence=1,quality=analyze(frame,self.camera,
                                BoardDetector(ROBOT_BOARD,0),self.args))
        self.state=dict(flange_pose_6dof=[400.,0.,300.,0.,20.,180.],joints_6dof=[0.]*6,
                   tool=1,tool_verified=True,flags={k:0 for k in ('emergency','hw_error','sw_error','abs_lost','paused','error')})
        state=self.state
        class Robot:
            mode='live'
            def state(self):
                return dict(state=copy.deepcopy(state),fault=None,moving=False,pending=False)
        sample=self.sample
        camera=self.camera
        class Rig:
            cameras=[camera]
            def fresh(self,start):
                s=copy.deepcopy(sample)
                s['frame']['host_monotonic_ts_ms']=(start+.01)*1000
                return {'cam0':s}
        self.robot=Robot()
        self.session=CaptureSession(self.args,self.robot,Rig(),threading.Event())
        self.session.stable=lambda:copy.deepcopy(self.state)

    def tearDown(self):
        self.tmp.cleanup()

    def test_generated_board_detects_all_24_corners(self):
        self.assertEqual(self.sample['quality']['charuco']['n_corners'],24)
        self.assertIsNone(self.sample['quality']['gate_reason'])

    def test_failed_detection_keeps_image_but_not_valid_selection(self):
        self.sample['quality']['gate_reason']='too_few_corners'
        r=self.session.capture('failure')
        self.assertTrue((self.session.path/r['cams']['0']['rgb_path']).exists())
        self.assertFalse(r['eligible_for_calibration'])
        self.assertEqual(len(json.loads((self.session.path/'meta.json').read_text())['captures']),1)
        self.assertEqual(json.loads((self.session.path/'valid_meta.json').read_text())['captures'],[])

    def test_motion_between_state_samples_invalidates_capture(self):
        before=copy.deepcopy(self.state)
        self.session.stable=lambda:before
        self.state['flange_pose_6dof'][0]+=1
        r=self.session.capture('moved')
        self.assertFalse(r['capture_gate']['pass'])
        self.assertIn('robot_state_or_stability_unverified',r['capture_gate']['reason'])

    def test_simulated_data_never_exported_as_calibration_input(self):
        self.robot.mode='simulate'
        r=self.session.capture('sim')
        self.assertTrue(r['capture_gate']['pass'])
        self.assertFalse(r['eligible_for_calibration'])

    def test_unknown_tool_reference_invalidates_capture(self):
        self.state['tool_verified']=False
        r=self.session.capture('unknown_tcp')
        self.assertFalse(r['eligible_for_calibration'])

    def test_nonfinite_detection_is_saved_as_failure_and_later_capture_recovers(self):
        original=copy.deepcopy(self.sample['quality'])
        q=self.sample['quality']
        q['charuco']['rvec']=[float('nan'),0.,0.]
        q['charuco']['tvec']=[0.,float('inf'),.5]
        q['charuco']['T_cam_board_4x4'][0][0]=float('nan')
        q['charuco']['reproj_error_px']=float('nan')
        q['score']=float('nan')
        r=self.session.capture('nonfinite',evaluation_only=True)
        self.assertFalse(r['capture_gate']['pass'])
        self.assertIn('nonfinite_detection',r['capture_gate']['reason'])
        self.assertTrue((self.session.path/r['cams']['0']['rgb_path']).exists())
        stored=json.loads((self.session.path/'meta.json').read_text(),
                          parse_constant=lambda x:self.fail('Invalid JSON number '+x))
        self.assertIsNone(stored['captures'][0]['cams']['0']['charuco']['T_cam_board_4x4'])
        self.session.event('operation_error',detail='regression check')
        self.sample['quality']=original
        self.assertTrue(self.session.capture('recovered')['capture_gate']['pass'])

    def test_pnp_refinement_nan_becomes_failed_pose(self):
        detector=BoardDetector(ROBOT_BOARD,0)
        with patch('cv2.solvePnPRefineLM',return_value=(np.full((3,1),np.nan),np.ones((3,1)))):
            q=analyze(self.sample['frame'],self.camera,detector,self.args)
        self.assertFalse(q['charuco']['ok'])
        self.assertEqual(q['charuco']['failure_reason'],'nonfinite_pose')
        self.assertGreater(q['charuco']['n_corners'],0)
        json.dumps(q,allow_nan=False)

    def test_collinear_corners_do_not_enter_pnp(self):
        detector=BoardDetector(ROBOT_BOARD,0)
        corners=np.array([[[20.,20.]],[[40.,20.]],[[60.,20.]],[[80.,20.]]],dtype=np.float32)
        ids=np.array([[0],[1],[2],[3]],dtype=np.int32)
        with patch.object(detector,'detector') as stub, patch('cv2.solvePnP') as solve:
            stub.detectBoard.return_value=(corners,ids,None,None)
            q=analyze(self.sample['frame'],self.camera,detector,self.args)
        solve.assert_not_called()
        self.assertEqual(q['charuco']['n_corners'],4)
        self.assertIn('collinear_corners',q['gate_reason'])

    def test_scaled_detection_maps_back_before_pose_estimation(self):
        detector=BoardDetector(ROBOT_BOARD,0,3)
        original=np.array([[[10.,20.]],[[30.,20.]],[[10.,40.]],[[30.,40.]]],dtype=np.float32)
        enlarged=(original+.5)*3-.5
        ids=np.array([[0],[1],[6],[7]],dtype=np.int32)
        frame=self.sample['frame']['color']
        before=frame.copy()
        K=self.camera.K.copy()
        with patch.object(detector,'detector') as stub, patch('cv2.solvePnP',return_value=(False,None,None)) as solve:
            stub.detectBoard.return_value=(enlarged,ids,None,None)
            result=detector.detect(frame,self.camera.K,self.camera.D)
        np.testing.assert_allclose(result['_draw'][0],original)
        np.testing.assert_allclose(np.asarray(solve.call_args.args[1]).reshape(-1,2),original.reshape(-1,2))
        np.testing.assert_array_equal(solve.call_args.args[2],K)
        np.testing.assert_array_equal(frame,before)
        self.assertEqual(stub.detectBoard.call_args.args[0].shape,(frame.shape[0]*3,frame.shape[1]*3))

    def test_scaled_synthetic_pose_and_error_use_original_pixels(self):
        import cv2
        frame=self.sample['frame']['color']
        detector=BoardDetector(ROBOT_BOARD,0,3)
        r=detector.detect(frame,self.camera.K,self.camera.D)
        self.assertEqual(r['n_corners'],24)
        self.assertTrue(r['ok'])
        obj,img=detector.grid.matchImagePoints(*r['_draw'])
        projected,_=cv2.projectPoints(obj,np.array(r['rvec']),np.array(r['tvec']),self.camera.K,self.camera.D)
        expected=float(np.sqrt(np.mean(np.sum((projected.reshape(-1,2)-img.reshape(-1,2))**2,axis=1))))
        self.assertAlmostEqual(r['reproj_error_px'],expected,places=6)
        baseline=self.sample['quality']['charuco']
        self.assertLess(np.linalg.norm(np.array(r['tvec'])-np.array(baseline['tvec'])),.002)
        self.assertEqual(r['corner_coordinate_space'],'original_image_pixels')

    def test_scaled_detection_saves_original_image_and_metadata(self):
        import cv2
        self.sample['quality']=analyze(self.sample['frame'],self.camera,
                                      BoardDetector(ROBOT_BOARD,0,3),self.args)
        r=self.session.capture('scaled')
        saved=cv2.imread(str(self.session.path/r['cams']['0']['rgb_path']))
        np.testing.assert_array_equal(saved,self.sample['frame']['color'])
        self.assertEqual(r['cams']['0']['charuco']['detection_scale'],3)
        self.assertEqual(self.session.meta['capture_config']['detection_scale'],3)


if __name__=='__main__':
    unittest.main()
