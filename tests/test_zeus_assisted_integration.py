"""Local TCP tests: starts only --simulate with Python 2; no i611 imports."""
import json
import os
from pathlib import Path
import shutil
import signal
import socket
import subprocess
import sys
import tempfile
import time
import unittest

ROOT=Path(__file__).resolve().parents[1]


@unittest.skipUnless(shutil.which('python2'),'Python 2.7 required for compatibility integration')
class IntegrationTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory()
        self.path=Path(self.tmp.name)
        with socket.socket() as s:
            s.bind(('127.0.0.1',0))
            self.port=s.getsockname()[1]
        self.log=(self.path/'server.txt').open('w')
        self.proc=subprocess.Popen(['python2',str(ROOT/'capture/robot/zeus_capture_server.py'),
                   '--simulate','--port',str(self.port),'--log-dir',str(self.path/'logs')],
                   stdout=self.log,stderr=subprocess.STDOUT)
        self.session=None
        deadline=time.monotonic()+5
        while time.monotonic()<deadline:
            if self.proc.poll() is not None:
                self.fail((self.path/'server.txt').read_text())
            try:
                with socket.create_connection(('127.0.0.1',self.port),timeout=.1):
                    break
            except OSError:
                time.sleep(.05)
        else:
            self.fail('simulation startup timeout')

    def tearDown(self):
        self.proc.send_signal(signal.SIGINT)
        try:
            self.proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            self.proc.kill()
            self.proc.wait()
        self.log.close()
        self.tmp.cleanup()

    def rpc(self,command,**extra):
        with socket.create_connection(('127.0.0.1',self.port),timeout=2) as s:
            s.sendall((json.dumps(dict(command=command,session=self.session,**extra))+'\n').encode())
            with s.makefile('rb') as f:
                r=json.loads(f.readline())
        return r

    def job(self,command,seq,**kw):
        r=self.rpc(command,seq=seq,**kw)
        self.assertTrue(r['ok'],r)
        jid=r['result']['job_id']
        for unused in range(100):
            r=self.rpc('job',job_id=jid)['result']
            if r['done']:
                self.assertTrue(r['ok'],r)
                return r
            time.sleep(.02)
        self.fail('job timeout')

    def connect_arm(self):
        self.session=self.rpc('connect')['result']['session']
        self.job('anchor',1)
        self.job('arm',2)

    def test_pc_camera_capture_and_roundtrip_sweep(self):
        output=self.path/'pc.txt'
        with output.open('w') as f:
            r=subprocess.run([sys.executable,str(ROOT/'capture/assisted_capture.py'),
                  '--robot-port',str(self.port),'--simulate-cameras','--no-preview',
                  '--commands',str(ROOT/'capture/assisted_simulation_commands.txt'),
                  '--dataset-root',str(self.path/'data')],stdout=f,stderr=subprocess.STDOUT,timeout=60)
        self.assertEqual(r.returncode,0,output.read_text())
        meta_path=next((self.path/'data').glob('*/meta.json'))
        meta=json.loads(meta_path.read_text())
        self.assertEqual(len(meta['captures']),14)
        for c in meta['captures']:
            self.assertTrue(c['capture_gate']['pass'],c['capture_gate'])
            self.assertFalse(c['eligible_for_calibration'])
            self.assertEqual(set(c['cams']),{'0','1'})
            for camera in c['cams'].values():
                self.assertTrue((meta_path.parent/camera['rgb_path']).is_file())
                self.assertEqual(camera['charuco']['n_corners'],24)
        self.assertEqual(json.loads((meta_path.parent/'valid_meta.json').read_text())['captures'],[])

    def test_lost_heartbeat_latches_fault(self):
        self.connect_arm()
        time.sleep(2.8)
        s=self.rpc('state')['result']
        self.assertFalse(s['armed'])
        self.assertEqual(s['fault']['reason'],'heartbeat_timeout')
        self.assertFalse(self.rpc('move',seq=3,target=[400,0,300,0,20,180])['ok'])

    def test_auto_multiframe_pair_collection_and_disarm(self):
        commands=self.path/'auto_commands.txt'
        commands.write_text('anchor\narm\nauto 3\nstate\n')
        output=self.path/'auto_pc.txt'
        with output.open('w') as f:
            r=subprocess.run([sys.executable,str(ROOT/'capture/assisted_capture.py'),
                  '--robot-port',str(self.port),'--simulate-cameras','--no-preview',
                  '--cameras','cam1','cam3','--auto-max-candidates','2',
                  '--commands',str(commands),'--dataset-root',str(self.path/'auto_data')],
                  stdout=f,stderr=subprocess.STDOUT,timeout=60)
        self.assertEqual(r.returncode,0,output.read_text())
        folder=next((self.path/'auto_data').iterdir())
        report=json.loads(next(folder.glob('auto_report_*')).read_text())
        self.assertEqual(report['status'],'incomplete')
        self.assertEqual(report['evaluated'],2)
        meta=json.loads((folder/'meta.json').read_text())
        self.assertEqual(len(meta['captures']),10)
        for row in meta['captures']:
            self.assertEqual(row['auto_validation']['joint_passes'],5)
            self.assertEqual(set(row['cams']),{'1','3'})
        self.assertEqual(json.loads(next(folder.glob('auto_valid_meta_*')).read_text())['captures'],[])
        self.session=self.rpc('connect')['result']['session']
        self.assertFalse(self.rpc('state')['result']['armed'])

    def test_duplicate_motion_is_rejected_and_stop_runs_out_of_band(self):
        self.connect_arm()
        target=[402,0,300,0,20,180]
        r=self.rpc('move',seq=3,target=target)
        self.assertTrue(r['ok'])
        self.assertFalse(self.rpc('move',seq=3,target=target)['ok'])
        deadline=time.monotonic()+1
        while not self.rpc('state')['result']['moving'] and time.monotonic()<deadline:
            time.sleep(.01)
        stop=self.rpc('stop')
        self.assertTrue(stop['ok'])
        time.sleep(.25)
        s=self.rpc('state')['result']
        self.assertEqual(s['fault']['reason'],'operator_stop')
        self.assertLess(s['state']['flange_pose_6dof'][0],402)
        self.assertFalse(s['armed'])


if __name__=='__main__':
    unittest.main()
