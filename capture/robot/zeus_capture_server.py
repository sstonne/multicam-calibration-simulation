#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""ZEUS Python 2.7/3 executor. Default read-only; bind localhost + SSH tunnel.

All i611Robot calls run in the main thread. Network/watchdog threads never call
that object. A separate RobSys process requests stop while line() is blocked.
No robot model limits or collision geometry are inferred from a name/reach.
"""
from __future__ import print_function, division
import argparse
import json
import math
import os
import socket
import subprocess
import sys
import threading
import time
import traceback
import uuid
try:
    import socketserver
    import queue
except ImportError:
    import SocketServer as socketserver
    import Queue as queue

from zeus_capture_core import Guard, angle, clock, distance, vector

PORT_NAMES = ('running','servo_on','emergency','hw_error','sw_error',
              'abs_lost','paused','error')


def sdk_ok(result, name):
    # Some SDK setters/join return None on success; motion returns [True, ...].
    if result is False or (isinstance(result,(list,tuple)) and result and result[0] is False):
        raise RuntimeError('%s returned failure %r' % (name,result))
    return result


def json_number(value):
    number=float(value)
    if math.isnan(number) or math.isinf(number):
        raise ValueError('Non-finite JSON number')
    return number


class Backend(object):
    def __init__(self, mode, event):
        self.mode, self.event, self.rb = mode, event, None
        self.pose = [400.,0.,300.,0.,20.,180.]
        self.origin = list(self.pose)
        self.q0 = [0.,20.,40.,0.,20.,0.]
        if mode != 'simulate':
            from i611shm import shm_read
            self.shm = shm_read
        if mode == 'live':
            try:
                self.open_live()
            except BaseException:
                if self.rb:
                    try:
                        self.rb.close()
                    except BaseException:
                        pass
                raise

    def open_live(self):
            from i611_MCS import i611Robot, MotionParam
            self.rb = i611Robot()
            sdk_ok(self.rb.open(), 'open')
            sdk_ok(self.rb.enable_interrupt(0, True), 'enable stop interrupt')
            sdk_ok(self.rb.settool(1, 0.,0.,0.,0.,0.,0.), 'settool')
            sdk_ok(self.rb.changetool(1), 'changetool')
            sdk_ok(self.rb.override(10.), 'override')
            # Explicit non-blended synchronous moves; no waypoint prefetch queue.
            sdk_ok(self.rb.asyncm(2), 'asyncm off')
            sdk_ok(self.rb.motionparam(MotionParam(lin_speed=5., jnt_speed=5.,
                    pose_speed=5., acctime=1., dacctime=1., overlap=0., passm=2)),
                   'motionparam')
            for name in ('Position2Joint','getpos','line','join'):
                if not callable(getattr(self.rb,name,None)):
                    raise RuntimeError('Required SDK API unavailable: '+name)

    def read(self):
        if self.mode == 'simulate':
            pose = list(self.pose)
            joints = self.ik(pose)
            flags = dict(zip(PORT_NAMES, [1,1,0,0,0,0,0,0]))
            status, err = 6, 0
        else:
            v = vector(self.shm(0x3000,6).split(','))
            pose = [x*1000. for x in v[:3]]+[math.degrees(x) for x in v[3:]]
            joints = [math.degrees(x) for x in vector(self.shm(0x3050,6).split(','))]
            hw, sw = int(self.shm(0x0300,1)), int(self.shm(0x0308,1))
            flags = dict(zip(PORT_NAMES, [sw&1,hw&1,(hw>>1)&1,(hw>>2)&1,
                    (sw>>1)&1,(hw>>3)&1,(sw>>2)&1,(sw>>3)&1]))
            status, err = (sw>>4)&15, (sw>>8)&255
        return dict(flange_pose_6dof=pose, joints_6dof=joints,
                    system_status=status, error_id=err, flags=flags,
                    tool=1 if self.mode!='readonly' else None,
                    tool_offset_6dof=[0.]*6 if self.mode!='readonly' else None,
                    tool_verified=self.mode!='readonly',
                    pose_reference='flange' if self.mode!='readonly' else 'active_tcp_unknown',
                    simulation=self.mode=='simulate', sampled_at=time.time())

    def prepare(self):
        if self.rb:
            self.template = self.rb.getpos().copy()

    def position(self, pose):
        # Preserve current posture and multiturn, unlike a fresh Position(...).
        return self.template.copy().replace(**dict(zip(('x','y','z','rz','ry','rx'),pose)))

    def ik(self, pose):
        if self.mode == 'simulate':
            return [self.q0[i]+(pose[i]-self.origin[i])*(0.1 if i<3 else 1.) for i in range(6)]
        answer = self.rb.Position2Joint(self.position(pose))
        if not hasattr(answer,'jnt2list'):
            raise ValueError('IK unavailable for candidate: %r' % (answer,))
        return vector(answer.jnt2list()[:6])

    def move(self, pose):
        if self.event.is_set():
            raise RuntimeError('Motion inhibited by stop request')
        if self.mode == 'simulate':
            for unused in range(4):
                if self.event.is_set():
                    raise RuntimeError('Stop requested')
                time.sleep(0.025)
            self.pose = list(pose)
        elif self.mode == 'live':
            result = self.rb.line(self.position(pose))
            if not isinstance(result,(list,tuple)) or not result or result[0] is not True:
                raise RuntimeError('line did not confirm acceptance: %r' % (result,))
            sdk_ok(self.rb.join(),'join')
        else:
            raise ValueError('Read-only server cannot move')

    def close(self):
        if self.rb:
            self.rb.close()


class Executor(object):
    def __init__(self, mode, log_path, helper=None, joint_limits=None):
        self.mode, self.helper = mode, helper
        self.lock, self.log_lock = threading.RLock(), threading.Lock()
        self.stop_event, self.shutdown = threading.Event(), threading.Event()
        self.tasks = queue.Queue(maxsize=1)
        self.guard = Guard(joint_limits)
        self.session, self.last_seq, self.last_seen = None, 0, 0.
        self.armed, self.pending, self.moving = False, False, False
        self.fault, self.anchor, self.snapshot = None, None, None
        self.sample_clock, self.job_started = 0., 0.
        self.jobs, self.job_counter = {}, 0
        self.stop_thread = None
        self.log_file = open(log_path,'a')
        self.log('start', mode=mode, limits=self.guard.limits)
        self.backend = Backend(mode,self.stop_event)
        self.update()

    def log(self, kind, **data):
        data.update(event=kind, time=time.time())
        with self.log_lock:
            if self.log_file.closed:
                return
            self.log_file.write(json.dumps(data,ensure_ascii=True,allow_nan=False)+'\n')
            self.log_file.flush()

    def update(self):
        value = self.backend.read()
        with self.lock:
            self.snapshot, self.sample_clock = value, clock()
        return value

    def healthy(self, state):
        f = state['flags']
        bad = [k for k in PORT_NAMES[2:] if f[k]]
        if not f['servo_on'] or bad or state['system_status'] not in (2,6):
            raise RuntimeError('Controller not ready: %s / status=%s error=%s' %
                               (bad,state['system_status'],state['error_id']))

    def stop(self, reason):
        with self.lock:
            first = self.fault is None
            active = self.armed or self.moving
            if first:
                self.fault = dict(reason=reason, time=time.time(), state=self.snapshot)
            self.armed = False
            self.stop_event.set()
        if first:
            if self.mode=='live' and active:
                self.start_stop_thread()
            try:
                self.log('fault', fault=self.fault)
            except Exception as error:
                # A full/unwritable disk must never prevent the stop request.
                sys.stderr.write('Fault log failed: %r\n' % (error,))

    def start_stop_thread(self):
        t = threading.Thread(target=self.external_stop)
        t.daemon = True
        self.stop_thread = t
        t.start()

    def external_stop(self):
        try:
            p = subprocess.Popen([sys.executable,self.helper,'stop'],
                                 stdout=subprocess.PIPE,stderr=subprocess.STDOUT)
            out = p.communicate()[0].decode('utf-8','replace')
            self.log('controller_stop_request',returncode=p.returncode,output=out,
                     actual_stop_confirmed=False)
        except Exception as error:
            self.log('controller_stop_request_failed',detail=repr(error))

    def watchdog(self):
        while not self.shutdown.wait(0.1):
            with self.lock:
                expired = self.session and clock()-self.last_seen>2.5
                active = self.armed or self.pending or self.moving
                overdue = self.pending and clock()-self.job_started>15.
            if overdue:
                self.stop('operation_timeout')
            elif expired and active:
                self.stop('heartbeat_timeout')

    def state(self):
        with self.lock:
            return dict(state=self.snapshot, age_s=clock()-self.sample_clock,
                        armed=self.armed, moving=self.moving, pending=self.pending,
                        fault=self.fault, anchor=self.anchor, mode=self.mode,
                        limits=self.guard.limits, absolute_joint_limits_checked=self.guard.joint_limits is not None,
                        configured_joint_limits=self.guard.joint_limits,
                        collision_check=False)

    def rpc(self, req):
        if not isinstance(req,dict):
            raise ValueError('JSON object required')
        command = req.get('command')
        if command=='stop':
            self.stop('operator_stop')
            return {'stop_requested':True,'actual_stop_confirmed':False}
        with self.lock:
            if command=='connect':
                if self.session and clock()-self.last_seen<=2.5:
                    raise ValueError('Another PC owns the session')
                if self.pending:
                    raise ValueError('Operation still pending')
                self.session, self.last_seq = uuid.uuid4().hex, 0
                self.last_seen = clock()
                self.armed = False
                return dict(session=self.session, **self.state())
            if req.get('session')!=self.session or self.session is None:
                raise ValueError('Invalid session')
            if command=='heartbeat':
                self.last_seen=clock()
                return {'alive':True}
            if command=='state':
                return self.state()
            if command=='job':
                job = self.jobs.get(str(req.get('job_id')))
                if job is None:
                    raise ValueError('Unknown job')
                return job
            if command=='disconnect':
                if self.pending or self.moving:
                    self.stop('disconnect_during_operation')
                self.armed=False
                self.session=None
                return {'disconnected':True}
            if command not in ('anchor','arm','disarm','move'):
                raise ValueError('Unknown command')
            seq=req.get('seq')
            if isinstance(seq,bool) or not isinstance(seq,int) or seq<=self.last_seq:
                raise ValueError('Sequence already used or invalid; commands are never replayed')
            self.last_seq=seq
            if self.pending:
                raise ValueError('Busy; no motion queue is accepted')
            if command!='disarm' and self.fault:
                raise ValueError('FAULT latched; inspect logs and controller before restarting server')
            if clock()-self.last_seen>2.5:
                raise ValueError('Heartbeat expired')
            self.job_counter+=1
            jid=str(self.job_counter)
            self.jobs[jid]={'job_id':jid,'done':False}
            self.pending=True
            self.job_started=clock()
            self.tasks.put_nowait((jid,dict(req),clock()))
            return {'job_id':jid}

    def execute(self, req):
        command=req['command']
        if command=='disarm':
            self.armed=False
            return self.state()
        if self.mode=='readonly':
            raise ValueError('Read-only mode: restart with --enable-motion for control')
        s=self.update()
        self.healthy(s)
        if command=='anchor':
            if self.armed:
                raise ValueError('Disarm before teaching a new anchor')
            self.anchor={'pose':s['flange_pose_6dof'],'joints':s['joints_6dof']}
            self.log('anchor',anchor=self.anchor)
            return self.state()
        if self.anchor is None:
            raise ValueError('Teach anchor first')
        if command=='arm':
            self.guard.path(s['flange_pose_6dof'],s['flange_pose_6dof'],self.anchor['pose'])
            self.guard.check_joints([s['joints_6dof']],s['joints_6dof'],self.anchor['joints'])
            self.armed=True
            return self.state()
        if not self.armed:
            raise ValueError('Arm first')
        target=vector(req.get('target'))
        path=self.guard.path(s['flange_pose_6dof'],target,self.anchor['pose'])
        self.backend.prepare()
        qpath=[self.backend.ik(p) for p in path]
        self.guard.check_joints(qpath,s['joints_6dof'],self.anchor['joints'])
        # Detect external motion during IK planning before issuing anything.
        fresh=self.update()
        if distance(s['flange_pose_6dof'],fresh['flange_pose_6dof'])>0.2 or \
                max(abs(a-b) for a,b in zip(s['joints_6dof'],fresh['joints_6dof']))>0.2:
            raise RuntimeError('Robot moved during planning')
        self.log('move_start',target=target,before=s,ik_path=qpath,
                 parameters={'lin_speed_mm_s':5,'pose_speed_percent':5,'override_percent':10,
                             'acc_s':1,'overlap_mm':0})
        for index,p in enumerate(path[1:],1):
            with self.lock:
                if self.stop_event.is_set() or not self.armed:
                    raise RuntimeError('Motion inhibited')
                self.moving=True
            self.healthy(self.update())
            self.backend.move(p)
            after=self.update()
            self.healthy(after)
            if distance(p,after['flange_pose_6dof'])>0.5 or angle(p,after['flange_pose_6dof'])>0.5:
                raise RuntimeError('Actual pose differs from segment target')
            if max(abs(a-b) for a,b in zip(qpath[index],after['joints_6dof']))>0.5:
                raise RuntimeError('Actual IK branch differs from planned branch')
        if self.stop_event.is_set():
            raise RuntimeError('Stop requested during move')
        self.log('move_complete',target=target,after=self.snapshot)
        return self.state()

    def run(self):
        while not self.shutdown.is_set():
            try:
                jid,req,created=self.tasks.get(timeout=0.1)
            except queue.Empty:
                try:
                    s=self.update()
                    if self.armed:
                        self.healthy(s)
                except Exception as error:
                    if self.armed:
                        self.stop('state_error: '+repr(error))
                continue
            try:
                if req.get('session')!=self.session or clock()-created>1.:
                    raise ValueError('Expired request')
                if self.fault and req['command']!='disarm':
                    raise ValueError('FAULT latched')
                result=self.execute(req)
                job={'job_id':jid,'done':True,'ok':True,'result':result}
            except ValueError as error:
                # A ValueError from an SDK after movement began is a fault too.
                rejected = not self.moving
                if not rejected:
                    self.stop('motion_error: '+repr(error))
                job={'job_id':jid,'done':True,'ok':False,'detail':repr(error),'rejected':rejected}
                self.log('command_rejected' if rejected else 'motion_error',request=req,detail=repr(error))
            except Exception as error:
                self.stop(type(error).__name__+': '+repr(error))
                job={'job_id':jid,'done':True,'ok':False,'detail':repr(error),'rejected':False}
                self.log('exception',request=req,traceback=traceback.format_exc())
            finally:
                with self.lock:
                    self.pending=False
                    self.moving=False
            with self.lock:
                self.jobs[jid]=job
                if len(self.jobs)>100:
                    del self.jobs[min(self.jobs,key=int)]


class Server(socketserver.ThreadingMixIn,socketserver.TCPServer):
    allow_reuse_address=True
    daemon_threads=True


class Handler(socketserver.StreamRequestHandler):
    def handle(self):
        self.request.settimeout(2.)
        try:
            raw=self.rfile.readline(65537)
            if not raw.endswith(b'\n') or len(raw)>65536:
                raise ValueError('Request too long or incomplete')
            result=self.server.executor.rpc(json.loads(raw.decode('utf-8'),
                                                       parse_float=json_number,parse_constant=json_number))
            response={'ok':True,'result':result}
        except Exception as error:
            response={'ok':False,'detail':str(error)}
        try:
            self.wfile.write((json.dumps(response,allow_nan=False)+'\n').encode('utf-8'))
        except (IOError,socket.error):
            pass


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    mode=ap.add_mutually_exclusive_group()
    mode.add_argument('--simulate',action='store_true')
    mode.add_argument('--enable-motion',action='store_true')
    ap.add_argument('--port',type=int,default=12351)
    ap.add_argument('--sdk-dir',help='Directory containing the actually installed i611_MCS.py')
    ap.add_argument('--check-sdk',action='store_true',help='Inspect APIs and stop transport without rb.open()')
    ap.add_argument('--joint-limits',help='JSON verified min_deg/max_deg/margin_deg; never inferred from model name')
    ap.add_argument('--log-dir',default=os.path.join(os.path.dirname(os.path.abspath(__file__)),'logs'))
    args=ap.parse_args()
    if args.sdk_dir:
        sdk_dir=os.path.abspath(os.path.expanduser(args.sdk_dir))
        sys.path.insert(1,sdk_dir)
        os.environ['ZEUS_CAPTURE_SDK_DIR']=sdk_dir
    helper=os.path.join(os.path.dirname(os.path.abspath(__file__)),'zeus_stop_helper.py')
    if args.check_sdk:
        import i611_MCS
        required=('open','close','getpos','line','join','Position2Joint','asyncm',
                  'motionparam','override','settool','changetool','enable_interrupt')
        missing=[n for n in required if not callable(getattr(i611_MCS.i611Robot,n,None))]
        print(json.dumps({'sdk_file':i611_MCS.__file__,'missing_apis':missing},indent=2))
        if missing:
            raise SystemExit(2)
        subprocess.check_call([sys.executable,helper,'probe'])
        print(json.dumps(Backend('readonly',threading.Event()).read(),indent=2))
        return
    mode='simulate' if args.simulate else ('live' if args.enable_motion else 'readonly')
    limits=None
    if args.joint_limits:
        with open(args.joint_limits) as f:
            limits=json.load(f)
        Guard(limits)
    if not os.path.isdir(args.log_dir):
        os.makedirs(args.log_dir)
    path=os.path.join(args.log_dir,'executor_'+time.strftime('%Y%m%d_%H%M%S')+'_'+uuid.uuid4().hex[:6]+'.jsonl')
    # Fail before opening the controller if independent stop transport cannot connect.
    if mode=='live':
        subprocess.check_call([sys.executable,helper,'probe'])
    server=Server(('127.0.0.1',args.port),Handler)
    executor=None
    try:
        executor=Executor(mode,path,helper,limits)
        server.executor=executor
        network=threading.Thread(target=server.serve_forever)
        watchdog=threading.Thread(target=executor.watchdog)
        network.daemon=watchdog.daemon=True
        network.start()
        watchdog.start()
        print('ZEUS %s server: 127.0.0.1:%d; log=%s' % (mode,args.port,path))
        sys.stdout.flush()
        executor.run()
    except KeyboardInterrupt:
        print('Shutdown requested')
    finally:
        if executor:
            executor.stop('server_shutdown')
            executor.shutdown.set()
            if executor.stop_thread:
                executor.stop_thread.join(5.)
            executor.backend.close()
            executor.log_file.close()
        server.server_close()


if __name__=='__main__':
    main()
