#!/usr/bin/env python3
"""PC keyboard/numeric control and bounded capture. See docs/zeus_assisted_capture.md."""
from __future__ import annotations
import argparse
import json
import os
from pathlib import Path
import queue
import shlex
import socket
import sys
import threading
import time
import uuid

HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE))
from robot.zeus_capture_core import angle, distance, offset_pose, quality_plan, vector


def atomic_json(path,data):
    # Validate before opening the temporary file, preserving earlier snapshots.
    payload=json.dumps(data,ensure_ascii=False,indent=2,allow_nan=False)
    path=Path(path)
    temporary=path.with_suffix(path.suffix+'.tmp')
    with temporary.open('w',encoding='utf-8') as f:
        f.write(payload)
        f.flush()
        os.fsync(f.fileno())
    temporary.replace(path)


class RobotLink:
    def __init__(self,host,port):
        self.address=(host,port)
        self.session=None
        self.seq=0
        self.done=threading.Event()
        self.broken=threading.Event()
        hello=self.raw('connect')
        self.session=hello['session']
        self.mode=hello['mode']
        self.thread=threading.Thread(target=self.heartbeat,daemon=True)
        self.thread.start()

    def raw(self,command,**payload):
        payload.update(command=command,session=self.session)
        with socket.create_connection(self.address,timeout=1.5) as s:
            s.settimeout(1.5)
            s.sendall((json.dumps(payload,allow_nan=False)+'\n').encode())
            with s.makefile('rb') as f:
                line=f.readline(1048577)
        if len(line)>1048576 or not line.endswith(b'\n'):
            raise RuntimeError('Incomplete/oversized robot response')
        r=json.loads(line)
        if not r.get('ok'):
            # A complete negative reply is a rejection, not a lost connection.
            raise ValueError(r.get('detail','Robot request failed'))
        return r['result']

    def heartbeat(self):
        while not self.done.wait(.4):
            try:
                self.raw('heartbeat')
            except Exception as error:
                self.broken.set()
                print('[통신 오류] 후속 작업 중단:',error,flush=True)
                self.stop()
                return

    def state(self):
        started=time.monotonic()
        result=self.raw('state')
        if result['age_s']+time.monotonic()-started>.75:
            raise RuntimeError('Robot state stale; capture/move not allowed')
        return result

    def operate(self,command,**payload):
        if self.broken.is_set():
            raise RuntimeError('Connection lost; restart PC session after inspection')
        self.seq+=1
        try:
            job=self.raw(command,seq=self.seq,**payload)
            deadline=time.monotonic()+18.
            while time.monotonic()<deadline:
                result=self.raw('job',job_id=job['job_id'])
                if result['done']:
                    if not result['ok']:
                        raise ValueError(result['detail'])
                    return result['result']
                if self.broken.is_set():
                    raise RuntimeError('Heartbeat failed')
                time.sleep(.08)
            raise TimeoutError('Robot operation timed out; not resending command')
        except ValueError:
            raise
        except Exception:
            # Receipt may have been lost after execution: never retry a motion.
            self.broken.set()
            self.stop()
            raise

    def stop(self):
        try:
            return self.raw('stop')
        except Exception as error:
            print('[정지 요청 전달 실패]',error,flush=True)
            return None

    def close(self):
        try:
            self.raw('disconnect')
        except Exception:
            self.stop()
        self.done.set()
        self.thread.join(2.)


def state_compatible(a,b,mm=.15,deg=.15):
    return (distance(a['flange_pose_6dof'],b['flange_pose_6dof'])<=mm and
            angle(a['flange_pose_6dof'],b['flange_pose_6dof'])<=deg and
            max(abs(x-y) for x,y in zip(a['joints_6dof'],b['joints_6dof']))<=deg)


class CaptureSession:
    def __init__(self,args,robot,rig,cancel):
        from board_config import ROBOT_BOARD
        from shah_capture_client import BoardDetector, build_session_header
        self.args,self.robot,self.rig,self.cancel=args,robot,rig,cancel
        self.detector=BoardDetector(ROBOT_BOARD,0,args.detection_scale)
        self.path=Path(args.dataset_root)/('session_'+time.strftime('%Y%m%d_%H%M%S')+'_'+uuid.uuid4().hex[:6])
        self.path.mkdir(parents=True,exist_ok=False)
        cameras=[] if rig is None else rig.cameras
        self.meta=build_session_header(args,cameras,self.detector,self.path)
        self.meta['capture_config']['detection_scale']=args.detection_scale
        self.meta['capture_config']['corner_coordinate_space']='original_image_pixels'
        self.meta.update(collector='assisted_capture_v1',simulation=robot.mode=='simulate',
                         camera_pair=list(args.cameras),
                         robot_model='ZEUS ZRA-0515P',operator_events=[],
                         timestamp_note='Frame receipt times; no hardware camera/robot synchronization',
                         pose_reference_verified=False)
        self.meta['board_config']['note']='User confirmed: 7 x 5 squares, 17 mm, markers 12 mm, first ID 0.'
        for c in cameras:
            (self.path/('cam'+str(c.index))).mkdir()
        self.target=None
        self.flush()
        print('저장 위치:',self.path,flush=True)

    def flush(self):
        atomic_json(self.path/'meta.json',self.meta)
        valid=dict(self.meta)
        valid['captures']=[r for r in self.meta['captures'] if r['eligible_for_calibration']]
        valid['selection_note']='Only quality + robot stability passed, real camera captures'
        atomic_json(self.path/'valid_meta.json',valid)

    def event(self,kind,**data):
        self.meta['operator_events'].append(dict(event=kind,time=time.time(),**data))
        self.flush()

    def stable(self):
        samples=[]
        started=time.monotonic()
        while time.monotonic()-started<12.:
            if self.cancel.is_set():
                raise InterruptedError('Operation cancelled')
            status=self.robot.state()
            if status['fault']:
                raise RuntimeError('Robot FAULT: '+str(status['fault']['reason']))
            s=status['state']
            bad=any(s['flags'][k] for k in ('emergency','hw_error','sw_error','abs_lost','paused','error'))
            if bad:
                raise RuntimeError('Controller not ready for capture: '+str(s['flags']))
            if status['moving'] or status['pending']:
                samples=[]
            elif not samples or s['sampled_at']!=samples[-1][1]['sampled_at']:
                if samples and not all(state_compatible(x[1],s) for x in samples):
                    samples=[]
                samples.append((time.monotonic(),s))
                if len(samples)>=5 and samples[-1][0]-samples[0][0]>=self.args.settle_time_s:
                    return s
            time.sleep(.1)
        raise TimeoutError('Robot did not remain stable')

    def capture(self,label='manual',evaluation_only=False):
        import cv2
        from assisted_camera import finite_quality
        from shah_capture_client import zeus_pose_to_matrix, CAPTURE_BLOCK
        before=self.stable()
        start=time.monotonic()
        event=len(self.meta['captures'])
        pose=before['flange_pose_6dof']
        matrix=zeus_pose_to_matrix(pose).tolist()
        record=dict(event_id=event,capture_index=event,capture_block=CAPTURE_BLOCK,
                    label=label,robot_pose_6dof=pose,robot_pose_matrix_4x4=matrix,
                    capture_gripper_pose_6dof=pose,capture_gripper_pose_matrix_4x4=matrix,
                    capture_robot_joints_6dof=before['joints_6dof'],tool=before['tool'],
                    requested_target=self.target,robot_before=before,cams={})
        reasons=[]
        items={} if self.rig is None else self.rig.fresh(start)
        receipts=[]
        for camera in ([] if self.rig is None else self.rig.cameras):
            key=str(camera.index)
            sample=items[camera.name]
            if sample is None:
                record['cams'][key]=dict(saved=False,is_gripper=False,
                        gate_reason='fresh_frame_unavailable',skip_reason='fresh_frame_unavailable')
                reasons.append(camera.name+':fresh_frame_unavailable')
                continue
            f,q=sample['frame'],finite_quality(sample['quality'])
            rel='cam%s/rgb_%05d.png' % (key,event)
            try:
                if not cv2.imwrite(str(self.path/rel),f['color']):
                    raise OSError('image write returned false')
                depth=None
                if f['depth'] is not None:
                    depth='cam%s/depth_%05d.png' % (key,event)
                    if not cv2.imwrite(str(self.path/depth),f['depth']):
                        raise OSError('depth write returned false')
                record['cams'][key]=dict(saved=True,is_gripper=False,rgb_path=rel,depth_path=depth,
                    ts_ms=f['ts_ms'],host_monotonic_ts_ms=f['host_monotonic_ts_ms'],
                    device_ts_ms=f['device_ts_ms'],frame_sequence=sample['sequence'],
                    charuco_detect_n=q['charuco']['n_corners'],**q)
                receipts.append(f['host_monotonic_ts_ms'])
                if q['gate_reason']:
                    reasons.append(camera.name+':'+q['gate_reason'])
            except Exception as error:
                record['cams'][key]=dict(saved=False,is_gripper=False,rgb_path=rel,
                                         gate_reason='storage_failed: '+str(error))
                reasons.append(camera.name+':storage_failed')
        try:
            end_status=self.robot.state()
            after=end_status['state']
            record['robot_after']=after
            robot_ok=(not end_status['fault'] and not end_status['moving'] and not end_status['pending']
                      and state_compatible(before,after) and before['tool_verified'] and after['tool_verified']
                      and not any(after['flags'][k] for k in ('emergency','hw_error','sw_error','abs_lost','paused','error')))
        except Exception as error:
            record['robot_after']=None
            record['robot_after_error']=str(error)
            robot_ok=False
        if not robot_ok:
            reasons.append('robot_state_or_stability_unverified')
        if len(receipts)>1 and max(receipts)-min(receipts)>self.args.max_camera_span_ms:
            reasons.append('camera_receipt_span_exceeded')
        if self.rig is None:
            reasons.append('no_camera')
        if self.cancel.is_set():
            reasons.append('operator_cancelled')
        passed=sum(bool(c.get('saved')) and c.get('gate_reason') is None for c in record['cams'].values())
        record['capture_gate']=dict(**{'pass':not reasons},n_cams_passed=passed,
                n_cams_required=len(record['cams']),reason='; '.join(reasons) or 'all passed')
        record['capture_span_ms']=(time.monotonic()-start)*1000.
        record['camera_receipt_span_ms']=max(receipts)-min(receipts) if receipts else None
        record['evaluation_only']=evaluation_only
        record['eligible_for_calibration']=not reasons and self.robot.mode!='simulate' and not evaluation_only
        record['quality_score']=min((c.get('score',0.) for c in record['cams'].values()),default=0.)
        self.meta['captures'].append(record)
        self.meta['pose_reference_verified']=before['tool_verified']
        self.flush()
        print('[%d] %s score=%.3f %s' % (event,label,record['quality_score'],record['capture_gate']['reason']),flush=True)
        return record


HELP='''
state                  실제 자세·관절·오류 상태
anchor                 현재 자세를 기준으로 저장 (disarm 상태)
arm / disarm           이 세션의 이동 허용 / 해제
jog x 1                현재 자세에서 BASE X +1 mm (y,z,rx,ry,rz 가능)
jog ry -0.5            현재 자세에서 BASE Y축 -0.5 deg 회전
goto x y z rz ry rx     절대 목표 (한 번에 2 mm / 1 deg 이내만 허용)
capture [이름]          안정 상태 확인 후 촬영; Enter도 촬영
plan [mm deg]          기준 주변 12개 후보 + 각 후보 뒤 기준 복귀 (기본 1, 0.5)
next                   계획 한 항목 실행; 촬영 후보이면 촬영
run                    계획 연속 실행 (먼저 next로 실제 구간 검증)
suggest                촬영 기록 중 두 카메라 품질이 가장 좋았던 자세 표시
auto [개수]            두 카메라 동시 유효 위치 탐색 + 다양한 자세 수집 (기본 30)
pause                  진행 중 이동 완료 후 계획 중단
stop                   즉시 소프트웨어 정지 요청 + FAULT 유지
quit                   종료
미리보기 창: Space 촬영 / X 또는 Esc 정지 요청
'''


class Operator:
    def __init__(self,robot,session,cancel):
        self.robot,self.session,self.cancel=robot,session,cancel
        self.plan=[]
        self.index=0

    def move(self,target):
        self.session.target=vector(target)
        self.robot.operate('move',target=self.session.target)

    def step(self):
        if self.cancel.is_set():
            raise InterruptedError('Plan paused')
        if self.index>=len(self.plan):
            print('계획 완료 또는 plan 필요',flush=True)
            return False
        p=self.plan[self.index]
        print('계획 %d/%d %s' % (self.index+1,len(self.plan),p['label']),flush=True)
        self.move(p['target'])
        self.index+=1
        if p['capture']:
            record=self.session.capture(p['label'])
            cameras=record['cams'].values()
            if not cameras or all(c.get('charuco',{}).get('n_corners',0)==0 for c in cameras):
                raise InterruptedError('Both cameras lost board; inspect and realign. No automatic return.')
            if not record['robot_after'] or not state_compatible(record['robot_before'],record['robot_after']):
                raise InterruptedError('Robot capture state invalid; inspect before further moves')
        return True

    def command(self,line):
        words=shlex.split(line)
        command=words[0].lower() if words else 'capture'
        args=words[1:]
        if command=='state':
            print(json.dumps(self.robot.state(),indent=2,ensure_ascii=False),flush=True)
        elif command in ('anchor','arm','disarm'):
            result=self.robot.operate(command)
            if command=='anchor':
                self.plan=[]
                self.index=0
                self.session.event('anchor',anchor=result['anchor'])
                self.session.meta['robot_motion_configuration']={k:result[k] for k in
                    ('limits','absolute_joint_limits_checked','configured_joint_limits','collision_check','mode')}
                self.session.flush()
            print(command+': 완료',flush=True)
        elif command=='jog':
            if len(args)!=2 or args[0] not in ('x','y','z','rz','ry','rx'):
                raise ValueError('사용법: jog x 1 / jog ry -0.5')
            d=[0.]*6
            d[('x','y','z','rz','ry','rx').index(args[0])]=float(args[1])
            current=self.robot.state()['state']['flange_pose_6dof']
            self.move(offset_pose(current,d))
            print('jog: 완료',flush=True)
        elif command=='goto':
            self.move(vector(args))
        elif command=='capture':
            self.session.capture(' '.join(args) or 'manual')
        elif command=='plan':
            a=self.robot.state()['anchor']
            if not a:
                raise ValueError('anchor 먼저 실행')
            if args and len(args)!=2:
                raise ValueError('사용법: plan 또는 plan 1 0.5')
            self.plan=quality_plan(a['pose'],*(map(float,args) if args else (1.,.5)))
            self.index=0
            self.session.event('plan',items=self.plan)
            print(json.dumps(self.plan,indent=2),flush=True)
        elif command in ('next','run'):
            if not self.plan:
                raise ValueError('plan 먼저 실행')
            if command=='run':
                if self.session.rig is None:
                    raise ValueError('run requires camera feedback')
                baseline=self.session.capture('before_run')
                if any(c.get('charuco',{}).get('n_corners',0)<4 for c in baseline['cams'].values()):
                    raise ValueError('Reposition board: each camera needs at least 4 corners to start run')
                while not self.cancel.is_set() and self.index<len(self.plan):
                    if not self.step():
                        break
            else:
                self.step()
        elif command=='auto':
            from assisted_search import AutoCollector
            if len(args)>1:
                raise ValueError('사용법: auto 또는 auto 30')
            AutoCollector(self).run(int(args[0]) if args else 30)
        elif command=='suggest':
            good=[r for r in self.session.meta['captures'] if r['capture_gate']['pass']
                  and not r.get('evaluation_only',False)]
            if not good:
                print('품질 비교에 사용할 기록 없음',flush=True)
            else:
                best=max(good,key=lambda r:r['quality_score'])
                print('관측된 최고 품질 (자동 이동 아님):',best['label'],
                      best['robot_pose_6dof'],'score=',best['quality_score'],flush=True)
        elif command=='help':
            print(HELP,flush=True)
        else:
            raise ValueError('알 수 없는 명령. help 입력')


def parse_args(argv=None):
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--robot-host',default='127.0.0.1')
    p.add_argument('--robot-port',type=int,default=12351)
    p.add_argument('--cameras',nargs='+',default=['cam0','cam1'])
    p.add_argument('--intrinsics-dir',default=str(HERE.parent/'intrinsics'))
    p.add_argument('--dataset-root',default=str(HERE.parent/'datasets'/'assisted'))
    p.add_argument('--no-camera',action='store_true')
    p.add_argument('--simulate-cameras',action='store_true')
    p.add_argument('--no-preview',action='store_true')
    p.add_argument('--commands',type=Path,help='명령 파일: 시뮬레이션에서만 허용')
    p.add_argument('--width',type=int,default=1280)
    p.add_argument('--height',type=int,default=720)
    p.add_argument('--fps',type=int,default=15)
    p.add_argument('--save-depth',action='store_true')
    p.add_argument('--startup-stagger-s',type=float,default=.8)
    p.add_argument('--settle-time-s',type=float,default=.6)
    p.add_argument('--min-corners',type=int,default=12)
    p.add_argument('--detection-scale',type=int,choices=(1,2,3),default=3,
                   help='검출 전용 확대 배율; 저장 영상·코너 좌표·내부 파라미터는 원본 기준')
    p.add_argument('--max-reproj-px',type=float,default=1.5)
    p.add_argument('--min-margin-px',type=float,default=15.)
    p.add_argument('--min-sharpness',type=float,default=0.,help='장비별 검증 후 설정; 0이면 흐림 수치만 기록')
    p.add_argument('--max-camera-span-ms',type=float,default=250.)
    p.add_argument('--auto-max-candidates',type=int,default=120)
    p.add_argument('--auto-max-minutes',type=float,default=25.)
    p.add_argument('--auto-radius-mm',type=float,default=10.)
    p.add_argument('--auto-angle-deg',type=float,default=6.)
    args=p.parse_args(argv)
    if len(set(args.cameras))!=len(args.cameras) or any(n not in ('cam0','cam1','cam2','cam3') for n in args.cameras):
        p.error('카메라 이름은 중복 없는 cam0~cam3')
    if not args.no_camera and len(args.cameras)!=2:
        p.error('이 첫 버전은 카메라 2대를 명시해야 합니다')
    if not 4<=args.min_corners<=24 or not .5<=args.settle_time_s<=5.:
        p.error('min-corners 4..24, settle-time-s 0.5..5')
    import math
    if not all(math.isfinite(v) for v in (args.max_reproj_px,args.max_camera_span_ms,
                                         args.min_margin_px,args.min_sharpness,args.startup_stagger_s)) or \
            args.max_reproj_px<=0 or args.max_camera_span_ms<=0 or args.min_margin_px<0 or args.min_sharpness<0:
        p.error('잘못된 품질 기준')
    if not (1<=args.auto_max_candidates<=500 and 0<args.auto_max_minutes<=120
            and 0<args.auto_radius_mm<=20 and 0<args.auto_angle_deg<=10):
        p.error('auto: 후보 1..500, 시간 0..120분, 시작점 반경 0..20mm / 0..10deg')
    return args


def main(argv=None):
    args=parse_args(argv)
    robot=RobotLink(args.robot_host,args.robot_port)
    rig=None
    cancel=threading.Event()
    exit_event=threading.Event()
    requests=queue.Queue(maxsize=1)
    submit_lock=threading.Lock()
    busy=threading.Event()
    worker=None
    result_code=[0]
    try:
        if (args.commands or args.simulate_cameras) and robot.mode!='simulate':
            raise ValueError('명령 파일/가상 카메라는 시뮬레이션 로봇에서만 허용')
        if not args.no_camera:
            from assisted_camera import CameraRig
            rig=CameraRig(args)
        session=CaptureSession(args,robot,rig,cancel)
        operator=Operator(robot,session,cancel)
        print('서버 모드:',robot.mode,flush=True)
        print(HELP,flush=True)
        def execute():
            while not exit_event.is_set():
                try:
                    line=requests.get(timeout=.1)
                except queue.Empty:
                    continue
                try:
                    operator.command(line)
                except Exception as error:
                    print('[작업 중단]',error,flush=True)
                    try:
                        session.event('operation_error',command=line,detail=str(error))
                    except Exception as storage_error:
                        print('[기록 실패, 종료]',storage_error,flush=True)
                        result_code[0]=1
                        exit_event.set()
                    if args.commands:
                        result_code[0]=1
                        exit_event.set()
                finally:
                    busy.clear()
                    requests.task_done()
        def submit(line):
            cmd=line.strip().lower()
            if cmd in ('stop','x'):
                cancel.set()
                robot.stop()
                print('정지 요청됨. 실제 정지는 컨트롤러/장비에서 확인하세요.',flush=True)
            elif cmd=='pause':
                cancel.set()
            elif cmd in ('quit','q'):
                cancel.set()
                if busy.is_set():
                    robot.stop()
                exit_event.set()
            else:
                with submit_lock:
                    if busy.is_set():
                        print('작업 중입니다. pause / stop 사용 가능',flush=True)
                    else:
                        cancel.clear()
                        busy.set()
                        requests.put_nowait(line)
        def terminal():
            if args.commands:
                for line in args.commands.read_text().splitlines():
                    if not line.strip() or line.lstrip().startswith('#'):
                        continue
                    if exit_event.is_set():
                        return
                    submit(line)
                    while busy.is_set() and not exit_event.wait(.05):
                        pass
                exit_event.set()
            else:
                while not exit_event.is_set():
                    try:
                        line=input('zeus> ')
                    except EOFError:
                        submit('quit')
                        return
                    submit(line)
        worker=threading.Thread(target=execute,daemon=True)
        worker.start()
        threading.Thread(target=terminal,daemon=True).start()
        while not exit_event.is_set():
            if robot.broken.is_set():
                raise RuntimeError('Robot connection lost')
            if rig and not args.no_preview:
                key=rig.preview()
                if key in (27,ord('x'),ord('X')):
                    submit('stop')
                elif key==32:
                    submit('capture')
            else:
                exit_event.wait(.05)
        return result_code[0]
    except KeyboardInterrupt:
        cancel.set()
        robot.stop()
        return 130
    finally:
        cancel.set()
        exit_event.set()
        if busy.is_set():
            robot.stop()
        if worker:
            worker.join(6.)
        robot.close()
        if rig:
            rig.close()
        if not args.no_preview:
            import cv2
            cv2.destroyAllWindows()


if __name__=='__main__':
    sys.exit(main())
