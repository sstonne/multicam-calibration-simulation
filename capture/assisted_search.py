"""Feedback-guided bounded search. No camera world transform or controller reset.

All motion goes through the existing executor, with its ORIGINAL anchor/IK limits.
The search tree records traversed edges; travel between branches retraces that tree.
An application envelope is not a collision model or a certified safety boundary.
"""
import math
import time
import uuid

import numpy as np

from robot.zeus_capture_core import Guard, angle, distance, offset_pose, rotation


class BudgetReached(Exception):
    pass


def distinct(pose, accepted):
    """Translation alone cannot replace rotational coverage (reported separately)."""
    return all(distance(pose, r['robot_pose_6dof']) >= 1.5 or
               angle(pose, r['robot_pose_6dof']) >= 1.0 for r in accepted)


def rotation_vector(matrix):
    import cv2
    return cv2.Rodrigues(np.asarray(matrix, dtype=float))[0].ravel()


def coverage(records):
    """Diagnostic only; does not certify Shah accuracy/observability."""
    if len(records) < 2:
        return dict(count=len(records), translation_span_mm=0., rotation_span_deg=0.,
                    rotation_singular_values_deg=[0., 0., 0.], diverse=False)
    poses=[r['robot_pose_6dof'] for r in records]
    base=np.asarray(rotation(poses[0]))
    vectors=np.array([np.degrees(rotation_vector(np.asarray(rotation(p)) @ base.T)) for p in poses])
    singular=np.linalg.svd(vectors-vectors.mean(axis=0), compute_uv=False).tolist()
    singular=(singular+[0., 0., 0.])[:3]
    span=max(angle(a,b) for a in poses for b in poses)
    translation=max(distance(a,b) for a in poses for b in poses)
    return dict(count=len(records), translation_span_mm=translation, rotation_span_deg=span,
                rotation_singular_values_deg=singular,
                diverse=bool(span>=3. and singular[1]>=1.))


def assess(records, names):
    """Require 4/5 *joint* passes, unique frame pairs, and stable camera poses."""
    passed=[]
    seen={name:set() for name in names}
    counts={name:[] for name in names}
    scores=[]
    for r in records:
        fresh=True
        for name in names:
            c=r['cams'].get(name,{})
            seq=c.get('frame_sequence')
            if seq is None or seq in seen[name]:
                fresh=False
            seen[name].add(seq)
            counts[name].append(c.get('charuco',{}).get('n_corners',0))
        scores.append(r.get('quality_score',0.))
        if fresh and r['capture_gate']['pass']:
            passed.append(r)
    consistent=True
    max_mm=max_deg=0.
    for name in names:
        matrices=[np.asarray(r['cams'][name]['charuco']['T_cam_board_4x4'],float) for r in passed]
        for i,a in enumerate(matrices):
            for b in matrices[i+1:]:
                mm=float(np.linalg.norm(a[:3,3]-b[:3,3])*1000.)
                deg=float(np.linalg.norm(np.degrees(rotation_vector(a[:3,:3] @ b[:3,:3].T))))
                max_mm=max(max_mm,mm)
                max_deg=max(max_deg,deg)
                consistent=consistent and mm<=5. and deg<=2.
    robust=len(records)==5 and len(passed)>=4 and consistent
    medians={k:float(np.median(v)) for k,v in counts.items()}
    # Corners still provide a search signal when solvePnP fails (score is zero).
    merit=float(np.median(scores)) + .3*min(medians.values())/24. + .3*len(passed)/5.
    return dict(robust=bool(robust), joint_passes=len(passed), frames=len(records),
                median_corners=medians, merit=merit, max_pose_jitter_mm=max_mm,
                max_pose_jitter_deg=max_deg,
                best_event=max(passed,key=lambda r:r['quality_score'])['event_id'] if passed else None)


class AutoCollector:
    def __init__(self, operator):
        self.op=operator
        self.robot=operator.robot
        self.session=operator.session
        self.args=self.session.args
        self.cancel=operator.cancel
        self.accepted=[]
        self.nodes=[]
        self.frontier=[]
        self.proposed=[]
        self.current=0
        self.evaluated=0
        self.run_id=uuid.uuid4().hex[:8]
        self.report={}

    def check(self):
        if self.cancel.is_set():
            raise InterruptedError('자동 수집 일시중단')
        if time.monotonic()>=self.deadline:
            raise BudgetReached('시간 한도 도달')
        s=self.robot.state()
        if s['fault'] or not s['armed'] or s['moving'] or s['pending']:
            raise RuntimeError('자동 수집 중 로봇 상태 변경: '+str(s['fault']))
        if s['anchor']!=self.anchor:
            raise RuntimeError('원래 anchor가 변경됨')
        return s

    def inside(self,p):
        a=self.anchor['pose']
        return (distance(p,a)<=self.args.auto_radius_mm+1e-7 and
                angle(p,a)<=self.args.auto_angle_deg+1e-7)

    def travel(self,target):
        """Every short segment is checked against the original anchor, not recentered."""
        self.check()
        current=self.robot.state()['state']['flange_pose_6dof']
        if not self.inside(current) or not self.inside(target):
            raise ValueError('자동 탐색 범위 밖의 목표')
        # Euler interpolation matches the existing executor; reject discontinuities.
        if max(abs(a-b) for a,b in zip(current[3:],target[3:]))>10.:
            raise ValueError('회전 표현 불연속')
        n=max(1,int(math.ceil(distance(current,target)/.8)),
              int(math.ceil(sum(abs(a-b) for a,b in zip(current[3:],target[3:]))/.4)))
        for i in range(1,n+1):
            s=self.check()
            p=[a+(b-a)*i/n for a,b in zip(current,target)]
            if not self.inside(p):
                raise ValueError('중간 자세가 자동 탐색 범위 밖')
            Guard().path(s['state']['flange_pose_6dof'],p,self.anchor['pose'])
            self.op.move(p)
            self.check()
            items=self.session.rig.fresh(time.monotonic(),timeout=2.)
            if any(v is None for v in items.values()):
                raise RuntimeError('카메라 프레임 유실; 현재 위치에서 중단')
            if all(v['quality']['charuco']['n_corners']==0 for v in items.values()):
                raise InterruptedError('두 카메라가 보드를 잃음; 자동 복귀 없이 중단')

    def route(self,destination):
        def ancestors(i):
            result=[]
            while i is not None:
                result.append(i)
                i=self.nodes[i]['parent']
            return result
        src=ancestors(self.current)
        dst=ancestors(destination)
        common=next(i for i in src if i in dst)
        route=src[1:src.index(common)+1]+list(reversed(dst[:dst.index(common)]))
        for i in route:
            self.travel(self.nodes[i]['pose'])
            self.current=i

    def evaluate(self):
        self.evaluated+=1
        records=[]
        print('AUTO 후보 %d/%d · 채택 %d/%d · 프레임 5쌍 검사' %
              (self.evaluated,self.args.auto_max_candidates,len(self.accepted),self.goal),flush=True)
        for i in range(5):
            self.check()
            r=self.session.capture('auto_%s_%03d_%d' % (self.run_id,self.evaluated,i+1),
                                   evaluation_only=True)
            records.append(r)
            if not r['robot_after'] or not self.robot_stable(r):
                raise RuntimeError('촬영 중 로봇 상태 불안정')
            if 'robot_state_or_stability_unverified' in r['capture_gate'].get('reason',''):
                raise RuntimeError('촬영 중 로봇 상태/도구 기준 검증 실패')
            from assisted_capture import state_compatible
            if not state_compatible(records[0]['robot_before'],r['robot_after']):
                raise RuntimeError('5프레임 검증 구간에서 로봇 자세가 변함')
            cams=list(r['cams'].values())
            if len(cams)!=2 or any(not c.get('saved') for c in cams):
                raise RuntimeError('카메라 취득/저장 실패')
            if all(c.get('charuco',{}).get('n_corners',0)==0 for c in cams):
                raise InterruptedError('두 카메라가 보드를 잃음; 자동 복귀 없이 중단')
        result=assess(records,self.names)
        representative=next((r for r in records if r['event_id']==result['best_event']),records[-1])
        accepted=result['robust'] and distinct(representative['robot_pose_6dof'],self.accepted)
        for r in records:
            r['auto_validation']=dict(result,run_id=self.run_id)
        if accepted:
            representative['evaluation_only']=False
            representative['eligible_for_calibration']=self.robot.mode!='simulate'
            representative['auto_selected']=True
            self.accepted.append(representative)
        self.session.flush()
        print('AUTO 동시 통과 %d/5 · 코너 중앙값 %s · %s · 채택 %d/%d' %
              (result['joint_passes'],result['median_corners'],
               '채택' if accepted else ('중복 자세' if result['robust'] else '추가 탐색'),
               len(self.accepted),self.goal),flush=True)
        self.save('collecting')
        return result

    @staticmethod
    def robot_stable(record):
        from assisted_capture import state_compatible
        return state_compatible(record['robot_before'],record['robot_after'])

    def expand(self,index):
        node=self.nodes[index]
        # Probe all six axes, then continue from promising observations. Include
        # coupled rotations to avoid a translation-only or single-axis dataset.
        deltas=[]
        for axis in (3,4,5,0,1,2):
            for sign in (1,-1):
                d=[0.]*6
                d[axis]=sign*(1. if axis<3 else .75)
                deltas.append(d)
        for a,b in ((3,4),(4,5),(3,5)):
            for sign in (1,-1):
                d=[0.]*6
                d[a],d[b]=.5,sign*.5
                deltas.append(d)
        for d in deltas:
            try:
                p=offset_pose(node['pose'],d)
            except ValueError:
                continue
            if not self.inside(p) or any(distance(p,q)<.25 and angle(p,q)<.25 for q in self.proposed):
                continue
            self.proposed.append(p)
            self.frontier.append(dict(parent=index,pose=p))

    def priority(self,candidate):
        parent=self.nodes[candidate['parent']]
        novelty=1.
        if self.accepted:
            novelty=min(min(2.,angle(candidate['pose'],r['robot_pose_6dof'])/1.)+
                        .15*min(2.,distance(candidate['pose'],r['robot_pose_6dof'])/1.5)
                        for r in self.accepted)
        # Camera feedback guides the search. Distance/rotation novelty guides
        # collection; retaining alternate branches allows escape from local dips.
        return parent['quality']['merit'] + .3*novelty - .015*parent['depth']

    def save(self,status,detail=None):
        from assisted_capture import atomic_json
        self.report=dict(run_id=self.run_id,status=status,detail=detail,
                         camera_pair=self.args.cameras,goal=self.goal,evaluated=self.evaluated,
                         original_anchor=self.anchor,
                         search_radius_mm=self.args.auto_radius_mm,search_angle_deg=self.args.auto_angle_deg,
                         validation=dict(frames=5,required_joint_passes=4,
                                         max_pose_jitter_mm=5.,max_pose_jitter_deg=2.,
                                         min_corners=self.args.min_corners),
                         selected_event_ids=[r['event_id'] for r in self.accepted],
                         coverage=coverage(self.accepted),simulation=self.robot.mode=='simulate',
                         note='Coverage is a screening diagnostic, not proof of Shah calibration accuracy.')
        atomic_json(self.session.path/('auto_report_'+self.run_id+'.json'),self.report)
        data=dict(self.session.meta)
        data['captures']=[r for r in self.accepted if r['eligible_for_calibration']]
        data['auto_collection']=self.report
        atomic_json(self.session.path/('auto_valid_meta_'+self.run_id+'.json'),data)

    def run(self,goal):
        if not 3<=goal<=100:
            raise ValueError('auto 목표 개수는 3..100')
        if self.session.rig is None or set(self.args.cameras) not in (
                {'cam0','cam1'},{'cam0','cam3'},{'cam1','cam3'}):
            raise ValueError('auto는 테이블 카메라 (0,1), (0,3), (1,3)만 지원; cam2 제외')
        s=self.robot.state()
        if not s['anchor'] or not s['armed'] or s['fault'] or s['pending'] or s['moving']:
            raise ValueError('정지 상태에서 anchor, arm 후 auto 실행')
        self.anchor=s['anchor']
        self.goal=goal
        self.names=[str(c.index) for c in self.session.rig.cameras]
        self.deadline=time.monotonic()+60*self.args.auto_max_minutes
        status,detail='incomplete',None
        try:
            p=s['state']['flange_pose_6dof']
            if not self.inside(p):
                raise ValueError('현재 자세가 설정한 원래 anchor 탐색 범위 밖')
            initial=self.evaluate()
            if any(v==0 for v in initial['median_corners'].values()):
                raise InterruptedError('시작점에서 두 카메라 모두 보드 일부가 보여야 합니다')
            self.nodes=[dict(pose=p,parent=None,quality=initial,depth=0)]
            self.proposed=[p]
            self.expand(0)
            while self.frontier and self.evaluated<self.args.auto_max_candidates:
                self.check()
                if len(self.accepted)>=goal and coverage(self.accepted)['diverse']:
                    status='complete'
                    break
                item=max(self.frontier,key=self.priority)
                self.frontier.remove(item)
                self.route(item['parent'])
                # A refusal may occur after earlier subsegments moved. Abort here;
                # never assume that the arm stayed at the parent or resend it.
                self.travel(item['pose'])
                quality=self.evaluate()
                actual=self.robot.state()['state']['flange_pose_6dof']
                self.nodes.append(dict(pose=actual,parent=item['parent'],quality=quality,
                                       depth=self.nodes[item['parent']]['depth']+1))
                self.current=len(self.nodes)-1
                self.expand(self.current)
            if len(self.accepted)>=goal and coverage(self.accepted)['diverse']:
                status='complete'
            if status!='complete':
                detail='후보 한도/탐색 범위 내 목표 개수 또는 회전 다양성 미달'
        except BudgetReached as error:
            detail=str(error)
        except InterruptedError as error:
            status,detail='paused',str(error)
        except Exception as error:
            status,detail='aborted',str(error)
            raise
        finally:
            # No automatic homing, fault reset, anchor change, or re-arming.
            try:
                self.robot.operate('disarm')
            finally:
                self.save(status,detail)
                self.session.event('auto_finished',**self.report)
                print('AUTO %s · 채택 %d/%d · %s · disarm 요청됨' %
                      (status,len(self.accepted),goal,detail or '개수 및 회전 다양성 검사 통과'),flush=True)
                print('선택 데이터: '+str(self.session.path/('auto_valid_meta_'+self.run_id+'.json')),flush=True)
