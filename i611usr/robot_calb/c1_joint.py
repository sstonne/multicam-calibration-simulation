#!/usr/bin/python
# -*- coding: utf-8 -*-

from i611_MCS import *
from teachdata import *
from i611_extend import *
from rbsys import *
from i611_common import *
from i611_io import *
from i611shm import *
import sys
import time
import socket
import json
from waypoint_safety import (
    SAFE_EMPTY_KEY,
    SAFE_GRIPPED_KEY,
    SAFE_MODE_KEY,
    SAFE_MODE_Z_LIFT,
    shortest_joint_error_deg,
    validate_safe_joint_config,
    validate_waypoint_semantics,
)

HOST = '0.0.0.0'
PORT = 12348

GRIPPER_IO_PORT = 48
GRIPPER_TIMEOUT_SEC = 5.0

# 이 저장소의 큐브는 59mm 다(config.py CubeConfig.cube_side_m = 0.059, 캘리퍼 확인).
# 30.0 은 형제 프로젝트(rb-ArucoCube_Robot_multi_calibration)의 큐브 값이다.
CUBE_SIZE_MM = 59.0
# 그리퍼는 큐브 윗면(마커 양옆)을 잡고 윗면보다 이만큼 아래까지 물린다.
# 2026-08-10: 1.7mm 로는 살짝만 걸쳐 잡혀 옮기다 놓쳤다. 실제로 확실히 물리는 TCP
# 높이(-5.2)에서 역산한 2.0mm 로 올린다(캘리퍼 1.7mm 와 0.3mm 차이는 측정 오차 범위).
CUBE_GRIP_DEPTH_MM = 2.0
# fingertip(TCP) 에서 큐브 중심까지의 거리. 윗면이 중심보다 CUBE_SIZE/2 위에 있고
# fingertip 은 윗면보다 GRIP_DEPTH 아래이므로, 중심은 fingertip 보다 이만큼 "아래"다.
CUBE_CENTER_OFFSET_Z = CUBE_SIZE_MM / 2.0 - CUBE_GRIP_DEPTH_MM  # 27.5mm

# fingertip 까지의 실제 tool 오프셋. 교시 파일의 place_tcp 를 역산하면 114.99mm 로,
# 이 값이 교시 당시 규약이다. (150.0 은 펜던트 표시값을 그대로 옮긴 것이라 fingertip
# 보다 34.5mm 아래를 가리켰다.) tool4(큐브 중심)가 이 값에서 파생되므로 여기가 기준이다.
TOOL_GRIPPER_Z = 115.5
# 큐브 중심은 fingertip 보다 아래 = 플랜지에서 더 먼 쪽이므로 tool 오프셋은 "더한다".
# (뺄셈은 중심을 fingertip 위로 놓아 부호가 뒤집힌다 — 윗면을 잡는 이 그리퍼에서는
#  큐브가 TCP 아래에 매달리므로 물리적으로 불가능한 배치였다.)
TOOL_CUBE_CENTER_Z = TOOL_GRIPPER_Z + CUBE_CENTER_OFFSET_Z  # 177.5mm

# 그립/놓기가 일어나는 TCP 높이(base frame, mm). 모든 set 에서 이 높이로 강제한다.
#
# 티칭 파일의 place_tcp z 를 그대로 쓰지 않는 이유: 그 값은 티칭 당시의 settool 값으로
# 기록되어 있어 tool 오프셋을 바꾸면 통째로 어긋난다(예: TOOL_GRIPPER_Z 113.5 -> 150.0
# 이면 같은 관절인데 기록된 z 가 ~36mm 높게 남는다). place_joints 는 tool 과 무관하므로
# 관절 이동은 그대로 두고, 마지막 수직 접근의 목표 z 만 여기로 정규화한다.
#
# 값은 실제 로봇에서 큐브를 올바르게 쥔 자세의 TCP z 를 읽어 넣는다. None 이면 정규화하지
# 않고 place_joints 도달 시 읽은 z 를 그대로 쓴다.
#
# 2026-08-14 부터 None 이다. 이 상수는 "활성 tool 이 무엇이냐"에 통째로 의존한다 —
# 같은 물리적 높이가 tool3(115.5) 에서는 +30.6, tool1(플랜지) 에서는 +146.1 로 읽힌다.
# 기록용 포즈를 플랜지(tool1)로 바꾸면서 이 절대값을 그대로 두면 그리퍼가 테이블
# 아래로 150mm 내려간다. 지금은 모든 접근이 교시된 place_joints 로 관절 이동한 뒤
# 상대 수직 이동만 하므로, 높이는 교시값이 정의하고 tool 설정과 무관해졌다.
# 세트마다 교시 높이를 강제로 통일하고 싶으면 그때의 활성 tool 기준 값을 넣을 것.
PLACE_TCP_Z_MM = None
# 그리퍼가 큐브를 "잡을" 때의 TCP 높이. 놓기와 집기의 적정 높이가 다를 수 있어 상수를
# 분리해 두었지만, 현재는 둘 다 None(교시값 그대로)이다.
GRIP_TCP_Z_MM = None

# 큐브를 잡을 때(재-그립) 항상 place 위치 +Z 위에서 접근 후 수직 하강하여 안전하게 잡는다.
GRIP_APPROACH_Z_MM = 100.0
# B(grip-sweep) TCP pose 로 line 이동 시: 목표 +Z 위로 먼저 간 뒤 하강 (급격한 직행 방지)
TCP_APPROACH_Z_MM = 40.0
# 큐브 내려놓기: 항상 place 자세 +Z 위(정자세)로 올렸다가 그대로 수직 하강하여 놓는다.
PLACE_APPROACH_Z_MM = 50.0
# set 사이 이동: 잡은 뒤 이만큼 올려 수평으로 옮기고, 도착해서 같은 만큼 내린다.
# 큐브가 이 높이로 워크스페이스를 가로지르므로 테이블 위 장애물보다 높아야 한다.
TRANSIT_LIFT_Z_MM = 50.0
# 촬영 종료 후 큐브를 set0 에 반납하고 물러나는 높이.
FINAL_LIFT_Z_MM = 100.0
# safe_pose_mode=z_lift_only 일 때 매 전이에서 물러나는 높이. 티칭된 안전자세가
# 없으므로 "현재 위치에서 수직으로 물러난다"가 유일한 공통 후퇴 동작이다.
# B(큐브 그립)에만 적용된다 — A 는 retract=False 로 직행한다. 100mm 는 매 캡처마다
# 왕복하기엔 과해서 20mm 로 낮췄다. 테이블을 살짝 벗어나는 정도면 충분하고, 이후
# 관절 보간 경로는 어차피 이 값이 보장하지 못한다. 0 으로 두면 B 도 직행한다.
RETRACT_Z_MM = 20.0
SAFE_JOINT_TOL_DEG = 2.0
# 상공 관절자세에서 수직 하강한 뒤 "교시된 place 자세에 내려앉았는지" 확인하는 허용
# 오차. 컨트롤러 IK 가 같은 해를 고르면 거의 0 이지만, 손목 등가각이나 미세한 IK 분기
# 차이를 흡수할 만큼은 열어 둔다. 목적은 엉뚱한 set 의 상공값을 쓰거나 리프트 높이가
# 어긋난 경우처럼 "크게 틀린" 상황을 그리퍼 동작 전에 잡는 것이다.
DESCEND_LAND_TOL_DEG = 5.0
MOTION_STILL_TOL_DEG = 0.15
MOTION_STILL_SAMPLE_SEC = 0.25
# save gate 실패 시: 자동 지터 없이 곧바로 사람이 jog하는 manual_recover로 진입한다.

TCP_AXIS_MAP = {'x': 'dx', 'y': 'dy', 'z': 'dz', 'rz': 'drz', 'ry': 'dry', 'rx': 'drx'}
JOINT_AXIS_MAP = {'d1': 'dj1', 'd2': 'dj2', 'd3': 'dj3', 'd4': 'dj4', 'd5': 'dj5', 'd6': 'dj6'}
VALID_AXES = set(list(TCP_AXIS_MAP.keys()) + list(JOINT_AXIS_MAP.keys()))


# ── Socket ──

# Newline-delimited JSON framing.
# 한 메시지가 단일 recv() 청크 크기(예: waypoints_data 응답은 15KB+)를 넘거나
# 여러 메시지가 한 청크에 합쳐져 도착해도 안전하게 한 건씩 잘라서 반환한다.
_RECV_BUF = {'data': b''}


def send_json(conn, obj):
    try:
        msg = json.dumps(obj)
        conn.sendall((msg + '\n').encode('utf-8'))
        print "Sent: {}".format(msg)
    except socket.error as e:
        print "Send error: {}".format(e)


def recv_json(conn):
    """Receive one newline-delimited JSON object (handles large/split messages)."""
    try:
        while b'\n' not in _RECV_BUF['data']:
            chunk = conn.recv(65536)
            if not chunk:
                # peer closed; try to parse any unterminated remainder.
                if _RECV_BUF['data']:
                    line = _RECV_BUF['data']
                    _RECV_BUF['data'] = b''
                    try:
                        return json.loads(line.decode('utf-8').strip())
                    except Exception as e:
                        print "Recv parse error: {}".format(e)
                return None
            _RECV_BUF['data'] += chunk
        line, _, rest = _RECV_BUF['data'].partition(b'\n')
        _RECV_BUF['data'] = rest
        return json.loads(line.decode('utf-8').strip())
    except socket.error as e:
        print "Recv error: {}".format(e)
    except Exception as e:
        print "Recv parse error: {}".format(e)
    return None


# ── Robot helpers ──

def read_key(prompt):
    """Enter 없이 한 글자를 읽는다. tty 가 아니면 raw_input 으로 폴백한다.

    raw 모드에서는 커널이 Ctrl-C 를 SIGINT 로 바꾸지 않으므로 \x03 을 직접 잡아
    KeyboardInterrupt 로 올린다. 로봇 앞에서 Ctrl-C 가 먹지 않는 상태를 만들면 안 된다.
    """
    sys.stdout.write(prompt)
    sys.stdout.flush()
    try:
        import termios
        import tty
        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)
        sys.stdout.write('\n')
        sys.stdout.flush()
        if ch == '\x03':
            raise KeyboardInterrupt
        return ch
    except Exception:
        # tty 가 아니거나 termios 를 못 쓰는 환경. KeyboardInterrupt 는
        # Exception 을 상속하지 않으므로 여기서 삼켜지지 않는다.
        line = raw_input()
        return line[:1] if line else ' '


def fmt6(v):
    return '[{:.1f}, {:.1f}, {:.1f}, {:.1f}, {:.1f}, {:.1f}]'.format(
        v[0], v[1], v[2], v[3], v[4], v[5])


# 평상시 활성 tool. 1 = 플랜지(오프셋 0). eye-in-hand 캘리브레이션이 로봇 FK 를
# 플랜지 기준으로 쓰므로, PC 로 보내는 포즈도 플랜지여야 한다. tool 오프셋은 그때
# hand-eye X 에 흡수되므로 여기서 더하면 이중 계산이 된다.
TOOL_BASE = 1
# 큐브 중심 실측용 tool. TOOL_CUBE_CENTER_Z 로 settool 되어 있다.
TOOL_CUBE_CENTER = 4


def get_tcp():
    return rb.getpos().pos2list()[:6]


def get_cube_center():
    """tool4(큐브 중심)로 잠시 바꿔 읽고 반드시 평상시 tool 로 되돌린다.

    되돌릴 tool 을 하드코딩하면(구: changetool(3)) 시작 tool 과 어긋나 세션 도중
    기준 좌표계가 조용히 바뀐다 — 첫 실측 이후의 모든 기록 포즈가 다른 프레임이
    되므로 캘리브레이션이 통째로 망가진다.
    """
    rb.changetool(TOOL_CUBE_CENTER)
    tcp = rb.getpos().pos2list()[:6]
    rb.changetool(TOOL_BASE)
    return tcp


def send_teach(conn, kind, data):
    """teach 기록(recpose/recgrip/recset)을 PC로 전송해 PC에만 저장하도록 한다.

    kind: 'pose'(뷰포인트/A) | 'grip'(그립-스윕/B) | 'set'(큐브 배치).
    로봇 로컬에는 저장하지 않는다. PC(Step2)가 받아서 세션 번호 붙은 파일로 기록한다.
    매번 전체 리스트를 보내 PC가 파일을 통째로 갱신하게 한다(undo 도 동일).
    """
    send_json(conn, {"command": "teach_save", "kind": kind, "data": data})


def get_joints():
    return rb.getjnt().jnt2list()[:6]


def show_pose():
    tcp = get_tcp()
    jnt = get_joints()
    print ''
    print '     joints: [{:.2f}, {:.2f}, {:.2f}, {:.2f}, {:.2f}, {:.2f}]'.format(
        jnt[0], jnt[1], jnt[2], jnt[3], jnt[4], jnt[5])
    print '     tcp:    ({:.1f}, {:.1f}, {:.1f}) / ({:.1f}, {:.1f}, {:.1f})'.format(
        tcp[0], tcp[1], tcp[2], tcp[3], tcp[4], tcp[5])
    print ''
    return tcp


def move_tcp(axis, value):
    if axis not in TCP_AXIS_MAP:
        print 'Invalid axis: {}. Use x,y,z,rz,ry,rx'.format(axis)
        return
    current = Position(*rb.getpos().pos2list()[:6])
    rb.line(current.offset(**{TCP_AXIS_MAP[axis]: value}))
    print 'TCP {} += {} done'.format(axis, value)


def move_joint(axis, value):
    if axis not in JOINT_AXIS_MAP:
        print 'Invalid axis: {}. Use d1~d6'.format(axis)
        return
    current = Joint(*rb.getjnt().jnt2list()[:6])
    rb.move(current.offset(**{JOINT_AXIS_MAP[axis]: value}))
    print 'Joint {} += {} done'.format(axis, value)


def undo_one(entry):
    mtype, maxis, mvalue = entry
    print '  {} {},{} -> {}'.format(mtype, maxis, mvalue, -mvalue)
    if mtype == 'p':
        move_tcp(maxis, -mvalue)
    else:
        move_joint(maxis, -mvalue)


# 등가각을 고를 때 벗어나면 안 되는 축별 범위. 티칭된 값들은 실제로 도달한 자세이므로
# 그 범위(+여유)를 넘지 않으면 안전하다. _run_auto_multiset 이 웨이포인트에서 채운다.
JOINT_BAND_PAD_DEG = 30.0
_JOINT_BAND = {'lo': None, 'hi': None}


def set_joint_band_from_waypoints(waypoints):
    """웨이포인트의 모든 관절값에서 축별 허용 범위를 만든다.

    한계표를 로봇에서 못 읽으므로, "이미 티칭으로 도달한 각도"를 도달 가능성의 근거로
    쓴다. 이 범위를 벗어나는 등가각은 같은 자세라도 명령하지 않는다 — 그렇지 않으면
    순차 정규화가 d6 를 한 바퀴씩 감아 -690 처럼 한계 밖 값으로 걸어나간다.
    """
    lo = [None] * 6
    hi = [None] * 6
    for wp in waypoints:
        for key in ('capture_joints', 'place_joints'):
            j = wp.get(key)
            if not j:
                continue
            for i in range(6):
                v = float(j[i])
                lo[i] = v if lo[i] is None else min(lo[i], v)
                hi[i] = v if hi[i] is None else max(hi[i], v)
    if any(v is None for v in lo):
        return None
    _JOINT_BAND['lo'] = [v - JOINT_BAND_PAD_DEG for v in lo]
    _JOINT_BAND['hi'] = [v + JOINT_BAND_PAD_DEG for v in hi]
    print '[Auto] 등가각 허용 범위(티칭 실측 +-{:.0f}deg):'.format(JOINT_BAND_PAD_DEG)
    print '       ' + '  '.join('d{}[{:.0f},{:.0f}]'.format(
        i + 1, _JOINT_BAND['lo'][i], _JOINT_BAND['hi'][i]) for i in range(6))
    return _JOINT_BAND


def nearest_joint_target(rb, target_joints):
    """목표 관절을 현재 자세에 가장 가까운 +-360 등가각으로 바꾼다.

    티칭 파일의 d6 는 -351.8 처럼 한 바퀴 감긴 값으로 기록돼 있다. 그대로 명령하면
    같은 자세인데도 컨트롤러가 300도 넘게 되돌아 도는 경로를 만든다(시간도, 케이블도,
    간섭 위험도 커진다).

    단, 후보는 티칭 범위(set_joint_band_from_waypoints) 안에 있어야 한다. 범위를 두지
    않으면 자세가 이어질수록 d6 가 한 바퀴씩 더 감겨 관절 한계 밖으로 걸어나간다.
    범위 안에 아무 등가각도 없으면 원본 값을 그대로 쓴다(티칭된 값은 도달 가능하다).
    """
    cur = rb.getjnt().jnt2list()[:6]
    lo, hi = _JOINT_BAND['lo'], _JOINT_BAND['hi']
    out = []
    for i in range(6):
        t = float(target_joints[i])
        c = float(cur[i])
        k0 = int(round((c - t) / 360.0))
        best = None
        # 가까운 쪽부터 훑어 범위 안에 드는 첫 후보를 쓴다.
        for dk in (0, -1, 1, -2, 2, -3, 3):
            cand = t + 360.0 * (k0 + dk)
            if lo is not None and not (lo[i] <= cand <= hi[i]):
                continue
            if best is None or abs(cand - c) < abs(best - c):
                best = cand
        out.append(best if best is not None else t)
    return out


def move_joint_shortest(rb, target_joints, label=''):
    """최단 등가각으로 관절 이동. 도달 불가면 원본 값으로 한 번 재시도한다."""
    shortest = nearest_joint_target(rb, target_joints)
    turned = max(abs(shortest[i] - float(target_joints[i])) for i in range(6))
    if turned > 1.0:
        print '[Auto] {}관절 목표를 최단 등가각으로 보정 (최대 {:.0f}deg 단축)'.format(
            (label + ' ') if label else '', turned)
    try:
        rb.move(Joint(*shortest[:6]))
        return shortest
    except Exception as e:
        print '[WARN] 최단 등가각 이동 실패({}); 원본 관절값으로 재시도'.format(e)
        rb.move(Joint(*[float(x) for x in target_joints[:6]]))
        return [float(x) for x in target_joints[:6]]


def retract_z(rb, dz_mm, label):
    """+Z 로 물러난다. 도달 불가면 절반씩 줄여 가능한 만큼만 물러난다.

    높은 자세에서 무조건 +100mm 를 명령하면 작업영역을 벗어나 Unreachable 로 죽는다.
    후퇴는 안전을 위한 동작이므로, 전부 실패할 때만 예외를 올린다.
    """
    dz = float(dz_mm)
    while dz >= 5.0:
        try:
            cur = Position(*rb.getpos().pos2list()[:6])
            rb.line(cur.offset(dz=dz))
            if dz < float(dz_mm):
                print '[SAFE] {} 리트랙트 {:.0f}mm 로 축소(작업영역 한계)'.format(label, dz)
            return dz
        except Exception:
            dz = dz / 2.0
    print '[SAFE] {} 리트랙트 불가(이미 상한). 그대로 진행'.format(label)
    return 0.0


def verify_robot_still(rb, tolerance_deg=MOTION_STILL_TOL_DEG,
                       sample_sec=MOTION_STILL_SAMPLE_SEC):
    """Confirm that joints do not keep changing after a blocking motion call."""
    first = rb.getjnt().jnt2list()[:6]
    time.sleep(sample_sec)
    second = rb.getjnt().jnt2list()[:6]
    errors = shortest_joint_error_deg(first, second)
    max_error = max(errors)
    if max_error > tolerance_deg:
        raise RuntimeError(
            'robot still moving: sampled joint delta {:.3f}deg > {:.3f}deg'.format(
                max_error, tolerance_deg))
    return [float(x) for x in second]


def verify_at_joint_pose(rb, target_joints, tolerance_deg=SAFE_JOINT_TOL_DEG):
    actual = rb.getjnt().jnt2list()[:6]
    errors = shortest_joint_error_deg(actual, target_joints)
    max_error = max(errors)
    if max_error > tolerance_deg:
        raise RuntimeError(
            'safe pose reach error {:.3f}deg > {:.3f}deg'.format(max_error, tolerance_deg))
    return [float(x) for x in actual]


def move_to_validated_safe(rb, safe_joints, safe_kind, transition_label,
                           retract=True):
    """Execute and verify the mandatory current -> safe portion of a transition.

    ``safe_joints=None`` means the waypoint declared safe_pose_mode=z_lift_only:
    there is no taught safe pose, so retract straight up by RETRACT_Z_MM instead.
    The lift bounds the start of the path but not the joint interpolation that
    follows, so the whole run still has to be validated by a slow dry-run.

    ``retract=False`` skips even that lift and goes straight to the target. It is
    for A_placement, where the cube already sits on the table and the gripper is
    empty: nothing is carried, so lifting 100mm between every viewpoint only
    costs time. B keeps the lift because the cube is held.
    """
    started = time.time()
    if safe_joints is None and not retract:
        print '[SAFE] {} -> 직행 (A 블록, 리트랙트 없음)'.format(transition_label)
        actual = verify_robot_still(rb)
        return {
            'state_machine': 'direct_to_target_v1',
            'safe_transition_verified': True,
            'safe_pose_kind': 'none_direct',
            'retract_z_mm': 0.0,
            'safe_joints_commanded': None,
            'safe_joints_actual': actual,
            'safe_move_started_epoch_s': started,
            'safe_reached_epoch_s': time.time(),
        }
    if safe_joints is None:
        print '[SAFE] {} -> +Z {:.0f}mm retract (no taught safe pose)'.format(
            transition_label, RETRACT_Z_MM)
        retract_z(rb, RETRACT_Z_MM, transition_label)
        actual = verify_robot_still(rb)
        return {
            'state_machine': 'z_retract_then_target_v1',
            'safe_transition_verified': True,
            'safe_pose_kind': 'z_lift_only',
            'retract_z_mm': RETRACT_Z_MM,
            'safe_joints_commanded': None,
            'safe_joints_actual': actual,
            'safe_move_started_epoch_s': started,
            'safe_reached_epoch_s': time.time(),
        }
    print '[SAFE] {} -> {} safe pose'.format(transition_label, safe_kind)
    rb.move(Joint(*safe_joints[:6]))
    actual = verify_at_joint_pose(rb, safe_joints)
    verify_robot_still(rb)
    reached = time.time()
    print '[SAFE] reached {} (max joint error <= {:.1f}deg, stopped)'.format(
        safe_kind, SAFE_JOINT_TOL_DEG)
    return {
        'state_machine': 'current_to_safe_to_target_v1',
        'safe_transition_verified': True,
        'safe_pose_kind': safe_kind,
        'safe_joints_commanded': [float(x) for x in safe_joints[:6]],
        'safe_joints_actual': actual,
        'safe_move_started_epoch_s': started,
        'safe_reached_epoch_s': reached,
    }


# ── Gripper ──

def check_gripper():
    return [din(GRIPPER_IO_PORT + i) for i in [3, 2, 1, 0]]


def gripper_open():
    print 'Gripper opening...'
    dout(GRIPPER_IO_PORT, '0000')
    t0 = time.time()
    while check_gripper() != ['0', '1', '0', '0']:
        dout(GRIPPER_IO_PORT, '0100')
        if time.time() - t0 > GRIPPER_TIMEOUT_SEC:
            print '[WARN] Gripper open timeout!'
            break
        time.sleep(0.05)
    print 'Gripper opened'


def gripper_close():
    print 'Gripper closing...'
    dout(GRIPPER_IO_PORT, '0000')
    t0 = time.time()
    while check_gripper() != ['0', '0', '0', '1']:
        dout(GRIPPER_IO_PORT, '0001')
        if time.time() - t0 > GRIPPER_TIMEOUT_SEC:
            print '[WARN] Gripper close timeout!'
            break
        time.sleep(0.05)
    print 'Gripper closed'


# ── Capture ──

def do_capture(conn, capture_index, set_cube_center=None, set_index=None,
               set_joints=None, set_tcp=None, place_joints=None,
               cube_gripped=False, capture_block="A_placement", grasp_id=0,
               force_save=False, motion_safety=None):
    """Returns (status, tcp, cube_tcp) or (None, None, None) on disconnect.

    capture_block / cube_gripped / grasp_id tag each frame so Step3 can separate:
      A_placement  : cube released on table (set_cube_center anchor, method (a))
      B_eyetohand  : cube rigidly gripped, robot sweeps (eye-to-hand, method (b))
    """
    tcp = get_tcp()
    cube_tcp = get_cube_center()
    joints = get_joints()
    print ''
    print '*** CAPTURE {} (block={} gripped={} grasp={}) ***'.format(
        capture_index, capture_block, cube_gripped, grasp_id)
    print '  fingertip:    {}'.format(fmt6(tcp))
    print '  cube center:  {}'.format(fmt6(cube_tcp))

    msg = {
        "command": "capture",
        "capture_gripper_pose_6dof": tcp,
        "capture_cube_center_6dof": cube_tcp,
        "capture_robot_joints_6dof": joints,
        "capture_index": capture_index,
        "cube_gripped": bool(cube_gripped),
        "capture_block": capture_block,
        "grasp_id": int(grasp_id),
        "force_save": bool(force_save),
    }
    if motion_safety is not None:
        msg["motion_safety"] = motion_safety
    if set_cube_center is not None:
        msg["set_cube_center_6dof"] = set_cube_center
    if set_index is not None:
        msg["set_index"] = set_index
    if set_joints is not None:
        msg["set_joints"] = set_joints
    if set_tcp is not None:
        msg["set_tcp"] = set_tcp
    if place_joints is not None:
        msg["place_joints"] = place_joints

    send_json(conn, msg)
    resp = recv_json(conn)
    if resp is None:
        print 'Client disconnected!'
        return None, None, None

    status = resp.get('status', 'unknown') if isinstance(resp, dict) else 'unknown'
    reason = resp.get('reason') if isinstance(resp, dict) else None
    if reason:
        print '*** Capture {} done (status={}, reason={}) ***'.format(capture_index, status, reason)
    else:
        print '*** Capture {} done (status={}) ***'.format(capture_index, status)
    return status, tcp, cube_tcp


# ── Auto capture ──

def move_above_then_descend(rb, above_joints, label, drop_mm=GRIP_APPROACH_Z_MM,
                            expect_joints=None,
                            tol_deg=DESCEND_LAND_TOL_DEG):
    """상공 관절자세로 관절 이동한 뒤 그만큼 수직 하강한다. 반환: 도착 TCP 실측값.

    above_joints 는 place_joints 를 base +Z 로 drop_mm 들어올린 IK 해로, PC 가
    waypoint 파일의 place_above_joints 에 미리 넣어 둔다.

    절대 TCP 로 직선 이동하지 않는 것이 핵심이다.
      - tool 무관: 교시된 관절값이 높이를 정의하므로 settool 을 바꿔도 안 틀어진다.
        (파일의 place_tcp 는 교시 당시 tool 규약이라 지금 그대로 명령하면 어긋난다.)
      - Unreachable 회피: 촬영 자세에서 그립 상공까지의 절대 직선은 워크스페이스를
        가로지르며 특이점/영역 밖을 지나 컨트롤러가 죽는다(set10->11 전이에서 발생).
        끝점은 도달 가능한데 "그리로 가는 직선"이 불가능한 것이라, 경로만 관절
        보간으로 바꾸면 해결된다.
    마지막 하강은 큐브를 옆에서 밀지 않도록 순수 수직을 유지한다.
    """
    print '[Auto] {}: joint move to +Z {:.0f}mm above, then straight down'.format(
        label, drop_mm)
    try:
        move_joint_shortest(rb, above_joints, label)
        verify_robot_still(rb)
    except Exception as e:
        raise RuntimeError('{} joint approach failed: {}'.format(label, e))
    time.sleep(0.3)
    try:
        p = Position(*rb.getpos().pos2list()[:6])
        rb.line(p.offset(dz=-float(drop_mm)))
        verify_robot_still(rb)
    except Exception as e:
        raise RuntimeError('{} vertical descent failed: {}'.format(label, e))
    # 하강 후 교시된 place 자세에 실제로 내려앉았는지 관절로 확인한다. above_joints
    # 가 다른 set 의 것이거나 drop_mm 이 place_above_joints 생성 시의 리프트와
    # 다르면 여기서 걸린다 — 그리퍼를 닫거나 큐브를 놓기 전에 멈춰야 한다.
    if expect_joints is not None:
        try:
            verify_at_joint_pose(rb, expect_joints, tol_deg)
        except RuntimeError as e:
            raise RuntimeError(
                '{}: 하강 후 place 자세와 불일치 ({}). place_above_joints 가 이 set 의 '
                'place_joints 를 +{:.0f}mm 올린 값인지 확인할 것'.format(label, e, drop_mm))
    time.sleep(0.2)
    return rb.getpos().pos2list()[:6]


def approach_and_close_gripper(rb, place_joints, place_tcp=None,
                                approach_z_mm=GRIP_APPROACH_Z_MM,
                                above_joints=None):
    """그리퍼 닫기 전 항상 +Z 위에서 접근 후 하강하여 닫는다.

    above_joints(waypoint 의 place_above_joints)가 있으면 관절 이동으로 상공에
    간 뒤 수직 하강한다. 없으면 place_tcp 로 직선(line) 접근하는 구 동작.

    관절 이동을 우선하는 이유: 직전 촬영 자세에서 그립 상공까지의 절대 직선
    경로는 워크스페이스를 가로지르며 특이점/영역 밖을 지나 컨트롤러가
    Unreachable 로 죽는다(실제로 set10->11 전이에서 발생). 상공 자세는 도달
    가능한데도 그 자세로 "가는 직선"이 불가능한 것이라, 경로만 관절 보간으로
    바꾸면 해결된다. 마지막 하강은 큐브를 옆에서 밀지 않도록 순수 수직을 유지한다.

    place_tcp 의 XY/자세는 그대로 쓰되 하강 목표 z 는 GRIP_TCP_Z_MM 으로 바꾼다.
    놓을 때(PLACE_TCP_Z_MM)와 집을 때의 적정 높이가 다르기 때문이다.
    place_tcp 도 없으면 place_joints 로 직접 이동 후 닫는다(폴백).
    """
    if above_joints is not None:
        move_above_then_descend(rb, above_joints, 'grip approach', approach_z_mm,
                                expect_joints=place_joints)
    elif place_tcp is not None:
        target = list(place_tcp[:6])
        if GRIP_TCP_Z_MM is not None:
            target[2] = GRIP_TCP_Z_MM
        above = list(target)
        above[2] += approach_z_mm
        try:
            print '[Auto] +Z {:.0f}mm approach above grip pose (grip z {:.2f})'.format(
                approach_z_mm, target[2])
            rb.line(Position(*above))
            time.sleep(0.3)
            print '[Auto] descend to grip pose'
            rb.line(Position(*target))
            time.sleep(0.2)
        except Exception as e:
            raise RuntimeError('grip line approach failed: {}'.format(e))
    else:
        rb.move(Joint(*place_joints[:6]))
        time.sleep(0.3)
    gripper_close()


def approach_place_pose(rb, place_j, label, approach_z_mm=PLACE_APPROACH_Z_MM):
    """관절이동으로 정자세 place 에 도달 -> 높이 정규화 -> +Z 위로 갔다가 수직 하강.

    마지막 접근을 항상 순수 수직/정자세로 보장한다. place_joints 는 관절값이라 FK 없이
    상공 TCP 를 알 수 없으므로, 먼저 관절로 도달해 기준 TCP 를 읽은 뒤 올렸다가 내린다.
    반환: 정규화된 place TCP (재-그립/재접근 기준으로도 쓴다).
    """
    move_joint_shortest(rb, place_j, 'place')
    verify_robot_still(rb)
    time.sleep(0.3)
    place_tcp = get_tcp()
    if PLACE_TCP_Z_MM is not None:
        measured_z = place_tcp[2]
        place_tcp[2] = PLACE_TCP_Z_MM
        print '[Auto] {} place z {:.2f} -> {:.2f} (set 간 그립 높이 통일)'.format(
            label, measured_z, PLACE_TCP_Z_MM)
        if abs(measured_z - PLACE_TCP_Z_MM) > 20.0:
            # 20mm 넘게 어긋나면 티칭 tool 과 현재 tool 이 다르거나 이 set 만 다른
            # 높이에서 티칭된 것이다. 큐브를 테이블에 찍거나 공중에서 놓을 수 있다.
            print '[WARN] 티칭 높이와 {:.1f}mm 차이. tool 설정 또는 이 set 의 ' \
                  'place_joints 를 확인할 것.'.format(measured_z - PLACE_TCP_Z_MM)
    above = list(place_tcp[:6])
    above[2] += approach_z_mm
    print '[Auto] +Z {:.0f}mm upright, then straight down'.format(approach_z_mm)
    rb.line(Position(*above))
    time.sleep(0.2)
    rb.line(Position(*place_tcp[:6]))
    verify_robot_still(rb)
    time.sleep(0.5)
    return place_tcp


def jog_at_pose(rb, kind):
    """촬영 확인 중 한 축만 상대 이동하거나 속도를 바꾼다.

    read_key 는 한 글자만 받으므로, 여기서 나머지를 한 줄로 마저 읽는다.
    예외는 삼켜서 오타 때문에 자동 촬영 루프가 죽지 않게 한다.
    """
    try:
        if kind == 'v':
            raw = raw_input('  speed 0-100 > ').strip()
            spd = int(raw)
            if not (0 <= spd <= 100):
                print '  (0-100 범위)'
                return
            rb.override(spd)
            print '  Speed: {}'.format(spd)
            return
        prompt = '  p <axis>,<delta>  (x,y,z,rz,ry,rx) > ' if kind == 'p' \
                 else '  j <joint>,<delta>  (d1..d6) > '
        raw = raw_input(prompt).strip()
        if not raw:
            return
        axis, _, val = raw.partition(',')
        delta = float(val)
        if kind == 'p':
            move_tcp(axis.strip(), delta)
        else:
            move_joint(axis.strip(), delta)
    except (ValueError, IndexError) as e:
        print '  err: {} (형식: <axis>,<delta>)'.format(e)
    except Exception as e:
        print '  이동 실패: {}'.format(e)


def manual_recover(rb, conn, pose_idx, capture_kwargs):
    """Marker detection failed at an auto waypoint. Hand control to the operator to
    jog the robot until the cube is detected, then re-capture from the current pose.

    Returns 'success' / 'skip' / 'quit' / None(disconnect). Jog commands mirror the
    main manual loop (p / j / gotop / gotoj / show).
    """
    print ''
    print '  [Recover] marker not detected here. Manual jog until visible, then c.'
    print '    p <axis>,<v>  j <axis>,<v>  gotop x,y,z[,rz,ry,rx]  gotoj d1..d6'
    print '    show | c: re-capture | s: skip this pose | q: quit'
    while True:
        try:
            line = raw_input('  recover> ').strip()
        except EOFError:
            return 'skip'
        if not line:
            continue
        ll = line.lower()
        if ll == 'c':
            status, _, _ = do_capture(conn, pose_idx, **capture_kwargs)
            if status is None:
                return None
            if status == 'success':
                print '  [Recover] -> OK'
                return 'success'
            print '  [Recover] still failing (status={}); jog more, or s/q'.format(status)
        elif ll == 's':
            return 'skip'
        elif ll == 'q':
            return 'quit'
        elif ll == 'show':
            show_pose()
        elif ll.startswith('p '):
            try:
                parts = line[2:].split(',')
                move_tcp(parts[0].strip(), float(parts[1]))
                show_pose()
            except Exception as e:
                print '  err: {}. Usage: p <axis>,<value>'.format(e)
        elif ll.startswith('j '):
            try:
                parts = line[2:].split(',')
                move_joint(parts[0].strip(), float(parts[1]))
                show_pose()
            except Exception as e:
                print '  err: {}. Usage: j <axis>,<value>'.format(e)
        elif ll.startswith('gotop ') or ll.startswith('goto '):
            try:
                rest = line[6:] if ll.startswith('gotop ') else line[5:]
                vals = [float(v.strip()) for v in rest.split(',')]
                if len(vals) == 6:
                    rb.line(Position(*vals))
                elif len(vals) == 3:
                    t = get_tcp()
                    rb.line(Position(vals[0], vals[1], vals[2], t[3], t[4], t[5]))
                else:
                    print '  usage: gotop x,y,z[,rz,ry,rx]'
                    continue
                show_pose()
            except Exception as e:
                print '  err: {}'.format(e)
        elif ll.startswith('gotoj '):
            try:
                vals = [float(v.strip()) for v in line[6:].split(',')]
                if len(vals) == 6:
                    rb.move(Joint(*vals))
                    show_pose()
                else:
                    print '  usage: gotoj d1,d2,d3,d4,d5,d6'
            except Exception as e:
                print '  err: {}'.format(e)
        else:
            print '  (p / j / gotop / gotoj / show / c / s / q)'


def request_waypoints_from_pc(conn, timeout_sec=10.0):
    """Request capture_waypoints.json content from the PC over the socket.

    Returns the parsed dict on success, or None on failure / timeout.
    """
    print 'Requesting waypoints from PC...'
    send_json(conn, {"command": "request_waypoints"})
    conn.settimeout(timeout_sec)
    try:
        resp = recv_json(conn)
    except socket.timeout:
        print '[ERROR] PC did not respond within {}s'.format(timeout_sec)
        conn.settimeout(None)
        return None
    finally:
        try:
            conn.settimeout(None)
        except Exception:
            pass
    if not isinstance(resp, dict):
        print '[ERROR] invalid response from PC'
        return None
    if resp.get('status') != 'ok':
        print '[ERROR] PC reported error: {}'.format(resp.get('reason', 'unknown'))
        return None
    data = resp.get('waypoints_data')
    if not isinstance(data, dict):
        print '[ERROR] PC response missing waypoints_data'
        return None
    n_wps = len(data.get('waypoints', []))
    print '  received {} waypoints from PC'.format(n_wps)
    return data


def run_auto_capture(rb, conn, waypoint_file=None, speed=30):
    """Run auto capture. If waypoint_file is None or empty, request waypoints
    from PC over the socket. Otherwise, load from local filesystem (legacy).

    기본(semi-auto): 각 capture pose로 이동 후 좌표를 표시하고, 사람이 'c'+Enter로
    확인해야 실제 촬영한다. `--noconfirm` 플래그를 주면 확인 없이 전부 자동 촬영한다.
    """
    confirm = '--noconfirm' not in sys.argv
    if not waypoint_file:
        data = request_waypoints_from_pc(conn)
        if data is None:
            return
    else:
        with open(waypoint_file, 'r') as f:
            data = json.load(f)

    # Multi-set joint-based: waypoints[] has per-waypoint set_index (5+ sets)
    waypoints = data.get('waypoints', [])
    if not waypoints or not any('set_index' in wp for wp in waypoints):
        print '[ERROR] waypoints missing set_index (multi-set format required)'
        send_json(conn, {"command": "quit"})
        return
    try:
        validate_safe_joint_config(data)
        validate_waypoint_semantics(data)
    except ValueError as e:
        print '[SAFETY-ABORT] {}'.format(e)
        print '[SAFETY-ABORT] Auto capture requires explicit, physically validated '
        print '               safe_joints_empty and safe_joints_gripped.'
        send_json(conn, {"command": "quit"})
        return
    _run_auto_multiset(rb, conn, data, speed, confirm=confirm)


def _capture_at_pose(rb, conn, wp, sidx, place_j, set_cc,
                     cube_gripped, capture_block, grasp_id, confirm,
                     safe_joints, safe_kind, label='', retract=True):
    """safe pose -> waypoint -> 정지 확인 -> 촬영 -> safe pose.

    이동 방식: wp 에 'capture_joints' 가 있으면 관절 이동(rb.move), 없고 'capture_tcp'
    만 있으면 TCP 직교 이동(rb.line). Phase A(placement)는 관절, Phase B(grip-sweep)는
    set 위치로 평행이동된 TCP 를 쓰므로 line 으로 실행된다.

    반환: 'success' | 'skip' | 'quit' | 'disconnect' | 'abort'
    cube_gripped/capture_block/grasp_id 는 프레임 태그로 do_capture 에 전달되어
    나중에 Step3 --capture_block 로 방법(a/b)을 분리 캘리브할 수 있게 한다.
    """
    cap_j = wp.get('capture_joints')
    cap_tcp = wp.get('capture_tcp')
    pose_idx = wp.get('capture_index', wp.get('pose_index'))
    print ''
    print '  -- {} capture (set={}, capture_index={}, block={}, move={}) --'.format(
        label, sidx, pose_idx, capture_block, 'joint' if cap_j is not None else 'tcp')
    try:
        motion_safety = move_to_validated_safe(
            rb, safe_joints, safe_kind, 'capture {}'.format(pose_idx),
            retract=retract)
        target_started = time.time()
        if cap_j is not None:
            move_joint_shortest(rb, cap_j, 'capture')
        else:
            # 안전 접근: 목표 TCP 의 +Z {approach}mm 위로 먼저 line 이동한 뒤 하강.
            tgt = [float(x) for x in cap_tcp[:6]]
            above = list(tgt)
            above[2] += TCP_APPROACH_Z_MM
            print '    (+Z {:.0f}mm approach above then descend)'.format(TCP_APPROACH_Z_MM)
            rb.line(Position(*above))
            time.sleep(0.2)
            rb.line(Position(*tgt))
        target_actual = verify_robot_still(rb)
        motion_safety['target_move_started_epoch_s'] = target_started
        motion_safety['target_reached_epoch_s'] = time.time()
        motion_safety['target_still_verified'] = True
        motion_safety['target_joints_actual'] = target_actual
    except Exception as e:
        print '  [SAFETY-ABORT] safe/target transition failed: {}'.format(e)
        return 'abort'
    time.sleep(0.5)
    show_pose()

    if confirm:
        action = None
        while action is None:
            ans = read_key("  [SPACE]=촬영 [s]=skip [q]=quit "
                           "[p]/[j]=수동이동 [v]=속도 > ")
            if ans in (' ', '\r', '\n', 'c', 'C'):
                action = 'capture'
            elif ans in ('s', 'S'):
                action = 'skip'
            elif ans in ('q', 'Q'):
                action = 'quit'
            elif ans in ('p', 'P', 'j', 'J', 'v', 'V'):
                # 촬영 직전에 조금 어긋난 프레이밍을 그 자리에서 고칠 수 있게 한다.
                # 여기서 움직인 결과가 그대로 촬영·기록되므로, 목표 자세를 바꾸는
                # 것이 아니라 "이 포즈를 실제로 쓸 수 있게 만드는" 조정이다.
                jog_at_pose(rb, ans.lower())
                show_pose()
            else:
                print "  (SPACE=촬영 / s=skip / q=quit / p,j=이동 / v=속도)"
        if action == 'skip':
            print '  -> skipped by user'
            try:
                move_to_validated_safe(rb, safe_joints, safe_kind, 'skip return',
                                       retract=retract)
            except Exception as e:
                print '  [SAFETY-ABORT] cannot return to safe pose: {}'.format(e)
                return 'abort'
            return 'skip'
        if action == 'quit':
            print '  -> quit by user'
            try:
                move_to_validated_safe(rb, safe_joints, safe_kind, 'quit return',
                                       retract=retract)
            except Exception as e:
                print '  [SAFETY-ABORT] cannot return to safe pose: {}'.format(e)
                return 'abort'
            return 'quit'

    cap_kwargs = {
        "set_cube_center": set_cc,
        "set_index": sidx,
        "set_joints": place_j,
        "set_tcp": None,
        "place_joints": place_j,
        "cube_gripped": cube_gripped,
        "capture_block": capture_block,
        "grasp_id": grasp_id,
        # Final experiment data must satisfy the block-aware PC gate.
        "force_save": False,
        "motion_safety": motion_safety,
    }
    status, _, _ = do_capture(conn, pose_idx, **cap_kwargs)
    try:
        move_to_validated_safe(rb, safe_joints, safe_kind, 'post-capture return',
                                       retract=retract)
    except Exception as e:
        print '  [SAFETY-ABORT] cannot return to safe pose: {}'.format(e)
        return 'abort'
    if status is None:
        print '[Auto] disconnected, stopping.'
        return 'disconnect'
    if status != 'success':
        print '  [Auto] -> rejected by capture gate (status={})'.format(status)
        return 'skip'
    print '  [Auto] -> captured (gate PASS)'
    return 'success'


def _run_auto_multiset(rb, conn, data, speed, confirm=True):
    """Multi-set joint-based auto capture (start-command flow).

    각 set(waypoints[].set_index 그룹)마다, capture_block 태그로 두 촬영 방법을 한 번에:
      Phase B (B_eyetohand): 큐브를 그립한 채 각 grip-sweep pose로 이동하며 촬영
                             (cube_gripped=True). 고정 카메라가 움직이는 큐브를 관측
                             = eye-to-hand (method b). 이 set의 그립 하나 = grasp_id.
      -- place_joints 로 이동해 큐브를 바닥에 내려놓고(하강) tool4로 큐브중점 실측 ->
         그리퍼 오픈(큐브 릴리즈) -> +Z clearance --
      Phase A (A_placement): 큐브가 바닥에 놓인 상태로 각 뷰포인트에서 촬영
                             (cube_gripped=False) = placement (method a).
                             set_cube_center 는 위에서 실측한 값을 사용.
      -- 다음 set이 있으면 큐브 재-그립 후 +Z transit lift 하고 이동 --

    waypoint 의 capture_block 은 'B_eyetohand' 또는 'A_placement'로 명시해야 한다.
    구버전/오타/큐브 상태 불일치는 첫 모션 전에 fail-closed 한다.
    """
    waypoints = data.get('waypoints', [])
    if not waypoints:
        print '[ERROR] no waypoints'
        send_json(conn, {"command": "quit"})
        return

    try:
        safe_cfg = validate_safe_joint_config(data)
        validate_waypoint_semantics(data)
    except ValueError as e:
        print '[SAFETY-ABORT] {}'.format(e)
        send_json(conn, {"command": "quit"})
        return
    if safe_cfg is None:
        safe_empty = safe_gripped = None
        print '[WARN] safe_pose_mode=z_lift_only — 티칭된 안전자세 없이 실행한다.'
        print '       매 전이는 +Z {:.0f}mm 리트랙트로 대체된다.'.format(RETRACT_Z_MM)
    else:
        safe_empty = safe_cfg[SAFE_EMPTY_KEY]
        safe_gripped = safe_cfg[SAFE_GRIPPED_KEY]

    # Group by set_index, preserving first-appearance order.
    sets_order = []
    by_set = {}
    for wp in waypoints:
        sidx = wp.get('set_index')
        if sidx is None:
            print '[ERROR] waypoint capture_index={} missing set_index'.format(wp.get('capture_index', wp.get('pose_index')))
            send_json(conn, {"command": "quit"})
            return
        if 'place_joints' not in wp or ('capture_joints' not in wp and 'capture_tcp' not in wp):
            print '[ERROR] waypoint capture_index={} missing place_joints or capture_joints/capture_tcp'.format(wp.get('capture_index', wp.get('pose_index')))
            send_json(conn, {"command": "quit"})
            return
        if sidx not in by_set:
            by_set[sidx] = []
            sets_order.append(sidx)
        by_set[sidx].append(wp)

    set_joint_band_from_waypoints(waypoints)

    total_caps = len(waypoints)
    n_sets = len(sets_order)
    print ''
    print '=========================================='
    print '  Multi-Set Auto Capture'
    print '  - sets:     {} ({})'.format(n_sets, sets_order)
    print '  - captures: {}'.format(total_caps)
    print '  - speed:    {}'.format(speed)
    print '  - +Z grip approach:     {}mm'.format(GRIP_APPROACH_Z_MM)
    print '  - +Z place approach:    {}mm'.format(PLACE_APPROACH_Z_MM)
    print '  - +Z transit lift:      {}mm  (set 간 수평 이동 높이)'.format(TRANSIT_LIFT_Z_MM)
    if PLACE_TCP_Z_MM is None:
        print '  - place TCP z:          교시값 그대로 (tool 무관)'
    else:
        print '  - place TCP z:          {}mm  (모든 set 공통)'.format(PLACE_TCP_Z_MM)
    # 관절 이동으로 접근할 수 있는 set 이 몇 개인지 미리 보여 준다. 없는 set 은
    # 구 동작(절대 TCP 직선)으로 떨어지므로 tool 규약이 맞는지 확인해야 한다.
    n_above = sum(1 for s in sets_order
                  if by_set[s][0].get('place_above_joints') is not None)
    n_capj = sum(1 for wp in waypoints if wp.get('capture_joints') is not None)
    print '  - 관절 접근(place_above_joints): {}/{} set'.format(n_above, n_sets)
    print '  - 관절 촬영(capture_joints):     {}/{} pose'.format(n_capj, total_caps)
    print '  - 활성 tool:            {} (기록 포즈 = 플랜지), 큐브측정 tool {}'.format(
        TOOL_BASE, TOOL_CUBE_CENTER)
    if safe_empty is None:
        print '  - safe pose:            NONE (z_lift_only, +Z {}mm retract)'.format(RETRACT_Z_MM)
    else:
        print '  - safe(empty):          {}'.format(fmt6(safe_empty))
        print '  - safe(gripped):        {}'.format(fmt6(safe_gripped))
    print '=========================================='
    print ''
    print 'PRECONDITION: cube is gripped and robot is already at validated safe(gripped).'
    print 'Per set: Phase B grip-sweep (cube held) -> place & release -> Phase A placement.'
    raw_input('Press ENTER to confirm payload state, guarding, E-stop, and safe start pose...')

    rb.override(speed)
    success = 0
    skipped = 0

    try:
        # Do not plan an automatic path out of an unknown initial configuration.
        # The operator must start at the validated gripped-payload safe pose.
        # With z_lift_only there is no such pose; only require that the robot is
        # stopped, and let the operator's own start position stand.
        if safe_gripped is not None:
            verify_at_joint_pose(rb, safe_gripped)
        verify_robot_still(rb)
    except Exception as e:
        print '[SAFETY-ABORT] invalid initial safe(gripped) state: {}'.format(e)
        send_json(conn, {"command": "quit"})
        return

    # 직전 set 의 전이에서 수평 이동 후 하강해 도착한 TCP. None 이면 이 set 은
    # 안전자세에서 +Z 접근으로 새로 내려가야 한다(첫 set, 또는 전이 실패 폴백).
    arrived_tcp = None

    for si, sidx in enumerate(sets_order):
        wps = by_set[sidx]
        place_j = wps[0]['place_joints']
        # place 를 +GRIP_APPROACH_Z_MM 들어올린 관절값(PC 에서 IK 로 미리 계산).
        # 있으면 재-그립을 직선이 아닌 관절 이동으로 접근한다. 없으면 구 동작.
        place_above_j = wps[0].get('place_above_joints')
        # set별 큐브 중점: waypoint에 저장된 값 우선, 없으면 파일 최상위로 폴백.
        # 티칭 파일의 set_cube_center_6dof 는 폴백일 뿐이다. 그 값은 티칭 당시의 tool4
        # 설정으로 기록되므로 tool 을 고치면 통째로 어긋나고, 그것을 Phase B 에 쓰고
        # 실측값을 Phase A 에 쓰면 한 세션 안에 두 규약이 섞인다. Step3 는
        # set_cube_center 와 cube object frame 사이의 "상수" delta 를 학습하므로 그
        # 혼재는 prior 자체를 망가뜨린다. 아래에서 현재 tool 로 한 번 실측해 덮는다.
        set_cc = wps[0].get('set_cube_center_6dof') or data.get('set_cube_center')
        # 이 set 의 그립(Phase B 스윕) 하나 = 하나의 grasp. 재-그립 시 gripper->cube
        # 변환이 조금 달라지므로 set 마다 grasp_id 를 달리해 Step3 가 구분하게 한다.
        grasp_id = si

        # capture_block 으로 두 방법을 분리: B(그립 스윕) 먼저, A(placement) 나중.
        block_b = [wp for wp in wps if wp.get('capture_block') == 'B_eyetohand']
        block_a = [wp for wp in wps if wp.get('capture_block') != 'B_eyetohand']

        # ---- 큐브를 쥔 채 이 set 에 도착해 tool4 로 중심을 실측한다. B 와 A 가 같은
        #      값을 쓰려면 B 보다 먼저여야 한다. si>0 은 직전 set 의 전이에서 이미
        #      수평 이동 후 하강해 도착해 있으므로 다시 접근하지 않는다. ----
        if arrived_tcp is None:
            print '[Auto] set {} arrive (+Z {:.0f}mm approach)'.format(
                sidx, GRIP_APPROACH_Z_MM)
            try:
                move_to_validated_safe(rb, safe_gripped, 'gripped',
                                       'set {} arrive'.format(sidx))
                place_tcp = approach_place_pose(rb, place_j,
                                                'set {} arrive'.format(sidx),
                                                GRIP_APPROACH_Z_MM)
            except Exception as e:
                print '[SAFETY-ABORT] set {} arrive failed: {}'.format(sidx, e)
                send_json(conn, {"command": "quit"})
                return
        else:
            place_tcp = arrived_tcp
        try:
            set_cc = get_cube_center()
            print '[Auto] set {} cube center (tool4, measured): '.format(sidx) + fmt6(set_cc)
        except Exception as e:
            print '[SAFETY-ABORT] set {} get_cube_center() failed: {}'.format(sidx, e)
            send_json(conn, {"command": "quit"})
            return

        print ''
        print '======== SET {}/{} (set_index={}: B={} grip-sweep, A={} placement) ========'.format(
            si + 1, n_sets, sidx, len(block_b), len(block_a))

        # ---- Phase B: eye-to-hand. 큐브를 그립한 채(최초 그립 또는 이전 set 재-그립)
        #      각 sweep pose 로 이동하며 촬영. 고정 카메라가 움직이는 큐브를 관측. ----
        if block_b:
            print '[Auto] --- Phase B: {} grip-sweep captures (cube gripped, grasp_id={}) ---'.format(
                len(block_b), grasp_id)
        for wi, wp in enumerate(block_b):
            r = _capture_at_pose(
                rb, conn, wp, sidx, place_j, set_cc,
                cube_gripped=True, capture_block='B_eyetohand', grasp_id=grasp_id,
                confirm=confirm, safe_joints=safe_gripped, safe_kind='gripped',
                label='B {}/{}'.format(wi + 1, len(block_b)))
            if r == 'success':
                success += 1
            elif r == 'skip':
                skipped += 1
            elif r == 'quit':
                send_json(conn, {"command": "quit"})
                return
            elif r == 'disconnect':
                return
            elif r == 'abort':
                send_json(conn, {"command": "quit"})
                return

        # ---- 큐브 내려놓기: 항상 place 위 +Z{PLACE_APPROACH_Z_MM}mm 정자세에서 수직으로
        #      그대로 하강해 내려놓는다 -> tool4 로 큐브중점 실측 -> 릴리즈 -> +Z clearance.
        #      place_joints 는 관절값이라 FK 없이 상공 TCP 를 알 수 없으므로: 관절이동으로
        #      정자세 place 에 도달해 기준 TCP 를 읽고 -> +Z 로 올렸다가 -> 같은 자세로 수직
        #      하강한다(마지막 접근을 항상 순수 수직/정자세로 보장). ----
        print '[Auto] -> set {} place: joint move to upright place pose'.format(sidx)
        try:
            move_to_validated_safe(rb, safe_gripped, 'gripped', 'set {} place'.format(sidx))
            place_tcp = approach_place_pose(rb, place_j, 'set {} place'.format(sidx))
        except Exception as e:
            print '[SAFETY-ABORT] place transition failed: {}'.format(e)
            send_json(conn, {"command": "quit"})
            return
        try:
            # 같은 자세를 두 번째로 방문한 것이므로 pre-measure 값과 일치해야 한다.
            # 어긋나면 Phase B 중에 큐브가 그리퍼 안에서 미끄러진 것이다 — 그 경우
            # set 전체의 앵커가 무효이므로 조용히 지나가지 않는다.
            measured_cc = get_cube_center()
            drift = max(abs(measured_cc[k] - set_cc[k]) for k in range(3))
            print '[Auto] measured cube center (tool4): ' + fmt6(measured_cc) + \
                  '  (pre-measure 대비 {:.2f}mm)'.format(drift)
            if drift > 2.0:
                print '[WARN] set {} 큐브 중심이 Phase B 중 {:.2f}mm 이동했다. ' \
                      '그립 미끄러짐 의심 — 이 set 의 B 프레임을 재검토할 것.'.format(sidx, drift)
            set_cc = measured_cc
        except Exception as e:
            print '[SAFETY-ABORT] get_cube_center() failed: {}'.format(e)
            send_json(conn, {"command": "quit"})
            return

        print '[Auto] gripper OPEN (release cube on floor)'
        gripper_open()
        if check_gripper() != ['0', '1', '0', '0']:
            print '[SAFETY-ABORT] gripper did not confirm OPEN state'
            send_json(conn, {"command": "quit"})
            return
        time.sleep(0.3)

        print '[Auto] -> +Z {:.0f}mm clearance'.format(TRANSIT_LIFT_Z_MM)
        try:
            cur = Position(*rb.getpos().pos2list()[:6])
            rb.line(cur.offset(dz=TRANSIT_LIFT_Z_MM))
            verify_robot_still(rb)
        except Exception as e:
            print '[SAFETY-ABORT] +Z clearance failed: {}'.format(e)
            send_json(conn, {"command": "quit"})
            return
        time.sleep(0.5)

        # ---- Phase A: placement. 큐브는 바닥, 그리퍼 카메라가 각 뷰포인트에서 촬영. ----
        if block_a:
            print '[Auto] --- Phase A: {} placement captures (cube on floor) ---'.format(len(block_a))
        for wi, wp in enumerate(block_a):
            r = _capture_at_pose(
                rb, conn, wp, sidx, place_j, set_cc,
                cube_gripped=False, capture_block='A_placement', grasp_id=grasp_id,
                confirm=confirm, safe_joints=safe_empty, safe_kind='empty',
                label='A {}/{}'.format(wi + 1, len(block_a)), retract=False)
            if r == 'success':
                success += 1
            elif r == 'skip':
                skipped += 1
            elif r == 'quit':
                send_json(conn, {"command": "quit"})
                return
            elif r == 'disconnect':
                return
            elif r == 'abort':
                send_json(conn, {"command": "quit"})
                return

        # ---- 다음 set 이 있으면: 큐브 재-그립 후 +Z transit lift 하고 이동 ----
        if si < n_sets - 1:
            print '[Auto] re-grip cube (+Z {:.0f}mm approach above)'.format(
                GRIP_APPROACH_Z_MM)
            try:
                move_to_validated_safe(rb, safe_empty, 'empty', 're-grip approach')
                approach_and_close_gripper(rb, place_j, place_tcp,
                                           above_joints=place_above_j)
                if check_gripper() != ['0', '0', '0', '1']:
                    raise RuntimeError('gripper did not confirm CLOSED state')
                verify_robot_still(rb)
            except Exception as e:
                print '[SAFETY-ABORT] re-grip transition failed: {}'.format(e)
                send_json(conn, {"command": "quit"})
                return
            time.sleep(0.3)
            # Lift +Z transit_lift_mm so the cube clears the floor during the
            # joint transit to the next set's place_joints.
            # ---- 다음 set 으로: +Z 올려 수평 이동한 뒤 같은 만큼 수직 하강.
            #      안전자세를 경유하지 않으므로 큐브는 이 높이로 워크스페이스를
            #      가로지른다. 테이블 위 장애물보다 높은지 dry-run 으로 확인할 것. ----
            nxt = by_set[sets_order[si + 1]][0]
            next_tcp = nxt.get('place_tcp')
            next_above_j = nxt.get('place_above_joints')
            try:
                cur = Position(*rb.getpos().pos2list()[:6])
                rb.line(cur.offset(dz=TRANSIT_LIFT_Z_MM))
                verify_robot_still(rb)
                time.sleep(0.2)
                if next_above_j is not None:
                    # 관절 이동으로 다음 set 상공까지 간 뒤 수직 하강한다.
                    arrived_tcp = move_above_then_descend(
                        rb, next_above_j,
                        'transit to set {}'.format(sets_order[si + 1]),
                        GRIP_APPROACH_Z_MM,
                        expect_joints=nxt['place_joints'])
                elif next_tcp is None:
                    # 구버전 waypoint: 수평 목표를 모르므로 안전자세 경유로 폴백한다.
                    print '[WARN] next set has no place_above_joints/place_tcp; via safe pose'
                    move_to_validated_safe(rb, safe_gripped, 'gripped', 'post-grip return')
                    arrived_tcp = None
                else:
                    tgt = [float(x) for x in next_tcp[:6]]
                    if PLACE_TCP_Z_MM is not None:
                        tgt[2] = PLACE_TCP_Z_MM
                    above = list(tgt)
                    above[2] += TRANSIT_LIFT_Z_MM
                    print '[Auto] transit to set {} at +Z {:.0f}mm, then straight down'.format(
                        sets_order[si + 1], TRANSIT_LIFT_Z_MM)
                    rb.line(Position(*above))
                    time.sleep(0.2)
                    rb.line(Position(*tgt))
                    verify_robot_still(rb)
                    arrived_tcp = tgt
            except Exception as e:
                print '[SAFETY-ABORT] transit to next set failed: {}'.format(e)
                send_json(conn, {"command": "quit"})
                return

    # ---- 마무리: 마지막 set 에 놓인 큐브를 집어 set0 으로 되돌리고 놓은 뒤 물러난다.
    #      다음 실행이 항상 set0 에서 시작할 수 있게 하려는 것이다. place_tcp 는 직전
    #      반복이 남긴 "마지막 set 의 place TCP" 이고, 목표는 sets_order[0] 의 것이다. ----
    first_tcp = by_set[sets_order[0]][0].get('place_tcp')
    first_above_j = by_set[sets_order[0]][0].get('place_above_joints')
    try:
        print ''
        print '[Auto] --- 마무리: 큐브를 set {} 으로 되돌린다 ---'.format(sets_order[0])
        move_to_validated_safe(rb, safe_empty, 'empty', 'final re-grip approach')
        # place_j / place_tcp / place_above_j 는 루프가 남긴 "마지막 set" 값이다.
        # 큐브가 지금 놓여 있는 곳이 바로 거기이므로 그대로 집는다.
        approach_and_close_gripper(rb, place_j, place_tcp,
                                   above_joints=place_above_j)
        if check_gripper() != ['0', '0', '0', '1']:
            raise RuntimeError('gripper did not confirm CLOSED state')
        verify_robot_still(rb)
        time.sleep(0.3)

        cur = Position(*rb.getpos().pos2list()[:6])
        rb.line(cur.offset(dz=TRANSIT_LIFT_Z_MM))
        verify_robot_still(rb)
        time.sleep(0.2)

        if first_above_j is not None:
            move_above_then_descend(
                rb, first_above_j, 'transit to set {}'.format(sets_order[0]),
                GRIP_APPROACH_Z_MM,
                expect_joints=by_set[sets_order[0]][0]['place_joints'])
        elif first_tcp is None:
            # place_tcp 가 없는 구버전 waypoint: 관절로 set0 place 에 접근한다.
            print '[WARN] set {} has no place_above_joints/place_tcp; joint approach'.format(
                sets_order[0])
            approach_place_pose(rb, by_set[sets_order[0]][0]['place_joints'],
                                'final set {}'.format(sets_order[0]))
        else:
            tgt = [float(x) for x in first_tcp[:6]]
            if PLACE_TCP_Z_MM is not None:
                tgt[2] = PLACE_TCP_Z_MM
            above = list(tgt)
            above[2] += TRANSIT_LIFT_Z_MM
            print '[Auto] transit to set {} at +Z {:.0f}mm, then straight down'.format(
                sets_order[0], TRANSIT_LIFT_Z_MM)
            rb.line(Position(*above))
            time.sleep(0.2)
            rb.line(Position(*tgt))
            verify_robot_still(rb)

        print '[Auto] gripper OPEN (큐브를 set {} 에 반납)'.format(sets_order[0])
        gripper_open()
        if check_gripper() != ['0', '1', '0', '0']:
            raise RuntimeError('gripper did not confirm OPEN state')
        time.sleep(0.3)

        print '[Auto] -> +Z {:.0f}mm 물러나고 종료'.format(FINAL_LIFT_Z_MM)
        retract_z(rb, FINAL_LIFT_Z_MM, 'final')
        verify_robot_still(rb)
    except Exception as e:
        # 마무리는 데이터에 영향이 없다. 여기서 죽어도 촬영 결과는 이미 저장돼 있으므로
        # 경고만 남기고 정상 종료 경로를 그대로 탄다.
        print '[WARN] 마무리(큐브 반납) 실패: {}'.format(e)
        print '       촬영 데이터는 이미 저장되었다. 큐브 위치를 눈으로 확인할 것.'
    send_json(conn, {"command": "quit"})
    print ''
    print '=========================================='
    print '  Multi-Set Auto Complete'
    print '  - success: {}/{}'.format(success, total_caps)
    print '  - skipped: {}'.format(skipped)
    print '=========================================='


# ── Main ──

def main():
    try:
        rbs = RobSys()
        rbs.open()

        global rb
        rb = i611Robot()
        Base()
        rb.open()
        IOinit(rb)

        m = MotionParam(jnt_speed=100, lin_speed=100, pose_speed=100,
                        overlap=0, acctime=0.8, dacctime=0.8)
        rb.motionparam(m)
        rb.override(100)

        rb.settool(1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        rb.settool(2, 0.0, 35.0, 330.0, 0.0, 0.0, 0.0)
        rb.settool(3, 0.0, 0.0, TOOL_GRIPPER_Z, 0.0, 0.0, 0.0)
        rb.settool(4, 0.0, 0.0, TOOL_CUBE_CENTER_Z, 0.0, 0.0, 0.0)
        rb.changetool(TOOL_BASE)
        rb.use_mt(True)
        
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind((HOST, PORT))
        s.listen(1)
        print "Server on port {}. Waiting...".format(PORT)

        conn, addr = s.accept()
        print "Client: {}".format(addr)

        # Auto mode
        if '--auto' in sys.argv:
            idx = sys.argv.index('--auto')
            auto_file = sys.argv[idx + 1] if idx + 1 < len(sys.argv) else 'capture_waypoints.json'
            auto_speed = 30
            if '--speed' in sys.argv:
                sidx = sys.argv.index('--speed')
                if sidx + 1 < len(sys.argv):
                    auto_speed = int(sys.argv[sidx + 1])
            try:
                run_auto_capture(rb, conn, auto_file, auto_speed)
            finally:
                try:
                    conn.close()
                    s.close()
                except Exception:
                    pass
            return

        # State
        capture_count = 0
        set_index = -1
        move_history = []
        home_pose = None
        home_joints = None
        set_cube_center = None
        last_place_joints = None
        waypoints = []
        # capture tagging: A_placement (cube released, method a) vs B_eyetohand
        # (cube gripped, robot sweeps, method b). gc(grip)/go(release) toggle gripped.
        capture_block = "A_placement"
        grasp_id = 0
        cube_gripped = False
        # teach 기록 리스트 (PC 로 전송해 PC 에만 저장; 로봇 로컬 파일 없음)
        capture_poses = []           # recpose: A 뷰포인트 풀
        capture_sets = []            # recset : 큐브 set 배치
        grip_poses = []              # recgrip: B 그립-스윕 포즈 풀

        print ''
        print '=========================================='
        print '  p <a>,<v> / j <a>,<v>  : rel move'
        print '  gotop x,y,z[,rz,ry,rx] : TCP abs move'
        print '  gotoj d1,d2,d3,d4,d5,d6: joint abs move'
        print '  show / speed <0-100>'
        print '  c: capture  set: save TCP+cube'
        print '  go: grip open(release)  gc: grip close(grip)'
        print '  block a|b : A=placement(method a) / B=eye-to-hand sweep(method b)'
        print '    (b)eye-to-hand: gc(grip cube) -> block b -> jog widely(z>=150mm,'
        print '     tilt>=30deg) -> c at each pose (cube must stay visible to fixed cams)'
        print '  undo [N|all|<axes>|set]  q: quit'
        print '  recpose | rp          -> A 촬영 뷰포인트 기록 (-> capture_poses.json)'
        print '  recgrip | rg          -> B 그립-스윕 포즈 기록 (-> grip_poses.json)'
        print '  recset  | rs          -> 큐브 set 배치 기록 (-> capture_sets.json)'
        print '    ( ...undo | ...list 지원 )'
        print '  start                 -> auto capture (PC sends waypoints)'
        print '  start <path> [speed]  -> auto capture (local file)'
        print '    (cube gripped + robot at safe_joints_gripped before start)'
        print '    (waypoint requires explicit safe_joints_empty/safe_joints_gripped)'
        print '    (각 pose 이동 후 c+Enter 확인 시 촬영; --noconfirm 로 전자동)'
        print '=========================================='
        print ''

        show_pose()

        while True:
            try:
                cmd = raw_input('> ').strip()
            except EOFError:
                break
            if not cmd:
                continue

            cl = cmd.lower()

            # Quit
            if cl == 'q':
                send_json(conn, {"command": "quit"})
                break

            # Start: multi-set auto capture.
            #   "start"         -> request waypoints from PC over socket
            #   "start <path>"  -> load local file (legacy/testing)
            #   "start <path> <speed>" or "start - <speed>" supported
            elif cl.startswith('start'):
                parts = cmd.split(None, 2)
                wp_file = None
                # 관절 기반 전환 후 첫 운용: 상공 접근/set 전환이 직선이 아니라
                # 관절 보간이라 중간 경로가 눈에 덜 익다. 기존 30 에서 30% 낮췄다.
                # 'start - <speed>' 로 매번 덮어쓸 수 있다.
                spd = 21
                if len(parts) >= 2 and parts[1] != '-':
                    wp_file = parts[1]
                if len(parts) >= 3:
                    try:
                        spd = int(parts[2])
                    except ValueError:
                        print '[ERROR] invalid speed: {}'.format(parts[2])
                        continue
                try:
                    run_auto_capture(rb, conn, wp_file, spd)
                except IOError as e:
                    print '[ERROR] cannot read {}: {}'.format(wp_file, e)
                    continue
                break

            # Show
            elif cl == 'show':
                show_pose()
                if home_pose is not None:
                    print '  [Set #{}] TCP:  {}'.format(set_index, fmt6(home_pose))
                if set_cube_center is not None:
                    print '  [Set #{}] Cube: [{:.1f}, {:.1f}, {:.1f}]'.format(
                        set_index, set_cube_center[0], set_cube_center[1], set_cube_center[2])

            # Speed
            elif cl.startswith('speed'):
                try:
                    spd = int(cmd.split()[1])
                    rb.override(spd)
                    print 'Speed: {}'.format(spd)
                except Exception:
                    print 'Usage: speed <0-100>'

            # Set
            elif cl == 'set':
                set_index += 1
                home_pose = get_tcp()
                home_joints = get_joints()
                set_cube_center = get_cube_center()
                move_history = []
                print ''
                print '*** Set #{} saved ***'.format(set_index)
                print '  TCP:    {}'.format(fmt6(home_pose))
                print '  Joints: [{:.2f}, {:.2f}, {:.2f}, {:.2f}, {:.2f}, {:.2f}]'.format(
                    home_joints[0], home_joints[1], home_joints[2],
                    home_joints[3], home_joints[4], home_joints[5])
                print '  Cube:   [{:.1f}, {:.1f}, {:.1f}] (offset={:.0f}mm)'.format(
                    set_cube_center[0], set_cube_center[1], set_cube_center[2],
                    CUBE_CENTER_OFFSET_Z)

            # Capture block toggle (a = placement / b = eye-to-hand sweep)
            elif cl.startswith('block'):
                parts = cl.split()
                if len(parts) >= 2 and parts[1] in ('a', 'b'):
                    capture_block = "B_eyetohand" if parts[1] == 'b' else "A_placement"
                    print 'capture_block = {} (grasp_id={}, cube_gripped={})'.format(
                        capture_block, grasp_id, cube_gripped)
                else:
                    print 'Usage: block a | block b   (current: {})'.format(capture_block)

            # Gripper
            elif cl == 'go':
                last_place_joints = get_joints()
                gripper_open()
                cube_gripped = False        # cube released on the table

            elif cl == 'gc':
                gripper_close()
                cube_gripped = True         # cube now rigidly held
                grasp_id += 1               # new grasp = new eye-to-hand target transform
                print '[grip] cube_gripped=True, grasp_id={}'.format(grasp_id)

            # Capture
            elif cl == 'c':
                status, tcp, cube_tcp = do_capture(
                    conn, capture_count, set_cube_center,
                    set_index if set_index >= 0 else None,
                    set_joints=home_joints, set_tcp=home_pose,
                    place_joints=last_place_joints,
                    cube_gripped=cube_gripped,
                    capture_block=capture_block,
                    grasp_id=grasp_id)
                if status is None:
                    break
                wp = {
                    "capture_index": capture_count,
                    "capture_joints": get_joints(),
                    "capture_tcp": tcp,
                    "cube_center_6dof": cube_tcp,
                }
                if last_place_joints is not None:
                    wp["place_joints"] = last_place_joints
                else:
                    print '  [WARN] go not called before capture'
                waypoints.append(wp)
                capture_count += 1

            # Undo
            elif cl.startswith('undo'):
                args = cl.split()[1:]

                if args == ['set']:
                    if home_pose is None:
                        print 'No set saved.'
                    else:
                        target = Position(home_pose[0], home_pose[1], 0.0,
                                          home_pose[3], home_pose[4], home_pose[5])
                        rb.line(target)
                        move_history = []
                        show_pose()

                elif not move_history:
                    print 'Nothing to undo.'

                else:
                    if not args:
                        undo_one(move_history.pop())

                    elif args[0] == 'all':
                        while move_history:
                            undo_one(move_history.pop())

                    elif args[0] in VALID_AXES:
                        axis_set = set(a for a in args if a in VALID_AXES)
                        indices = [i for i, h in enumerate(move_history) if h[1] in axis_set]
                        if not indices:
                            print 'No moves on [{}]'.format(','.join(sorted(axis_set)))
                        else:
                            for idx in reversed(indices):
                                undo_one(move_history.pop(idx))
                    else:
                        try:
                            count = min(int(args[0]), len(move_history))
                        except ValueError:
                            print 'Usage: undo [N|all|<axes>|set]'
                            continue
                        for _ in range(count):
                            undo_one(move_history.pop())

                    show_pose()

            # Goto - joint absolute move
            elif cl.startswith('gotoj '):
                try:
                    vals = [float(v.strip()) for v in cmd[6:].strip().split(',')]
                    if len(vals) != 6:
                        print 'Usage: gotoj d1,d2,d3,d4,d5,d6'
                        continue
                    rb.move(Joint(*vals))
                    show_pose()
                except Exception as e:
                    print 'Error: {}'.format(e)

            # Goto - TCP absolute move (gotop / 기존 goto는 별칭으로 호환 유지)
            elif cl.startswith('gotop ') or cl.startswith('goto '):
                try:
                    rest = cmd[6:] if cl.startswith('gotop ') else cmd[5:]
                    vals = [float(v.strip()) for v in rest.strip().split(',')]
                    if len(vals) == 6:
                        rb.line(Position(*vals))
                    elif len(vals) == 3:
                        tcp = get_tcp()
                        rb.line(Position(vals[0], vals[1], vals[2], tcp[3], tcp[4], tcp[5]))
                    else:
                        print 'Usage: gotop x,y,z[,rz,ry,rx]'
                        continue
                    show_pose()
                except Exception as e:
                    print 'Error: {}'.format(e)

            # TCP move
            elif cl.startswith('p '):
                try:
                    parts = cmd[2:].strip().split(',')
                    axis, value = parts[0].strip(), float(parts[1].strip())
                    move_tcp(axis, value)
                    move_history.append(('p', axis, value))
                    show_pose()
                except Exception as e:
                    print 'Error: {}. Usage: p <axis>,<value>'.format(e)

            # Joint move
            elif cl.startswith('j '):
                try:
                    parts = cmd[2:].strip().split(',')
                    axis, value = parts[0].strip(), float(parts[1].strip())
                    move_joint(axis, value)
                    move_history.append(('j', axis, value))
                    show_pose()
                except Exception as e:
                    print 'Error: {}. Usage: j <axis>,<value>'.format(e)

            # Record capture viewpoint pose (teach pool for waypoint generation)
            #   recpose | rp          -> 현재 포즈를 풀에 기록 + capture_poses.json 저장
            #   recpose undo | rp undo-> 마지막 기록 취소
            #   recpose list | rp list-> 기록된 포즈 목록
            elif cl == 'recpose' or cl == 'rp' or cl.startswith('recpose ') or cl.startswith('rp '):
                parts = cl.split()
                sub = parts[1] if len(parts) >= 2 else None
                if sub == 'undo':
                    if capture_poses:
                        capture_poses.pop()
                        # pose_index 재부여(0..N-1 유지)
                        for i, p in enumerate(capture_poses):
                            p['pose_index'] = i
                        send_teach(conn, 'pose', capture_poses)
                        print '[recpose] undo -> {} poses (sent to PC)'.format(len(capture_poses))
                    else:
                        print '[recpose] nothing to undo'
                elif sub == 'list':
                    print '[recpose] {} poses recorded:'.format(len(capture_poses))
                    for p in capture_poses:
                        print '  #{} joints={}'.format(p['pose_index'], fmt6(p['capture_joints']))
                else:
                    pose = {
                        "pose_index": len(capture_poses),
                        "capture_joints": get_joints(),
                        "capture_tcp": get_tcp(),
                        "cube_center_6dof": get_cube_center(),
                    }
                    capture_poses.append(pose)
                    send_teach(conn, 'pose', capture_poses)
                    print ''
                    print '[recpose] #{} saved'.format(pose['pose_index'])
                    print '  joints: {}'.format(fmt6(pose['capture_joints']))
                    print '  tcp:    {}'.format(fmt6(pose['capture_tcp']))
                    print '  cube:   {}'.format(fmt6(pose['cube_center_6dof']))
                    print '  -> sent to PC ({} poses total)'.format(len(capture_poses))

            # Record grip-sweep (eye-to-hand, block B) pose for waypoint gen.
            #   recgrip | rg           -> 현재(큐브 그립 상태) 스윕 포즈 기록
            #   recgrip undo | rg undo -> 마지막 취소
            #   recgrip list | rg list -> 목록
            elif cl == 'recgrip' or cl == 'rg' or cl.startswith('recgrip ') or cl.startswith('rg '):
                parts = cl.split()
                sub = parts[1] if len(parts) >= 2 else None
                if sub == 'undo':
                    if grip_poses:
                        grip_poses.pop()
                        for i, p in enumerate(grip_poses):
                            p['pose_index'] = i
                        send_teach(conn, 'grip', grip_poses)
                        print '[recgrip] undo -> {} poses (sent to PC)'.format(len(grip_poses))
                    else:
                        print '[recgrip] nothing to undo'
                elif sub == 'list':
                    print '[recgrip] {} grip-sweep poses recorded:'.format(len(grip_poses))
                    for p in grip_poses:
                        print '  #{} joints={}'.format(p['pose_index'], fmt6(p['capture_joints']))
                else:
                    pose = {
                        "pose_index": len(grip_poses),
                        "capture_joints": get_joints(),
                        "capture_tcp": get_tcp(),
                        "cube_center_6dof": get_cube_center(),
                    }
                    grip_poses.append(pose)
                    send_teach(conn, 'grip', grip_poses)
                    print ''
                    print '[recgrip] #{} saved (grip-sweep, block B)'.format(pose['pose_index'])
                    print '  joints: {}'.format(fmt6(pose['capture_joints']))
                    print '  tcp:    {}'.format(fmt6(pose['capture_tcp']))
                    print '  cube:   {}'.format(fmt6(pose['cube_center_6dof']))
                    print '  -> sent to PC ({} poses total)'.format(len(grip_poses))

            # Record cube set placement (place_joints + cube center) for waypoint gen.
            #   recset | rs           -> 현재(큐브 그립+바닥에 놓은 상태) 기록
            #   recset undo | rs undo -> 마지막 취소
            #   recset list | rs list -> 목록
            elif cl == 'recset' or cl == 'rs' or cl.startswith('recset ') or cl.startswith('rs '):
                parts = cl.split()
                sub = parts[1] if len(parts) >= 2 else None
                if sub == 'undo':
                    if capture_sets:
                        capture_sets.pop()
                        for i, sset in enumerate(capture_sets):
                            sset['set_index'] = i
                        send_teach(conn, 'set', capture_sets)
                        print '[recset] undo -> {} sets (sent to PC)'.format(len(capture_sets))
                    else:
                        print '[recset] nothing to undo'
                elif sub == 'list':
                    print '[recset] {} sets recorded:'.format(len(capture_sets))
                    for sset in capture_sets:
                        print '  set#{} cube={}'.format(sset['set_index'], fmt6(sset['set_cube_center_6dof']))
                else:
                    sset = {
                        "set_index": len(capture_sets),
                        "place_joints": get_joints(),
                        "place_tcp": get_tcp(),
                        "set_cube_center_6dof": get_cube_center(),
                    }
                    capture_sets.append(sset)
                    send_teach(conn, 'set', capture_sets)
                    print ''
                    print '[recset] set#{} saved'.format(sset['set_index'])
                    print '  place_joints: {}'.format(fmt6(sset['place_joints']))
                    print '  place_tcp:    {}'.format(fmt6(sset['place_tcp']))
                    print '  cube_center:  {}'.format(fmt6(sset['set_cube_center_6dof']))
                    print '  -> sent to PC ({} sets total)'.format(len(capture_sets))

            else:
                print 'Unknown: {}'.format(cmd)

        # Save waypoints
        if waypoints:
            save_data = {
                "set_joints": home_joints,
                "set_tcp": home_pose,
                "set_cube_center": set_cube_center,
                "waypoints": waypoints,
            }
            with open('capture_waypoints.json', 'w') as f:
                json.dump(save_data, f, indent=2)
            print '\nWaypoints saved: {} poses'.format(len(waypoints))

        print '\nTotal captures: {}'.format(capture_count)

    except KeyboardInterrupt:
        print '\nInterrupted'
        try:
            send_json(conn, {"command": "quit"})
        except Exception:
            pass
    except Robot_emo as e:
        print(e)
    except Robot_error as e:
        print(e)
    except Robot_fatalerror as e:
        print(e)
    except Exception as e:
        print(e)
    finally:
        try:
            rb.exit(0)
            rb.close()
            rbs.close()
        except Exception:
            pass
        try:
            s.close()
        except Exception:
            pass


if __name__ == '__main__':
    main()
