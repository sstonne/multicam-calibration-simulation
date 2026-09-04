#!/usr/bin/python
# -*- coding: utf-8 -*-
u"""ZEUS 로봇측 Shah(AX=YB) 촬영 서버 — 보드가 플랜지에 볼트 고정된 리그 전용.

  로봇: 플랜지에 ChArUco 보드(7x5 / square 17mm / marker 12mm)가 강체 고정
  카메라: 테이블에 고정된 RealSense 3대 (cam0/cam1/cam3), 손목 카메라 없음
  → 전부 eye-to-hand.  T_base_gripper(k) @ X == Y_i @ T_cam_i_board(k)
      X   = T_gripper_board  (보드가 플랜지 어디에 붙었나 — 모든 카메라 공통)
      Y_i = T_base_cam_i     (카메라가 base 어디에 있나 — 주 출력)

이 파일은 ZEUS 컨트롤러(Python 2.7)에서 실행한다. scp 로 로봇에 올린 뒤
SSH 로 접속해 실행하라. 로봇에는 numpy 가 없으므로 순수 python 만 쓴다.

i611usr/c1.py (= robot_calb/c2.py = jiwoo server/c1.py) 와 다른 점:
  * 그리퍼/큐브 코드 없음. 이 리그에는 그리퍼가 없으므로 IO 48번을 건드리지 않는다.
    (c1.py 를 그대로 쓰면 gripper_close() 가 5초 타임아웃 후 경고만 찍고 조용히
     잘못된 상태로 진행한다.)
  * changetool(1) 고정. 보드 오프셋은 Shah 의 X 가 추정하므로 tool 로 넣지 않는다.
    tool 3(150mm)을 쓰면 보드 오프셋이 이중으로 들어가 결과가 어긋난다.
  * c1.py 의 get_cube_center() 는 changetool(4) 후 3 으로 되돌려서, 첫 촬영만
    tool 1 이고 이후는 tool 3 이 되는 누수가 있었다. 여기엔 changetool 이 한 번뿐이다.

촬영 자세 지침 (시뮬레이션으로 측정한 값):
  포즈 간 평균 상대회전이 정확도를 지배한다. 5deg -> 95mm, 40deg -> 5.4mm.
  포즈 수 증가는 효과가 훨씬 작다 (6개 9.7mm -> 30개 3.7mm).
  ==> 평행이동만 하지 말고 크게 기울여라. 목표: 평균 상대회전 40deg 이상, 20 포즈.
  이 서버는 기록할 때마다 현재 평균 상대회전을 실시간으로 표시한다.

실행:
  python shah_capture_server.py            # 대화형
  python shah_capture_server.py --auto 20  # 기록된 포즈 자동 순회

PC 측 상대 스크립트: capture/shah_capture_client.py
전체 절차와 배경: docs/real_shah_capture.md
"""

from i611_MCS import *
from teachdata import *
from i611_extend import *
from rbsys import *
from i611_common import *
from i611_io import *
from i611shm import *

import sys
import math
import time
import socket
import json

HOST = '0.0.0.0'
PORT = 12348

# 포즈 간 평균 상대회전 목표치 (deg). 이 값을 넘겨야 Shah 가 잘 조건화된다.
ROTATION_DIVERSITY_TARGET_DEG = 40.0
# 자동 순회 시 기본 속도 (override %)
DEFAULT_SPEED = 30

TCP_AXIS_MAP = {'x': 'dx', 'y': 'dy', 'z': 'dz', 'rz': 'drz', 'ry': 'dry', 'rx': 'drx'}
JOINT_AXIS_MAP = {'d1': 'dj1', 'd2': 'dj2', 'd3': 'dj3',
                  'd4': 'dj4', 'd5': 'dj5', 'd6': 'dj6'}


# ──────────────────────────────────────────────────────────────
# 소켓 (newline-delimited JSON — PC 측과 동일 규약)
# ──────────────────────────────────────────────────────────────

def send_json(conn, obj):
    try:
        conn.sendall((json.dumps(obj) + '\n').encode('utf-8'))
        return True
    except socket.error as e:
        print '[ERROR] send: {}'.format(e)
        return False


def recv_json(conn):
    try:
        data = conn.recv(8192).decode('utf-8')
        if not data:
            return None
        for line in data.splitlines():
            line = line.strip()
            if line:
                return json.loads(line)
    except Exception as e:
        print '[ERROR] recv: {}'.format(e)
    return None


# ──────────────────────────────────────────────────────────────
# 로봇 상태
# ──────────────────────────────────────────────────────────────

def get_tcp():
    u"""[x_mm, y_mm, z_mm, rz_deg, ry_deg, rx_deg] — tool 1(플랜지) 기준."""
    return rb.getpos().pos2list()[:6]


def get_joints():
    return rb.getjnt().jnt2list()[:6]


def fmt6(v):
    return '[' + ', '.join('{:.2f}'.format(x) for x in v) + ']'


# ──────────────────────────────────────────────────────────────
# 회전 다양성 계산 (numpy 없이 순수 python — 컨트롤러에 numpy 가 없다)
# ZEUS 포즈는 [x,y,z,rz,ry,rx], 회전은 Rz @ Ry @ Rx (PC 측
# robot_comm.euler_deg_to_matrix 와 반드시 동일한 규약이어야 한다).
# ──────────────────────────────────────────────────────────────

def _matmul3(A, B):
    return [[sum(A[i][k] * B[k][j] for k in range(3)) for j in range(3)]
            for i in range(3)]


def pose_to_rotation(pose):
    rz, ry, rx = [math.radians(a) for a in pose[3:6]]
    Rz = [[math.cos(rz), -math.sin(rz), 0.0],
          [math.sin(rz),  math.cos(rz), 0.0],
          [0.0, 0.0, 1.0]]
    Ry = [[math.cos(ry), 0.0, math.sin(ry)],
          [0.0, 1.0, 0.0],
          [-math.sin(ry), 0.0, math.cos(ry)]]
    Rx = [[1.0, 0.0, 0.0],
          [0.0, math.cos(rx), -math.sin(rx)],
          [0.0, math.sin(rx),  math.cos(rx)]]
    return _matmul3(_matmul3(Rz, Ry), Rx)


def relative_angle_deg(R1, R2):
    u"""두 회전 사이의 각도. trace(R1^T R2) = 1 + 2cos(theta)."""
    trace = sum(sum(R1[k][i] * R2[k][j] for k in range(3))
                for i, j in ((0, 0), (1, 1), (2, 2)))
    cos_theta = (trace - 1.0) / 2.0
    return math.degrees(math.acos(max(-1.0, min(1.0, cos_theta))))


def rotation_diversity_deg(poses):
    u"""모든 포즈쌍 상대회전 각도의 평균. Shah 조건화의 대리 지표."""
    if len(poses) < 2:
        return 0.0
    rotations = [pose_to_rotation(p) for p in poses]
    angles = []
    for i in range(len(rotations)):
        for j in range(i + 1, len(rotations)):
            angles.append(relative_angle_deg(rotations[i], rotations[j]))
    return sum(angles) / len(angles)


def report_diversity(poses):
    value = rotation_diversity_deg(poses)
    if value >= ROTATION_DIVERSITY_TARGET_DEG:
        verdict = 'OK'
    elif value >= ROTATION_DIVERSITY_TARGET_DEG * 0.6:
        verdict = '부족 — 더 기울여라'
    else:
        verdict = '심각히 부족 — 평행이동만 하고 있다'
    print '  포즈 {}개 / 평균 상대회전 {:.1f}deg (목표 {:.0f}deg) -> {}'.format(
        len(poses), value, ROTATION_DIVERSITY_TARGET_DEG, verdict)
    return value


# ──────────────────────────────────────────────────────────────
# 이동
# ──────────────────────────────────────────────────────────────

def move_tcp(axis, value):
    key = TCP_AXIS_MAP.get(axis)
    if key is None:
        print '[ERROR] unknown TCP axis: {}'.format(axis)
        return False
    rb.relline(**{key: float(value)})
    return True


def move_joint(axis, value):
    key = JOINT_AXIS_MAP.get(axis)
    if key is None:
        print '[ERROR] unknown joint axis: {}'.format(axis)
        return False
    rb.reljntmove(**{key: float(value)})
    return True


def show_pose():
    print '  joints: {}'.format(fmt6(get_joints()))
    print '  flange: {}'.format(fmt6(get_tcp()))


# ──────────────────────────────────────────────────────────────
# 촬영
# ──────────────────────────────────────────────────────────────

def do_capture(conn, index):
    u"""로봇 자세를 먼저 읽고 PC 에 촬영을 요청한 뒤 결과를 기다린다.

    포즈와 이미지가 어긋나지 않도록 반드시 정지 상태에서 호출한다.
    반환: (status, pose) — status 'saved' 만 유효 데이터.
    """
    pose = get_tcp()
    joints = get_joints()
    print ''
    print '*** CAPTURE {} ***'.format(index)
    print '  flange: {}'.format(fmt6(pose))

    ok = send_json(conn, {
        "command": "capture",
        "index": int(index),
        "flange_pose_6dof": pose,       # [x,y,z mm, rz,ry,rx deg], tool 1
        "joints_6dof": joints,
        "tool": 1,
    })
    if not ok:
        return None, pose

    response = recv_json(conn)
    if response is None:
        print '[ERROR] PC 연결 끊김'
        return None, pose

    status = response.get('status', 'unknown')
    detail = response.get('detail', '')
    corners = response.get('corners_per_camera', {})
    print '  -> status={} {}'.format(status, detail)
    if corners:
        print '  -> corners: {}'.format(
            ', '.join('{}={}'.format(k, corners[k]) for k in sorted(corners)))
    return status, pose


# ──────────────────────────────────────────────────────────────
# 자동 순회
# ──────────────────────────────────────────────────────────────

def run_auto(conn, waypoints, speed=DEFAULT_SPEED, confirm=True):
    if not waypoints:
        print '[ERROR] 기록된 포즈가 없다. rec 으로 먼저 티칭하라.'
        return
    print ''
    print '=== 자동 촬영 시작: {}개 포즈, speed={} ==='.format(len(waypoints), speed)
    report_diversity([w['pose'] for w in waypoints])
    rb.override(int(speed))

    captured = []
    for index, waypoint in enumerate(waypoints):
        joints = waypoint['joints']
        print ''
        print '--- [{}/{}] 이동 중 ---'.format(index + 1, len(waypoints))
        try:
            rb.move(Joint(joints[0], joints[1], joints[2],
                          joints[3], joints[4], joints[5]))
        except Exception as e:
            print '[ERROR] 이동 실패, 건너뜀: {}'.format(e)
            continue
        time.sleep(0.4)          # 잔진동이 가라앉은 뒤 촬영

        if confirm:
            try:
                answer = raw_input('  촬영? [Enter=예 / s=건너뜀 / q=중단] ').strip().lower()
            except EOFError:
                answer = 'q'
            if answer == 'q':
                print '중단'
                break
            if answer == 's':
                continue

        status, pose = do_capture(conn, len(captured))
        if status is None:
            break
        if status == 'saved':
            captured.append(pose)
        else:
            print '  [WARN] 저장되지 않음 — 이 포즈는 데이터에 포함되지 않는다'

    print ''
    print '=== 자동 촬영 종료: {}개 저장 ==='.format(len(captured))
    if captured:
        report_diversity(captured)
    send_json(conn, {"command": "session_end", "saved": len(captured)})


# ──────────────────────────────────────────────────────────────

MENU = u"""
==================================================================
  Shah 촬영 서버 — 보드가 플랜지에 고정된 eye-to-hand 리그
------------------------------------------------------------------
  p <axis>,<v>   TCP 상대이동   axis: x y z rz ry rx   (mm / deg)
  j <axis>,<v>   관절 상대이동  axis: d1..d6           (deg)
  show           현재 자세
  speed <0-100>  override

  c              지금 자세로 촬영 (PC 에 요청)
  rec            지금 자세를 촬영 포즈로 기록
  list           기록된 포즈 + 회전 다양성
  undo           마지막 기록 취소
  start [speed]  기록된 포즈 자동 순회하며 촬영
  q              종료

  ※ 평행이동만 하지 말 것. 매 포즈마다 rz/ry/rx 를 20~30deg 씩 바꿔라.
     rec 할 때마다 평균 상대회전이 표시된다. {:.0f}deg 이상이 목표.
==================================================================
""".format(ROTATION_DIVERSITY_TARGET_DEG)


def interactive(conn):
    waypoints = []
    capture_index = 0
    print MENU
    show_pose()

    while True:
        try:
            command = raw_input('> ').strip()
        except EOFError:
            break
        if not command:
            continue
        lowered = command.lower()

        if lowered == 'q':
            send_json(conn, {"command": "quit"})
            break

        elif lowered == 'show':
            show_pose()

        elif lowered.startswith('speed'):
            parts = command.split()
            if len(parts) >= 2:
                try:
                    rb.override(int(parts[1]))
                    print '  override={}'.format(int(parts[1]))
                except ValueError:
                    print '[ERROR] speed 값이 잘못됐다'

        elif lowered.startswith('p ') or lowered.startswith('j '):
            kind = lowered[0]
            try:
                axis, value = command[2:].split(',')
                axis = axis.strip().lower()
                value = float(value)
            except ValueError:
                print '[ERROR] 형식: p x,10  /  j d1,5'
                continue
            moved = move_tcp(axis, value) if kind == 'p' else move_joint(axis, value)
            if moved:
                show_pose()

        elif lowered == 'c':
            status, _ = do_capture(conn, capture_index)
            if status is None:
                break
            if status == 'saved':
                capture_index += 1

        elif lowered == 'rec':
            waypoints.append({'pose': get_tcp(), 'joints': get_joints()})
            print '  기록됨.'
            report_diversity([w['pose'] for w in waypoints])

        elif lowered == 'list':
            for index, waypoint in enumerate(waypoints):
                print '  [{}] {}'.format(index, fmt6(waypoint['pose']))
            report_diversity([w['pose'] for w in waypoints])

        elif lowered == 'undo':
            if waypoints:
                waypoints.pop()
                print '  취소됨.'
                report_diversity([w['pose'] for w in waypoints])
            else:
                print '  기록이 비어 있다.'

        elif lowered.startswith('start'):
            parts = command.split()
            speed = DEFAULT_SPEED
            if len(parts) >= 2:
                try:
                    speed = int(parts[1])
                except ValueError:
                    print '[ERROR] speed 값이 잘못됐다'
                    continue
            run_auto(conn, waypoints, speed)
            break

        else:
            print '[ERROR] 알 수 없는 명령: {}'.format(command)


def main():
    global rb, rbs
    rbs = RobSys()
    rbs.open()
    rb = i611Robot()
    Base()
    rb.open()
    IOinit(rb)

    rb.motionparam(MotionParam(jnt_speed=70, lin_speed=50, pose_speed=50,
                               overlap=0, acctime=1.0, dacctime=1.0))
    rb.override(30)

    # tool 1 = 플랜지 원점. 보드 오프셋은 Shah 의 X 가 추정하므로 여기서 넣지 않는다.
    rb.settool(1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    rb.changetool(1)
    rb.use_mt(True)

    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind((HOST, PORT))
    server.listen(1)
    print 'Shah capture server on port {}. PC 클라이언트를 기다리는 중...'.format(PORT)
    conn, address = server.accept()
    print 'PC 연결됨: {}'.format(address)

    try:
        interactive(conn)
    finally:
        try:
            conn.close()
            server.close()
        except Exception:
            pass


if __name__ == '__main__':
    try:
        main()
    except Robot_emo as e:
        print 'EMO: {}'.format(e)
        rb.exit(0)
        rbs.cmd_reset()
    except (Robot_error, Robot_fatalerror) as e:
        print 'Robot error: {}'.format(e)
        rb.exit(0)
        rbs.cmd_reset()
    except KeyboardInterrupt:
        print 'Interrupted'
    finally:
        try:
            rb.close()
            rbs.close()
            rb.exit(0)
        except Exception:
            pass
