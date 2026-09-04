#!/usr/bin/python
# -*- coding: utf-8 -*-
u"""ZEUS 로봇측 자세 응답 서버 — PC 가 물을 때마다 현재 자세를 돌려준다.

PC 측 상대 스크립트: capture/record_dataset.py

왜 따로 있는가
  capture/robot/shah_capture_server.py 는 로봇 터미널이 주도하는 구조다.
  raw_input 에서 블로킹하며 사람이 로봇 콘솔에서 명령을 치고, 촬영 시작도
  로봇이 보낸다. 그래서 PC 가 "지금 자세 알려 달라"고 물을 수 없다.

  이 파일은 반대다. 아무것도 주도하지 않고 요청에만 답한다. PC 터미널에서
  엔터를 칠 때마다 자세를 한 번씩 읽어 가는 용도다. 로봇 콘솔은 로그만 찍는다.
  촬영·검출·저장은 전부 PC 가 한다 (ZEUS 에는 카메라도 numpy 도 없다).

  포트는 12350 을 쓴다. 이 저장소에서 이미 쓰이는 포트를 피한 것이다.
    12344/12345  i611 SDK 내부 (rblib.py, i611_MCS.py)
    12346        calibration_server.py, server_comm.py, robot_env_comm.py 계열
    12348        shah_capture_server.py, handeye_server.py, sam3d_calb/robot_pose_server.py
    12349        model.py (sim-to-real 15Hz 스트리밍) — 여기와 겹치면 bind 실패한다
  어느 쪽이든 로봇 서버를 두 개 동시에 띄우지는 마라. 같은 로봇을 두 프로그램이 잡는다.

기존 sam3d_calb/robot_pose_server.py 와의 관계
  그 파일도 PC 가 묻고 로봇이 답하는 같은 구조다. 다만 큐브 리그 전용이라
  changetool(3)(그리퍼 150mm) 상태의 TCP 를 돌려주므로, 이 리그에서 그대로 쓰면
  보드 오프셋이 이중으로 들어간다. 명령 이름은 그쪽에 맞춰 두었다 —
  get_pose / ping / notify_saved / quit 를 모두 받고, 응답에도 그쪽 필드
  이름(tcp_6dof, joint_6dof)을 함께 넣는다. 그래서 PC 측 도구를 서로 바꿔 써도 된다.

규약 (PC 측과 반드시 같아야 한다 — docs/real_shah_capture.md §8.2, §8.3)
  * 자세는 tool 1(플랜지) 기준 [x, y, z mm, rz, ry, rx deg]
  * 보드 오프셋은 Shah 의 X 가 추정하므로 tool 로 넣지 않는다.
    tool 3(150mm) 등을 쓰면 오프셋이 이중으로 들어가 결과가 어긋난다.
  * 그리퍼/큐브 코드 없음. 이 리그에는 그리퍼가 없으므로 IO 48번을 건드리지 않는다.

프로토콜 (newline-delimited JSON, 요청 1개에 응답 1개)
  -> {"command": "get_state"}   (get_pose 도 같은 뜻으로 받는다)
  <- {"status": "ok", "flange_pose_6dof": [...6], "joints_6dof": [...6], "tool": 1,
      "tcp_6dof": [...6], "joint_6dof": [...6]}      뒤 두 개는 기존 서버와 같은 별칭

  -> {"command": "jog", "space": "tcp"|"joint", "axis": "x", "value": 10.0}
  <- {"status": "ok", "flange_pose_6dof": [...], "joints_6dof": [...], "tool": 1}

  -> {"command": "speed", "value": 30}      <- {"status": "ok", "override": 30}
  -> {"command": "ping"}                    <- {"status": "ok"}
  -> {"command": "notify_saved", ...}       PC 가 저장을 알릴 때. 로봇 콘솔에 찍기만 한다
  -> {"command": "bye"}                     연결만 끊고 다음 PC 를 기다린다
  -> {"command": "quit"}                    서버 종료
  실패하면 어떤 명령이든 {"status": "error", "detail": "...", "reason": "..."}
  (detail 과 reason 은 같은 문자열. reason 은 기존 서버와 맞추기 위한 별칭이다)

  PC 가 끊겨도 서버는 죽지 않고 다시 accept 로 돌아간다. PC 쪽 스크립트를
  다시 실행해도 로봇 프로그램은 그대로 두면 된다.

이 파일은 Python 2.7 전용이다 (ZEUS 컨트롤러의 파이썬).
  * 첫 줄의 coding 선언을 지우면 로봇에서 컴파일조차 되지 않는다 —
    한글 주석이 있는 파일은 Python 2 에서 이 선언이 필수다.
  * VS Code 등 Python 3 도구는 print 문을 전부 오류로 표시한다. 정상이다.
    이 파일을 PC 에서 실행하거나 문법 검사하지 마라. i611usr/ 의 다른
    로봇 스크립트들도 전부 같다.
  * 편집은 줄바꿈을 LF 로 유지하라 (CRLF 면 ./pose_query_server.py 실행이 깨진다).

실행
  scp capture/robot/pose_query_server.py zeus:~/i611usr/
  ssh zeus 'cd i611usr && python pose_query_server.py'
"""

from i611_MCS import *
from teachdata import *
from i611_extend import *
from rbsys import *
from i611_common import *
from i611_io import *
from i611shm import *

import json
import socket

HOST = '0.0.0.0'
# 12346/12348 은 기존 서버들이, 12349 는 model.py 가 이미 쓴다. 위 docstring 참조.
PORT = 12350

DEFAULT_OVERRIDE = 20        # jog 는 사람이 옆에서 보고 있으므로 느리게 시작한다

# jog 안전 한계. 오타 하나로 팔이 크게 날아가는 것을 막는다.
MAX_JOG_MM = 100.0
MAX_JOG_DEG = 45.0

TCP_AXIS_MAP = {'x': 'dx', 'y': 'dy', 'z': 'dz', 'rz': 'drz', 'ry': 'dry', 'rx': 'drx'}
JOINT_AXIS_MAP = {'d1': 'dj1', 'd2': 'dj2', 'd3': 'dj3',
                  'd4': 'dj4', 'd5': 'dj5', 'd6': 'dj6'}
ROTATION_AXES = ('rz', 'ry', 'rx')


# ──────────────────────────────────────────────────────────────
# 로봇 상태
# ──────────────────────────────────────────────────────────────

def get_tcp():
    u"""[x_mm, y_mm, z_mm, rz_deg, ry_deg, rx_deg] — tool 1(플랜지) 기준."""
    return rb.getpos().pos2list()[:6]


def get_joints():
    return rb.getjnt().jnt2list()[:6]


def state_payload():
    u"""자세 응답. 필드 이름을 두 벌 담는다.

    flange_pose_6dof / joints_6dof 는 이 리그(record_dataset.py)의 이름이고,
    tcp_6dof / joint_6dof 는 sam3d_calb/robot_pose_server.py 가 쓰던 이름이다.
    값은 같다. 어느 PC 스크립트가 붙어도 읽을 수 있게 둘 다 넣는다.
    """
    tcp = get_tcp()
    joints = get_joints()
    return {
        "status": "ok",
        "flange_pose_6dof": tcp,
        "joints_6dof": joints,
        "tcp_6dof": tcp,
        "joint_6dof": joints,
        "tool": 1,
    }


def error(detail):
    u"""detail 은 이 서버의 이름, reason 은 기존 서버의 이름. 같은 값을 넣는다."""
    return {"status": "error", "detail": detail, "reason": detail}


def fmt6(values):
    return '[' + ', '.join('{:.2f}'.format(v) for v in values) + ']'


# ──────────────────────────────────────────────────────────────
# 명령 처리
# ──────────────────────────────────────────────────────────────

def do_jog(request):
    space = request.get('space', 'tcp')
    axis = str(request.get('axis', '')).lower()
    try:
        value = float(request.get('value'))
    except (TypeError, ValueError):
        return error("value 가 숫자가 아니다")

    if space == 'tcp':
        key = TCP_AXIS_MAP.get(axis)
        if key is None:
            return error("알 수 없는 TCP 축: {}".format(axis))
        limit = MAX_JOG_DEG if axis in ROTATION_AXES else MAX_JOG_MM
    elif space == 'joint':
        key = JOINT_AXIS_MAP.get(axis)
        if key is None:
            return error("알 수 없는 관절 축: {}".format(axis))
        limit = MAX_JOG_DEG
    else:
        return error("space 는 tcp 또는 joint")

    if abs(value) > limit:
        return error("이동량 {} 이 한계 {} 를 넘는다".format(value, limit))

    try:
        if space == 'tcp':
            rb.relline(**{key: value})
        else:
            rb.reljntmove(**{key: value})
    except Exception as e:
        return error("이동 실패: {}".format(e))

    print '  jog {} {} {:+.2f} -> {}'.format(space, axis, value, fmt6(get_tcp()))
    return state_payload()


def do_speed(request):
    try:
        value = int(request.get('value'))
    except (TypeError, ValueError):
        return error("value 가 정수가 아니다")
    if not 0 < value <= 100:
        return error("override 는 1~100")
    rb.override(value)
    print '  override={}'.format(value)
    return {"status": "ok", "override": value}


def handle(request):
    u"""요청 하나를 처리해 응답 dict 를 돌려준다. None 이면 연결을 끊는다."""
    command = request.get('command')

    # get_pose 는 sam3d_calb/robot_pose_server.py 가 쓰던 이름이다. 같이 받는다.
    if command in ('get_state', 'get_pose'):
        payload = state_payload()
        print '  get_state -> flange {} joints {}'.format(
            fmt6(payload['flange_pose_6dof']), fmt6(payload['joints_6dof']))
        return payload

    elif command == 'jog':
        return do_jog(request)

    elif command == 'speed':
        return do_speed(request)

    elif command == 'ping':
        return {"status": "ok"}

    elif command == 'notify_saved':
        # PC 가 저장을 알려 온 것. 로봇 콘솔에서 진행 상황을 보기 위한 것뿐이다.
        print '  [SAVED] event_id={} (총 {}건)'.format(
            request.get('event_id', '?'), request.get('n_captures', '?'))
        return {"status": "ok"}

    return error("알 수 없는 명령: {}".format(command))


# ──────────────────────────────────────────────────────────────
# 서버
# ──────────────────────────────────────────────────────────────

def serve_connection(conn):
    u"""한 PC 클라이언트를 담당한다. 반환 True 면 서버를 종료한다."""
    buffer = ''
    while True:
        try:
            chunk = conn.recv(8192).decode('utf-8')
        except socket.error as e:
            print '[ERROR] recv: {}'.format(e)
            return False
        if not chunk:
            print 'PC 연결 종료'
            return False

        buffer += chunk
        while '\n' in buffer:
            line, buffer = buffer.split('\n', 1)
            line = line.strip()
            if not line:
                continue

            try:
                request = json.loads(line)
            except ValueError as e:
                response = error("JSON 오류: {}".format(e))
                request = {}
            else:
                command = request.get('command')
                if command == 'bye':
                    print 'PC 가 연결을 닫았다'
                    return False
                if command == 'quit':
                    print '종료 요청'
                    try:
                        conn.sendall(json.dumps({"status": "ok"}) + '\n')
                    except socket.error:
                        pass
                    return True
                try:
                    response = handle(request)
                except Exception as e:
                    response = error(str(e))

            try:
                conn.sendall((json.dumps(response) + '\n').encode('utf-8'))
            except socket.error as e:
                print '[ERROR] send: {}'.format(e)
                return False


def main():
    global rb, rbs
    rbs = RobSys()
    rbs.open()
    rb = i611Robot()
    Base()
    rb.open()
    IOinit(rb)

    rb.motionparam(MotionParam(jnt_speed=30, lin_speed=30, pose_speed=30,
                               overlap=0, acctime=1.0, dacctime=1.0))
    rb.override(DEFAULT_OVERRIDE)

    # tool 1 = 플랜지 원점. 보드 오프셋은 Shah 의 X 가 추정하므로 넣지 않는다.
    rb.settool(1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    rb.changetool(1)
    rb.use_mt(True)

    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind((HOST, PORT))
    server.listen(1)

    print '=================================================================='
    print '  자세 응답 서버 (tool 1 / 플랜지 기준)'
    print '  포트 {} 에서 PC 를 기다린다.'.format(PORT)
    print '  PC:  python capture/record_dataset.py --robot-port {}'.format(PORT)
    print '  로봇은 펜던트로 움직여도 되고 PC 의 p/j 명령으로 움직여도 된다.'
    print '  Ctrl+C 로 종료.'
    print '=================================================================='
    print '현재 flange: {}'.format(fmt6(get_tcp()))
    print '현재 joints: {}'.format(fmt6(get_joints()))

    try:
        while True:
            conn, address = server.accept()
            print ''
            print 'PC 연결됨: {}'.format(address)
            try:
                should_quit = serve_connection(conn)
            finally:
                try:
                    conn.close()
                except socket.error:
                    pass
            if should_quit:
                break
            print '다음 PC 를 기다린다...'
    finally:
        try:
            server.close()
        except socket.error:
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
