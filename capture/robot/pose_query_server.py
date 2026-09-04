#!/usr/bin/python
# -*- coding: utf-8 -*-
u"""ZEUS 로봇측 자세 응답 서버 (shm-only, zeus_jog_onboard.py 와 공존).

PC 측 상대 스크립트: capture/record_dataset.py

이 파일은 Python 2.7 전용이다 (ZEUS 컨트롤러의 파이썬)
  * 편집은 줄바꿈을 LF 로 유지

왜 shm-only 인가 — 다른 조작 스크립트와 동시 실행 가능
  ZEUS 컨트롤러는 rb.open() 을 한 번만 허용함.
  이 파일은 rb.open() 을 부르지 않는다. 로봇 상태는 전부 공유 메모리에서
  읽는다. i611usr/output.py 가 zeus_pose_log.csv 를 15Hz 로 쓰는 방식이고
  (i611usr/output.py:33), example_6axis_pendant.py 의 관절/자세 읽기와 같다
  (i611usr/example_6axis_pendant.py:35, 57).

  따라서 zeus_jog_onboard.py 로 로봇을 계속 조작하면서, 이 서버는 옆에서 자세만
  받아 PC 에 넘겨준다. 조작은 로봇 콘솔에서, 촬영과 저장은 PC 에서 한다.

주의 - shm 이 주는 자세는 컨트롤러의 현재 tool 기준이다
  즉 zeus_jog_onboard.py 가 어떤 tool 을 설정해 두었는지에 달렸다. 이 리그의
  Shah 규약은 tool 1 (플랜지) 이다 (docs/real_shah_capture.md 8.2).
  그리퍼가 없으므로 zeus_jog_onboard.py 도 tool 1 일 확률이 높지만, 확실하지 않다면 로봇 콘솔에서 확인.

프로토콜 (newline-delimited JSON, 요청 1개에 응답 1개)
  -> {"command": "get_state"}   (get_pose 도 같은 뜻으로 받는다)
  <- {"status": "ok", "flange_pose_6dof": [...6], "joints_6dof": [...6],
      "tcp_6dof": [...6], "joint_6dof": [...6],
      "tool": null,          shm 만으로는 확정 불가. 위 주의 참조.
      "source": "shm"}       rb.open() 없이 공유 메모리에서 읽었음을 표시

  -> {"command": "ping"}                    <- {"status": "ok"}
  -> {"command": "notify_saved", ...}       PC 가 저장을 알릴 때. 콘솔에 찍기만 한다
  -> {"command": "jog", ...}                <- 거부 (에러에 이유 안내)
  -> {"command": "speed", ...}              <- 거부 (같은 이유)
  -> {"command": "bye"}                     연결만 끊고 다음 PC 를 기다린다
  -> {"command": "quit"}                    서버 종료
  실패하면 어떤 명령이든 {"status": "error", "detail": "...", "reason": "..."}

포트는 12350 을 쓴다. 저장소에서 이미 쓰이는 포트를 피한 것이다.
  12344/12345  i611 SDK 내부
  12346        calibration_server.py 계열
  12348        shah_capture_server.py, handeye_server.py, sam3d_calb/robot_pose_server.py
  12349        model.py (sim-to-real 15Hz 스트리밍)

실행
  scp capture/robot/pose_query_server.py i611usr@192.168.0.23:~/i611usr/
  ssh i611usr@192.168.0.23
  cd i611usr
  # zeus_jog_onboard.py 는 이미 돌고 있어도 된다. 그대로 두고 다른 SSH 세션에서:
  python pose_query_server.py
"""

# rb.open() 을 부르지 않으므로 i611_MCS/rbsys 등 로봇 열기 관련 모듈은 필요 없다.
# i611shm 만 있으면 된다.
from i611shm import shm_read

import json
import math
import socket

HOST = '0.0.0.0'
PORT = 12350

# shm 레지스터. i611usr/output.py:33, example_6axis_pendant.py:35,57 과 동일.
SHM_POSE_REG = 0x3000        # x,y,z(m), rz,ry,rx(rad) - 컨트롤러 현재 tool 기준
SHM_JOINT_REG = 0x3050       # j1..j6(rad)


# ----------------------------------------------------------------
# 로봇 상태 (shm 만)
# ----------------------------------------------------------------

def read_pose():
    u"""컨트롤러의 EE 자세를 [x_mm, y_mm, z_mm, rz_deg, ry_deg, rx_deg] 로 반환.

    변환은 i611usr/output.py:read_position_info 와 동일하다.
    """
    values = shm_read(SHM_POSE_REG, 6).split(',')
    return [
        float(values[0]) * 1000.0,
        float(values[1]) * 1000.0,
        float(values[2]) * 1000.0,
        math.degrees(float(values[3])),
        math.degrees(float(values[4])),
        math.degrees(float(values[5])),
    ]


def read_joints():
    u"""관절값을 [j1_deg .. j6_deg] 로 반환. i611usr/example_6axis_pendant.py 와 동일."""
    values = shm_read(SHM_JOINT_REG, 6).split(',')
    return [round(math.degrees(float(v)), 4) for v in values]


def state_payload():
    u"""자세 응답. 필드 이름을 두 벌 담는다.

    flange_pose_6dof / joints_6dof : capture/record_dataset.py 의 이름
    tcp_6dof / joint_6dof          : i611usr/sam3d_calb/robot_pose_server.py 의 이름
    같은 값이다. 어느 PC 스크립트가 붙어도 읽을 수 있게 둘 다 넣는다.
    """
    pose = read_pose()
    joints = read_joints()
    return {
        "status": "ok",
        "flange_pose_6dof": pose,
        "joints_6dof": joints,
        "tcp_6dof": pose,
        "joint_6dof": joints,
        "tool": None,                    # shm 로는 알 수 없다. docstring 주의 참조.
        "source": "shm",
    }


def error(detail):
    u"""detail 은 이 서버, reason 은 기존 서버가 쓰는 이름. 같은 값을 넣는다."""
    return {"status": "error", "detail": detail, "reason": detail}


def fmt6(values):
    return '[' + ', '.join('{:.2f}'.format(v) for v in values) + ']'


# ----------------------------------------------------------------
# 명령 처리
# ----------------------------------------------------------------

JOG_REJECT = (
    "이 서버는 로봇을 움직이지 못한다 (rb.open() 을 부르지 않는 shm-only 서버). "
    "조작은 로봇 콘솔의 zeus_jog_onboard.py 에서 하라. "
    "이 서버는 자세를 읽어 PC 에 넘겨줄 뿐이다."
)


def handle(request):
    command = request.get('command')

    # get_pose 는 sam3d_calb/robot_pose_server.py 가 쓰던 이름이다. 같이 받는다.
    if command in ('get_state', 'get_pose'):
        try:
            payload = state_payload()
        except Exception as e:
            return error("shm 읽기 실패: {}".format(e))
        print '  get_state -> flange {} joints {}'.format(
            fmt6(payload['flange_pose_6dof']), fmt6(payload['joints_6dof']))
        return payload

    elif command == 'ping':
        return {"status": "ok"}

    elif command == 'notify_saved':
        # PC 가 저장을 알려 온 것. 로봇 콘솔에서 진행 상황을 보기 위한 것뿐이다.
        print '  [SAVED] event_id={} (총 {}건)'.format(
            request.get('event_id', '?'), request.get('n_captures', '?'))
        return {"status": "ok"}

    elif command in ('jog', 'speed', 'settool', 'changetool'):
        return error(JOG_REJECT)

    return error("알 수 없는 명령: {}".format(command))


# ----------------------------------------------------------------
# 서버
# ----------------------------------------------------------------

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
    # 시작 시점에 shm 이 읽히는지 한 번 확인. 실패하면 컨트롤러가 켜져 있지 않거나
    # zeus_jog_onboard.py 등 rb.open() 을 부른 프로세스가 없다는 뜻일 수 있다.
    try:
        pose = read_pose()
        joints = read_joints()
        shm_ok = True
    except Exception as e:
        pose = joints = None
        shm_ok = False
        print '[경고] shm 초기 읽기 실패: {}'.format(e)
        print '       zeus_jog_onboard.py 등 rb.open() 을 부른 프로세스가 없으면'
        print '       shm 이 비어 있을 수 있다. 조작 스크립트를 먼저 띄우고 다시 시작하라.'

    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind((HOST, PORT))
    server.listen(1)

    print '=================================================================='
    print '  자세 응답 서버 (shm-only, zeus_jog_onboard.py 와 공존)'
    print '  포트 {} 에서 PC 를 기다린다.'.format(PORT)
    print '  PC:  python capture/record_dataset.py --robot-port {}'.format(PORT)
    print '  Ctrl+C 로 종료.'
    print '=================================================================='
    if shm_ok:
        print '현재 flange: {}'.format(fmt6(pose))
        print '현재 joints: {}'.format(fmt6(joints))
    print ''

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
    except KeyboardInterrupt:
        print ''
        print '중단됨'
