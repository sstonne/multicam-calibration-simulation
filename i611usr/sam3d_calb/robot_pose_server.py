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
import select
import json

HOST = '0.0.0.0'
PORT = 12348

GRIPPER_IO_PORT = 48
GRIPPER_TIMEOUT_SEC = 5.0

# Cube / tool 형상.
#
# 그리퍼가 잡는 위치 = 큐브 "윗면" (flange 와 가까운 쪽 면)의 정중앙.
# 즉 gripper tip 에서 tool +z 방향(= flange 에서 멀어지는 방향)으로
# 큐브가 (CUBE_SIZE/2 - CUBE_GRIP_DEPTH) mm 만큼 뻗어 있고, 그 끝에 cube center.
#
#   flange ──┬── (tool 0)
#            │   tool +z (down/away from flange)
#            │   ┊
#            ▼   ┊
#         gripper tip (tool 3, z = TOOL_GRIPPER_Z)
#            │   = 큐브 윗면 중점에 닿음
#            │   ┊ ← 그리퍼가 큐브에 2mm (CUBE_GRIP_DEPTH_MM) 박힘
#            ▼   ┊
#         cube center (tool 4, z = TOOL_GRIPPER_Z + CUBE_CENTER_OFFSET_Z)
#            │   ┊
#            ▼   ┊
#         cube bottom face
#
# robot_calb.py 는 TOOL_CUBE_CENTER_Z = TOOL_GRIPPER_Z - CUBE_CENTER_OFFSET_Z 로
# 부호가 반대로 되어 있음 — 그 코드는 cube center 를 flange 쪽으로 13mm
# (= 큐브가 위로 뻗는 형상) 잡아서 hand-eye 입력으로 쓰면 26mm 어긋남.
# 이 서버는 사용자 확인된 실제 기하 (윗면 그립) 로 정정.
CUBE_SIZE_MM = 30.0
CUBE_GRIP_DEPTH_MM = 2.0
CUBE_CENTER_OFFSET_Z = CUBE_SIZE_MM / 2.0 - CUBE_GRIP_DEPTH_MM  # +13.0 mm (away from flange)
TOOL_GRIPPER_Z = 150.0
TOOL_CUBE_CENTER_Z = TOOL_GRIPPER_Z + CUBE_CENTER_OFFSET_Z      # 163.0 mm

TCP_AXIS_MAP = {'x': 'dx', 'y': 'dy', 'z': 'dz',
                'rz': 'drz', 'ry': 'dry', 'rx': 'drx'}
JOINT_AXIS_MAP = {'d1': 'dj1', 'd2': 'dj2', 'd3': 'dj3',
                  'd4': 'dj4', 'd5': 'dj5', 'd6': 'dj6'}

# Per-connection receive buffer (fileno -> bytes).
# Multi-client 지원: 여러 클라이언트가 동시 접속해도 각자 buffer 보존.
_RECV_BUFS = {}


# -- Socket helpers --

def send_json(conn, obj):
    try:
        msg = json.dumps(obj)
        conn.sendall((msg + '\n').encode('utf-8'))
    except socket.error as e:
        print('Send error: {}'.format(e))


def try_recv_json(conn):
    """Non-blocking: read available bytes, return (parsed_json|None, peer_closed_bool).
    Returns parsed_json when a full line is in this conn's buffer."""
    fno = conn.fileno()
    if fno not in _RECV_BUFS:
        _RECV_BUFS[fno] = b''
    try:
        chunk = conn.recv(65536)
    except socket.error:
        return None, False
    if not chunk:
        return None, True
    _RECV_BUFS[fno] += chunk
    if b'\n' not in _RECV_BUFS[fno]:
        return None, False
    line, _, rest = _RECV_BUFS[fno].partition(b'\n')
    _RECV_BUFS[fno] = rest
    try:
        return json.loads(line.decode('utf-8').strip()), False
    except Exception as e:
        print('Recv parse error: {}'.format(e))
        return None, False


def cleanup_recv_buf(conn):
    fno = conn.fileno()
    if fno in _RECV_BUFS:
        del _RECV_BUFS[fno]


# -- Robot helpers --

def get_tcp():
    return rb.getpos().pos2list()[:6]


def get_joints():
    return rb.getjnt().jnt2list()[:6]


def get_cube_center():
    """Switch to tool 4 (cube center), read TCP, restore tool 3."""
    rb.changetool(4)
    tcp = rb.getpos().pos2list()[:6]
    rb.changetool(3)
    return tcp


def check_gripper():
    return [din(GRIPPER_IO_PORT + i) for i in [3, 2, 1, 0]]


def gripper_state():
    bits = check_gripper()
    if bits == ['0', '1', '0', '0']:
        return 'open'
    if bits == ['0', '0', '0', '1']:
        return 'closed'
    return 'unknown'


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


def show_pose():
    tcp = get_tcp()
    jnt = get_joints()
    cube = get_cube_center()
    print ''
    print '  joints: [{:.2f}, {:.2f}, {:.2f}, {:.2f}, {:.2f}, {:.2f}]'.format(
        jnt[0], jnt[1], jnt[2], jnt[3], jnt[4], jnt[5])
    print '  tcp:    ({:.1f}, {:.1f}, {:.1f}) / ({:.1f}, {:.1f}, {:.1f})'.format(
        tcp[0], tcp[1], tcp[2], tcp[3], tcp[4], tcp[5])
    print '  cube:   ({:.1f}, {:.1f}, {:.1f}) / ({:.1f}, {:.1f}, {:.1f})'.format(
        cube[0], cube[1], cube[2], cube[3], cube[4], cube[5])
    print '  grip:   {}'.format(gripper_state())
    print ''


def move_tcp(axis, value):
    if axis not in TCP_AXIS_MAP:
        print 'Invalid axis: {}. Use x,y,z,rz,ry,rx'.format(axis)
        return
    current = Position(*rb.getpos().pos2list()[:6])
    rb.line(current.offset(**{TCP_AXIS_MAP[axis]: value}))


def move_joint(axis, value):
    if axis not in JOINT_AXIS_MAP:
        print 'Invalid axis: {}. Use d1~d6'.format(axis)
        return
    current = Joint(*rb.getjnt().jnt2list()[:6])
    rb.move(current.offset(**{JOINT_AXIS_MAP[axis]: value}))


# -- Socket request handler --

def handle_socket_message(conn, obj):
    """Return one of: 'continue', 'disconnect'."""
    if not isinstance(obj, dict):
        send_json(conn, {"status": "error", "reason": "invalid request (not a dict)"})
        return 'continue'
    cmd = obj.get('command')
    if cmd == 'ping':
        send_json(conn, {"status": "ok"})
        return 'continue'
    if cmd == 'get_pose':
        try:
            jnt = get_joints()
            tcp = get_tcp()
            cube = get_cube_center()
            send_json(conn, {
                "status": "ok",
                "joint_6dof": jnt,
                "tcp_6dof": tcp,
                "cube_center_6dof": cube,
                "gripper_state": gripper_state(),
                "tool_gripper_z_mm": TOOL_GRIPPER_Z,
                "tool_cube_center_z_mm": TOOL_CUBE_CENTER_Z,
            })
            print '[Sock] get_pose -> joint d1={:.2f}.. tcp z={:.1f} cube z={:.1f}'.format(
                jnt[0], tcp[2], cube[2])
        except Exception as e:
            send_json(conn, {"status": "error", "reason": str(e)})
        return 'continue'
    if cmd == 'gotoj':
        # Calib_Step2c_replay_joint_sequence 용. joints 6-list (deg) 로 절대 joint move.
        # 이동이 완료될 때까지 blocking — 클라이언트는 timeout 충분히 크게 (기본 30초).
        try:
            joints = obj.get('joints')
            if not (isinstance(joints, (list, tuple)) and len(joints) == 6):
                send_json(conn, {"status": "error",
                                 "reason": "gotoj requires 'joints': [d1..d6]"})
                return 'continue'
            vals = [float(v) for v in joints]
            # use_mt(True) 라도 rb.move() 자체가 motion 완료까지 block 함 (i611 SDK 동작).
            # → 응답은 motion 끝난 뒤 전송 → 클라이언트는 단순히 wait.
            rb.move(Joint(*vals))
            jnt_after = get_joints()
            send_json(conn, {
                "status": "ok",
                "joint_6dof": jnt_after,
            })
            print('[Sock] gotoj -> d=[{:.1f},{:.1f},{:.1f},{:.1f},{:.1f},{:.1f}]'.format(
                jnt_after[0], jnt_after[1], jnt_after[2],
                jnt_after[3], jnt_after[4], jnt_after[5]))
        except Exception as e:
            send_json(conn, {"status": "error", "reason": str(e)})
        return 'continue'
    if cmd == 'gotop':
        # TCP absolute move. tcp 6-list [x_mm,y_mm,z_mm,rz_deg,ry_deg,rx_deg].
        try:
            tcp = obj.get('tcp')
            if not (isinstance(tcp, (list, tuple)) and len(tcp) == 6):
                send_json(conn, {"status": "error",
                                 "reason": "gotop requires 'tcp': [x,y,z,rz,ry,rx]"})
                return 'continue'
            vals = [float(v) for v in tcp]
            rb.line(Position(*vals))
            tcp_after = get_tcp()
            send_json(conn, {"status": "ok", "tcp_6dof": tcp_after})
            print('[Sock] gotop -> tcp z={:.1f} (rz,ry,rx)=({:.1f},{:.1f},{:.1f})'.format(
                tcp_after[2], tcp_after[3], tcp_after[4], tcp_after[5]))
        except Exception as e:
            send_json(conn, {"status": "error", "reason": str(e)})
        return 'continue'
    if cmd == 'notify_saved':
        # PC (Calib_Step2) 가 meta.json 저장 직후 호출.
        # 서버 터미널에 눈에 띄게 표시해 사용자가 확인 가능.
        eid = obj.get('event_id', '?')
        n_total = obj.get('n_captures', '?')
        rgb_path = obj.get('rgb_path', '')
        print('')
        print('==================================================')
        print('  [CAPTURED] event_id={} saved (total={})'.format(eid, n_total))
        if rgb_path:
            print('  rgb: {}'.format(rgb_path))
        print('==================================================')
        sys.stdout.write('> '); sys.stdout.flush()
        send_json(conn, {"status": "ok"})
        return 'continue'
    if cmd == 'quit':
        send_json(conn, {"status": "ok"})
        return 'disconnect'
    send_json(conn, {"status": "error", "reason": "unknown command: {}".format(cmd)})
    return 'continue'


# -- REPL command handler --

def handle_stdin(cmd):
    """Return False to quit server."""
    cl = cmd.lower()
    if cl == 'q':
        return False
    if cl == 'show':
        show_pose()
    elif cl.startswith('speed'):
        parts = cmd.split()
        if len(parts) >= 2:
            try:
                rb.override(int(parts[1]))
                print 'Speed: {}'.format(int(parts[1]))
            except ValueError:
                print 'Usage: speed <0-100>'
    elif cl == 'go':
        gripper_open()
    elif cl == 'gc':
        gripper_close()
    elif cl.startswith('gotoj '):
        vals = [float(v.strip()) for v in cmd[6:].strip().split(',')]
        if len(vals) != 6:
            print 'Usage: gotoj d1,d2,d3,d4,d5,d6'
        else:
            rb.move(Joint(*vals))
            show_pose()
    elif cl.startswith('gotop '):
        vals = [float(v.strip()) for v in cmd[6:].strip().split(',')]
        if len(vals) == 6:
            rb.line(Position(*vals))
            show_pose()
        elif len(vals) == 3:
            tcp = get_tcp()
            rb.line(Position(vals[0], vals[1], vals[2], tcp[3], tcp[4], tcp[5]))
            show_pose()
        else:
            print 'Usage: gotop x,y,z[,rz,ry,rx]'
    elif cl.startswith('p '):
        parts = cmd[2:].strip().split(',')
        if len(parts) == 2:
            move_tcp(parts[0].strip(), float(parts[1].strip()))
            show_pose()
        else:
            print 'Usage: p <axis>,<value>'
    elif cl.startswith('j '):
        parts = cmd[2:].strip().split(',')
        if len(parts) == 2:
            move_joint(parts[0].strip(), float(parts[1].strip()))
            show_pose()
        else:
            print 'Usage: j <axis>,<value>'
    else:
        print 'Unknown: {}'.format(cmd)
    return True


def main():
    rbs = None
    srv = None
    conns = []  # multi-client connection list (filled in main loop)
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
        rb.override(50)

        rb.settool(1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        rb.settool(3, 0.0, 0.0, TOOL_GRIPPER_Z, 0.0, 0.0, 0.0)
        rb.settool(4, 0.0, 0.0, TOOL_CUBE_CENTER_Z, 0.0, 0.0, 0.0)
        rb.changetool(3)
        rb.use_mt(True)

        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        srv.bind((HOST, PORT))
        srv.listen(8)  # multi-client backlog
        srv.setblocking(False)

        print ''
        print '=========================================='
        print '  Robot Pose Server (port {})'.format(PORT)
        print '  - tool 3 (gripper) z = {:.1f} mm'.format(TOOL_GRIPPER_Z)
        print '  - tool 4 (cube ctr) z = {:.1f} mm'.format(TOOL_CUBE_CENTER_Z)
        print '  Socket: {"command":"get_pose"} -> joint/tcp/cube_center'
        print '  REPL:   gotoj | gotop | p | j | show | speed | go | gc | q'
        print '=========================================='
        print ''
        show_pose()
        sys.stdout.write('> '); sys.stdout.flush()

        # Multi-client: 여러 클라이언트가 동시 접속 가능
        # (stepper + Calib_Step2 가 동시에 server 와 통신).
        # conns 리스트는 main() 진입부에서 이미 초기화됨.

        running = True
        while running:
            rlist = [sys.stdin, srv] + conns
            try:
                ready, _, _ = select.select(rlist, [], [], 0.2)
            except select.error:
                continue

            if srv in ready:
                try:
                    new_conn, addr = srv.accept()
                    new_conn.setblocking(False)
                    conns.append(new_conn)
                    print('\n[Sock] client connected: {} (total={})'.format(addr, len(conns)))
                    sys.stdout.write('> '); sys.stdout.flush()
                except socket.error as e:
                    print('[Sock] accept error: {}'.format(e))

            # 각 연결 독립 처리
            for c in list(conns):
                if c not in ready:
                    continue
                obj, peer_closed = try_recv_json(c)
                if peer_closed:
                    print('\n[Sock] client disconnected (remaining={})'.format(len(conns) - 1))
                    conns.remove(c)
                    cleanup_recv_buf(c)
                    try: c.close()
                    except Exception: pass
                    sys.stdout.write('> '); sys.stdout.flush()
                elif obj is not None:
                    action = handle_socket_message(c, obj)
                    if action == 'disconnect':
                        conns.remove(c)
                        cleanup_recv_buf(c)
                        try: c.close()
                        except Exception: pass

            if sys.stdin in ready:
                try:
                    cmd = sys.stdin.readline()
                except Exception:
                    break
                if not cmd:
                    break
                cmd = cmd.strip()
                if cmd:
                    try:
                        if not handle_stdin(cmd):
                            running = False
                    except Exception as e:
                        print 'Error: {}'.format(e)
                sys.stdout.write('> '); sys.stdout.flush()

    except KeyboardInterrupt:
        print '\nInterrupted'
    except Robot_emo as e:
        print(e)
    except Robot_error as e:
        print(e)
    except Robot_fatalerror as e:
        print(e)
    except Exception as e:
        print(e)
    finally:
        for c in conns:
            try: c.close()
            except Exception: pass
        try:
            if srv is not None: srv.close()
        except Exception: pass
        try:
            if rb is not None:
                rb.exit(0)
                rb.close()
        except Exception: pass
        try:
            if rbs is not None: rbs.close()
        except Exception: pass


if __name__ == '__main__':
    main()

