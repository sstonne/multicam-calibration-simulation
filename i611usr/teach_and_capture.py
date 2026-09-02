#!/usr/bin/python
# -*- coding: utf-8 -*-

from i611_MCS import *
from teachdata import *
from i611_extend import *
from rbsys import *
from i611_common import *
from i611_io import *
from i611shm import *
import time
import socket
import json

HOST = '0.0.0.0'
PORT = 12348

# Gripper IO port
GRIPPER_IO_PORT = 48
GRIPPER_TIMEOUT_SEC = 5.0

# Default capture height offset (mm above place z)
DEFAULT_CAPTURE_Z_OFFSET = 200.0


# ──────────────────────────────────────────────────────────────
# Socket helpers
# ──────────────────────────────────────────────────────────────

def send_json(conn, obj):
    try:
        msg = json.dumps(obj)
        conn.sendall(msg.encode('utf-8'))
        print "Sent: {}".format(msg)
    except socket.error as e:
        print "Send error: {}".format(e)


def recv_json(conn):
    try:
        data = conn.recv(8192).decode('utf-8')
        if not data:
            return None
        return json.loads(data.strip())
    except Exception as e:
        print "Recv error: {}".format(e)
    return None


# ──────────────────────────────────────────────────────────────
# Robot helpers
# ──────────────────────────────────────────────────────────────

def get_tcp():
    pose = rb.getpos()
    vals = pose.pos2list()
    return [vals[0], vals[1], vals[2], vals[3], vals[4], vals[5]]


def get_joints():
    jnt = rb.getjnt()
    vals = jnt.jnt2list()
    return [vals[0], vals[1], vals[2], vals[3], vals[4], vals[5]]


def show_pose():
    tcp = get_tcp()
    jnt = get_joints()
    print ''
    print '=== Current TCP Pose ==='
    print '  x={:.3f}  y={:.3f}  z={:.3f}'.format(tcp[0], tcp[1], tcp[2])
    print '  rz={:.3f}  ry={:.3f}  rx={:.3f}'.format(tcp[3], tcp[4], tcp[5])
    print '=== Current Joints ==='
    print '  d1={:.3f}  d2={:.3f}  d3={:.3f}'.format(jnt[0], jnt[1], jnt[2])
    print '  d4={:.3f}  d5={:.3f}  d6={:.3f}'.format(jnt[3], jnt[4], jnt[5])
    print ''
    return tcp


def move_tcp(axis, value):
    pose = rb.getpos()
    vals = pose.pos2list()
    current = Position(vals[0], vals[1], vals[2], vals[3], vals[4], vals[5])

    axis_map = {
        'x': 'dx', 'y': 'dy', 'z': 'dz',
        'rz': 'drz', 'ry': 'dry', 'rx': 'drx'
    }
    if axis not in axis_map:
        print 'Invalid axis: {}. Use x,y,z,rz,ry,rx'.format(axis)
        return

    kwargs = {axis_map[axis]: value}
    target = current.offset(**kwargs)
    print 'TCP move: {} += {}'.format(axis, value)
    rb.line(target)
    print 'Move complete'


def move_joint(axis, value):
    jnt = rb.getjnt()
    vals = jnt.jnt2list()
    current = Joint(vals[0], vals[1], vals[2], vals[3], vals[4], vals[5])

    axis_map = {
        'd1': 'dj1', 'd2': 'dj2', 'd3': 'dj3',
        'd4': 'dj4', 'd5': 'dj5', 'd6': 'dj6'
    }
    if axis not in axis_map:
        print 'Invalid axis: {}. Use d1,d2,d3,d4,d5,d6'.format(axis)
        return

    kwargs = {axis_map[axis]: value}
    target = current.offset(**kwargs)
    print 'Joint move: {} += {}'.format(axis, value)
    rb.move(target)
    print 'Move complete'


def move_z_to(target_z):
    """Move only z axis to absolute target_z value."""
    tcp = get_tcp()
    current = Position(tcp[0], tcp[1], tcp[2], tcp[3], tcp[4], tcp[5])
    dz = target_z - tcp[2]
    target = current.offset(dz=dz)
    print 'Moving z: {:.1f} -> {:.1f} (dz={:.1f})'.format(tcp[2], target_z, dz)
    rb.line(target)
    print 'Move complete'


def move_z_offset(offset):
    """Move z axis by offset mm."""
    tcp = get_tcp()
    current = Position(tcp[0], tcp[1], tcp[2], tcp[3], tcp[4], tcp[5])
    target = current.offset(dz=offset)
    print 'Moving z: {:.1f} -> {:.1f} (dz={:.1f})'.format(tcp[2], tcp[2] + offset, offset)
    rb.line(target)
    print 'Move complete'


def check_gripper():
    """Read gripper state from din 48~51."""
    a = din(GRIPPER_IO_PORT)
    b = din(GRIPPER_IO_PORT + 1)
    c = din(GRIPPER_IO_PORT + 2)
    d = din(GRIPPER_IO_PORT + 3)
    return [d, c, b, a]


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


def do_capture(conn, capture_count):
    """Send capture command and wait for response."""
    tcp = get_tcp()
    print ''
    print '*** CAPTURING *** (pose_index={})'.format(capture_count)
    print '  TCP: [{:.1f}, {:.1f}, {:.1f}, {:.1f}, {:.1f}, {:.1f}]'.format(
        tcp[0], tcp[1], tcp[2], tcp[3], tcp[4], tcp[5])

    send_json(conn, {
        "command": "capture",
        "capture_pose_6dof": tcp,
        "pose_index": capture_count
    })

    resp = recv_json(conn)
    if resp is None:
        print 'Client disconnected!'
        return False

    status = resp.get('status', 'unknown') if isinstance(resp, dict) else 'unknown'
    print '*** Capture {} done (status={}) ***'.format(capture_count, status)
    print ''
    return True


# ──────────────────────────────────────────────────────────────
# Main
# ──────────────────────────────────────────────────────────────

def main():
    try:
        rbs = RobSys()
        rbs.open()

        global rb
        rb = i611Robot()
        _BASE = Base()
        rb.open()
        IOinit(rb)

        m = MotionParam(jnt_speed=30, lin_speed=50, pose_speed=50,
                        overlap=0, acctime=0.8, dacctime=0.8)
        rb.motionparam(m)
        rb.override(30)

        rb.settool(1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        rb.settool(2, 0.0, 35.0, 330.0, 0.0, 0.0, 0.0)
        rb.settool(3, 0.0, 0.0, 150.0, 0.0, 0.0, 0.0)
        rb.changetool(3)
        rb.use_mt(True)

        # ─── Start server ───
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind((HOST, PORT))
        s.listen(1)
        print "Teach-and-Capture server on port {}. Waiting for client...".format(PORT)

        conn, addr = s.accept()
        print "Client connected: {}".format(addr)

        # ─── State ───
        place_z = None          # table z height (set with 'setz')
        capture_z_offset = DEFAULT_CAPTURE_Z_OFFSET
        capture_count = 0

        print ''
        print '=========================================='
        print '  Teach-and-Capture Mode'
        print '=========================================='
        print ''
        print '--- Movement ---'
        print '  p <axis>,<value>  : TCP move (p z,-10)'
        print '  j <axis>,<value>  : Joint move (j d1,10)'
        print '  show              : Show current pose'
        print '  speed <0-100>     : Set speed'
        print ''
        print '--- Place-Capture Cycle ---'
        print '  setz              : Save current z as place height'
        print '  up <mm>           : Set capture z offset (default: {:.0f})'.format(capture_z_offset)
        print '  cycle             : Full: open -> up -> capture -> down -> close'
        print '  c                 : Capture only (just trigger cameras)'
        print ''
        print '--- Gripper ---'
        print '  go                : Gripper open (manual wait)'
        print '  gc                : Gripper close (manual wait)'
        print ''
        print '  q                 : Quit'
        print '=========================================='
        print ''
        print '*** Step 1: Move robot down until cube touches table'
        print '***         Then type "setz" to save the height'
        print ''

        show_pose()

        while True:
            try:
                cmd = raw_input('> ').strip()
            except EOFError:
                break

            if not cmd:
                continue

            cmd_lower = cmd.lower()

            # ─── Quit ───
            if cmd_lower == 'q':
                send_json(conn, {"command": "quit"})
                break

            # ─── Show pose ───
            elif cmd_lower == 'show':
                show_pose()
                if place_z is not None:
                    print '  [Saved] place_z = {:.3f}'.format(place_z)
                    print '  [Saved] capture_z_offset = {:.1f}'.format(capture_z_offset)
                    print '  [Saved] capture_z = {:.3f}'.format(place_z + capture_z_offset)
                else:
                    print '  [!] place_z not set. Use "setz" first.'
                print ''

            # ─── Speed ───
            elif cmd_lower.startswith('speed'):
                try:
                    spd = int(cmd.split()[1])
                    rb.override(spd)
                    print 'Speed set to {}'.format(spd)
                except Exception:
                    print 'Usage: speed <0-100>'

            # ─── Set place Z ───
            elif cmd_lower.startswith('setz'):
                parts = cmd.split()
                if len(parts) >= 2:
                    try:
                        place_z = float(parts[1])
                    except ValueError:
                        print('Usage: setz or setz <value>')
                        continue
                else:
                    tcp = get_tcp()
                    place_z = tcp[2]
                print ''
                print '*** Place Z saved: {:.3f} ***'.format(place_z)
                print '    Capture Z will be: {:.3f} (offset +{:.0f})'.format(
                    place_z + capture_z_offset, capture_z_offset)
                print ''

            # ─── Set capture Z offset ───
            elif cmd_lower.startswith('up'):
                try:
                    parts = cmd.split()
                    if len(parts) >= 2:
                        capture_z_offset = float(parts[1])
                    print 'Capture Z offset set to: {:.1f} mm'.format(capture_z_offset)
                    if place_z is not None:
                        print 'Capture Z will be: {:.3f}'.format(place_z + capture_z_offset)
                except Exception:
                    print 'Usage: up <mm> (e.g., up 200)'

            # ─── Gripper open ───
            elif cmd_lower == 'go':
                gripper_open()

            # ─── Gripper close ───
            elif cmd_lower == 'gc':
                gripper_close()

            # ─── CYCLE: full place-capture-pickup ───
            elif cmd_lower == 'cycle':
                if place_z is None:
                    print '[ERROR] place_z not set! Move to table height and type "setz" first.'
                    continue

                current_tcp = get_tcp()
                capture_z = place_z + capture_z_offset

                print ''
                print '====== CYCLE START (#{}) ======'.format(capture_count)
                print '  Current z: {:.1f}'.format(current_tcp[2])
                print '  Place z:   {:.1f}'.format(place_z)
                print '  Capture z: {:.1f} (+{:.0f})'.format(capture_z, capture_z_offset)
                print ''

                # 1. Move down to place z (if not already there)
                if abs(current_tcp[2] - place_z) > 1.0:
                    print '--- 1/6: Moving down to place z ---'
                    move_z_to(place_z)
                else:
                    print '--- 1/6: Already at place z ---'

                # 2. Gripper open (release cube)
                print '--- 2/6: Gripper open ---'
                gripper_open()

                # 3. Move up to capture z
                print '--- 3/6: Moving up to capture z ---'
                move_z_to(capture_z)
                time.sleep(0.5)

                # 4. Capture
                print '--- 4/6: Capturing ---'
                ok = do_capture(conn, capture_count)
                if not ok:
                    break
                capture_count += 1

                # 5. Move down to place z
                print '--- 5/6: Moving down to place z ---'
                move_z_to(place_z)

                # 6. Gripper close (grab cube)
                print '--- 6/6: Gripper close ---'
                gripper_close()

                print '====== CYCLE DONE ======'.format()
                print '  Total captures: {}'.format(capture_count)
                print '  Move to next position, then type "cycle" again'
                print ''

            # ─── CAPTURE only (no move/gripper) ───
            elif cmd_lower == 'c':
                ok = do_capture(conn, capture_count)
                if not ok:
                    break
                capture_count += 1

            # ─── TCP move ───
            elif cmd_lower.startswith('p '):
                try:
                    parts = cmd[2:].strip().split(',')
                    axis = parts[0].strip()
                    value = float(parts[1].strip())
                    move_tcp(axis, value)
                    show_pose()
                except Exception as e:
                    print 'Error: {}. Usage: p <axis>,<value>'.format(e)

            # ─── Joint move ───
            elif cmd_lower.startswith('j '):
                try:
                    parts = cmd[2:].strip().split(',')
                    axis = parts[0].strip()
                    value = float(parts[1].strip())
                    move_joint(axis, value)
                    show_pose()
                except Exception as e:
                    print 'Error: {}. Usage: j <axis>,<value>'.format(e)

            else:
                print 'Unknown: {}. (p/j/c/cycle/setz/up/go/gc/show/speed/q)'.format(cmd)

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

