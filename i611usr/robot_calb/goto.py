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

# 그리퍼 툴 오프셋 (robot_calb.py와 동일하게 맞춤)
TOOL_GRIPPER_Z = 150.0


def fmt6(v):
    return '[{:.1f}, {:.1f}, {:.1f}, {:.1f}, {:.1f}, {:.1f}]'.format(
        v[0], v[1], v[2], v[3], v[4], v[5])


def get_tcp(rb):
    return rb.getpos().pos2list()[:6]


def get_joints(rb):
    return rb.getjnt().jnt2list()[:6]


def show_pose(rb):
    tcp = get_tcp(rb)
    jnt = get_joints(rb)
    print ''
    print '=== TCP Pose ==='
    print '  x={:.3f}  y={:.3f}  z={:.3f}'.format(tcp[0], tcp[1], tcp[2])
    print '  rz={:.3f}  ry={:.3f}  rx={:.3f}'.format(tcp[3], tcp[4], tcp[5])
    print '=== Joints ==='
    print '  d1={:.3f}  d2={:.3f}  d3={:.3f}'.format(jnt[0], jnt[1], jnt[2])
    print '  d4={:.3f}  d5={:.3f}  d6={:.3f}'.format(jnt[3], jnt[4], jnt[5])
    print ''


def parse_csv_floats(s):
    return [float(v.strip()) for v in s.strip().split(',')]


def main():
    rb = None
    rbs = None
    try:
        rbs = RobSys()
        rbs.open()

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
        rb.changetool(3)
        rb.use_mt(True)

        print ''
        print '=========================================='
        print '  Standalone Goto Server'
        print '  gotoj d1,d2,d3,d4,d5,d6 : joint abs move'
        print '  gotop x,y,z[,rz,ry,rx]  : TCP abs move'
        print '  p <axis>,<delta>        : TCP single-axis rel move'
        print '  j <joint>,<delta>       : joint single-axis rel move'
        print '  show / speed <0-100> / q'
        print '=========================================='
        print ''

        show_pose(rb)

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
                break

            # Show
            elif cl == 'show':
                show_pose(rb)

            # Speed
            elif cl.startswith('speed'):
                try:
                    spd = int(cmd.split()[1])
                    rb.override(spd)
                    print 'Speed: {}'.format(spd)
                except Exception:
                    print 'Usage: speed <0-100>'

            # Joint absolute move
            elif cl.startswith('gotoj '):
                try:
                    vals = parse_csv_floats(cmd[6:])
                    if len(vals) != 6:
                        print 'Usage: gotoj d1,d2,d3,d4,d5,d6'
                        continue
                    rb.move(Joint(*vals))
                    show_pose(rb)
                except Exception as e:
                    print 'Error: {}'.format(e)

            # TCP single-axis relative move (예: p z,-30 / p x, 10)
            elif cl.startswith('p '):
                try:
                    body = cmd[2:].replace(',', ' ')
                    parts = body.split()
                    if len(parts) != 2:
                        print 'Usage: p <axis>,<delta>  (axis=x,y,z,rz,ry,rx)'
                        continue
                    axis = parts[0].strip().lower()
                    delta = float(parts[1])
                    idx = {'x': 0, 'y': 1, 'z': 2,
                           'rz': 3, 'ry': 4, 'rx': 5}.get(axis)
                    if idx is None:
                        print 'Usage: p <axis>,<delta>  (axis=x,y,z,rz,ry,rx)'
                        continue
                    tcp = get_tcp(rb)
                    tcp[idx] += delta
                    rb.line(Position(*tcp))
                    show_pose(rb)
                except Exception as e:
                    print 'Error: {}'.format(e)

            # Joint single-axis relative move (예: j d1,-30 / j 3, 10)
            elif cl.startswith('j '):
                try:
                    body = cmd[2:].replace(',', ' ')
                    parts = body.split()
                    if len(parts) != 2:
                        print 'Usage: j <joint>,<delta>  (joint=d1~d6 or 1~6)'
                        continue
                    joint = parts[0].strip().lower()
                    delta = float(parts[1])
                    idx = {'d1': 0, 'd2': 1, 'd3': 2, 'd4': 3, 'd5': 4, 'd6': 5,
                           '1': 0, '2': 1, '3': 2, '4': 3, '5': 4,
                           '6': 5}.get(joint)
                    if idx is None:
                        print 'Usage: j <joint>,<delta>  (joint=d1~d6 or 1~6)'
                        continue
                    jnt = get_joints(rb)
                    jnt[idx] += delta
                    rb.move(Joint(*jnt))
                    show_pose(rb)
                except Exception as e:
                    print 'Error: {}'.format(e)

            # TCP absolute move
            elif cl.startswith('gotop '):
                try:
                    vals = parse_csv_floats(cmd[6:])
                    if len(vals) == 6:
                        rb.line(Position(*vals))
                    elif len(vals) == 3:
                        tcp = get_tcp(rb)
                        rb.line(Position(vals[0], vals[1], vals[2],
                                         tcp[3], tcp[4], tcp[5]))
                    else:
                        print 'Usage: gotop x,y,z[,rz,ry,rx]'
                        continue
                    show_pose(rb)
                except Exception as e:
                    print 'Error: {}'.format(e)

            else:
                print 'Unknown: {}'.format(cmd)

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
        try:
            if rb is not None:
                rb.exit(0)
                rb.close()
        except Exception:
            pass
        try:
            if rbs is not None:
                rbs.close()
        except Exception:
            pass


if __name__ == '__main__':
    main()

