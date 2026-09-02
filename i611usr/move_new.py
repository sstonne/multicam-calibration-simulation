# -*- coding: utf-8 -*-

import socket
import traceback

from i611_MCS import *
from i611_common import Robot_stop, Robot_poweroff, Robot_emo

HOST = "0.0.0.0"
PORT = 12348

rb = i611Robot()
rb.open()

# Same style as working move.py
try:
    _BASE = Base()
except Exception as e:
    print("WARN Base init:", e)

try:
    m = MotionParam(
        jnt_speed=100,
        lin_speed=10,
        pose_speed=10,
        overlap=0,
        acctime=0.4,
        dacctime=0.4
    )
    rb.motionparam(m)
    rb.override(10)
    print("motionparam OK")
except Exception as e:
    print("WARN motionparam:", e)

try:
    rb.use_mt(True)
    print("use_mt OK")
except Exception as e:
    print("WARN use_mt:", e)

try:
    rb.settool(1, 0.0, 0, 0, 245.0, 0.0, 0.0, 0.0)
    rb.changetool(1)
    print("tool OK")
except Exception as e:
    print("WARN tool:", e)


def list6_from_obj(obj, kind):
    """
    Convert Position/Joint object to list.
    """
    method_names = [
        "pos2list",
        "jnt2list",
        "joint2list",
        "to_list",
    ]

    for name in method_names:
        if hasattr(obj, name):
            try:
                values = getattr(obj, name)()
                return [
                    float(values[0]),
                    float(values[1]),
                    float(values[2]),
                    float(values[3]),
                    float(values[4]),
                    float(values[5]),
                ]
            except:
                pass

    attr_sets = [
        ["x", "y", "z", "rz", "ry", "rx"],
        ["j1", "j2", "j3", "j4", "j5", "j6"],
    ]

    for attrs in attr_sets:
        try:
            return [
                float(getattr(obj, attrs[0])),
                float(getattr(obj, attrs[1])),
                float(getattr(obj, attrs[2])),
                float(getattr(obj, attrs[3])),
                float(getattr(obj, attrs[4])),
                float(getattr(obj, attrs[5])),
            ]
        except:
            pass

    print("DEBUG cannot parse", kind)
    print("DEBUG object =", obj)
    print("DEBUG dir =", dir(obj))
    raise Exception("Cannot convert " + kind + " object to list")


def get_current_pose():
    p = rb.getpos()
    return list6_from_obj(p, "Position")


def get_current_joint():
    try:
        j = rb.getjnt()
        return list6_from_obj(j, "Joint")
    except:
        # fallback: current position -> IK joint
        p = rb.getpos()
        j = rb.Position2Joint(p)
        return list6_from_obj(j, "Joint")


def move_pose_relative(axis_name, delta):
    pose = get_current_pose()

    idx_map = {
        "px": 0,
        "py": 1,
        "pz": 2,
        "prz": 3,
        "pry": 4,
        "prx": 5,
    }

    idx = idx_map[axis_name]
    pose[idx] += delta

    x, y, z, rz, ry, rx = pose

    print("POSE MOVE:", axis_name, delta)
    print("TARGET:", x, y, z, rz, ry, rx)

    target = Position(x, y, z, rz, ry, rx)

    # Cartesian linear motion
    rb.line(target)

    after = get_current_pose()
    print("AFTER:", after)

    return after


def move_joint_relative(axis_name, delta):
    joint = get_current_joint()

    idx_map = {
        "j1": 0,
        "j2": 1,
        "j3": 2,
        "j4": 3,
        "j5": 4,
        "j6": 5,
    }

    idx = idx_map[axis_name]
    joint[idx] += delta

    j1, j2, j3, j4, j5, j6 = joint

    print("JOINT MOVE:", axis_name, delta)
    print("TARGET:", j1, j2, j3, j4, j5, j6)

    target = Joint(j1, j2, j3, j4, j5, j6)

    # Joint motion
    rb.move(target)

    after = get_current_joint()
    print("AFTER JOINT:", after)

    return after


def handle_command(cmd):
    cmd = cmd.strip()

    if not cmd:
        return "ERR empty command"

    parts = cmd.split()
    key = parts[0].lower()

    if key == "stop":
        rb.stop()
        return "OK stop"

    if key == "abort":
        rb.abort()
        return "OK abort"

    if key == "where":
        return "OK pose {}".format(get_current_pose())

    if key == "wherej":
        return "OK joint {}".format(get_current_joint())

    if len(parts) != 2:
        return "ERR format: px 1 / py -1 / pz 1 / prz 1 / j1 1 / where"

    try:
        value = float(parts[1])
    except ValueError:
        return "ERR value must be number"

    pose_keys = ["px", "py", "pz", "prz", "pry", "prx"]
    joint_keys = ["j1", "j2", "j3", "j4", "j5", "j6"]

    if key in pose_keys:
        target = move_pose_relative(key, value)
        return "OK pose {} {} -> {}".format(key, value, target)

    if key in joint_keys:
        target = move_joint_relative(key, value)
        return "OK joint {} {} -> {}".format(key, value, target)

    return "ERR unknown command: {}".format(key)


def run_server():
    print("Robot move server start: {}:{}".format(HOST, PORT))
    print("svstat =", rb.svstat())
    print("current pose =", get_current_pose())

    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind((HOST, PORT))
    server.listen(1)

    while True:
        conn, addr = server.accept()
        print("client connected:", addr)

        try:
            while True:
                data = conn.recv(1024)
                if not data:
                    break

                cmd = data.strip()
                print("CMD:", cmd)

                try:
                    response = handle_command(cmd)
                except Robot_stop:
                    response = "ERR Robot_stop"
                except Robot_poweroff:
                    response = "ERR Robot_poweroff"
                except Robot_emo:
                    response = "ERR Robot_emo"
                except Exception as e:
                    traceback.print_exc()
                    response = "ERR {}: {}".format(e.__class__.__name__, e)

                conn.sendall(response + "\n")

        finally:
            conn.close()
            print("client disconnected:", addr)


if __name__ == "__main__":
    try:
        run_server()

    except KeyboardInterrupt:
        print("keyboard interrupt")

    except Robot_poweroff:
        print("Robot power off")

    except Robot_stop:
        print("Robot stop")

    except Robot_emo:
        print("Robot emergency stop")

    except Exception as e:
        print("error:", e.__class__.__name__, ":", e)

    finally:
        try:
            rb.close()
        except:
            pass
