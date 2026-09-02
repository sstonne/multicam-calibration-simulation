#!/usr/bin/python
# -*- coding: utf-8 -*-
# zeus_server/align_pose_onboard.py
"""Snap the ZEUS TCP rotation to the top-down pick-and-place pose — runs ON
THE ROBOT PC, no TCP/client, no numpy/scipy. PYTHON 2.

Position (x,y,z) is left untouched; only rotation is re-aligned. The target
is the SAME orientation tools/align_pose.py aligns to for the UR3/ZEUS
client path — task-frame (rx=180, ry=0, rz=0), i.e. looking straight down
with wrist yaw 0 — but computed directly in ZEUS native (rz,ry,rx) terms
using the frame calibration constants below, since this process has no
access to robot/frames.py (Python 3 + scipy).

KEEP BASE_FROM_TASK_YAW_DEG / TOOL_ALIGN_YAW_DEG IN SYNC with
config/real_config.zeus.json -> "frame" -> base_from_task_yaw_deg /
tool_align_yaw_deg. If those get recalibrated, update the two constants
here too (or pass --base-yaw / --tool-align to override for one run).

Do not run this at the same time as zeus_server.py or zeus_jog_onboard.py
(all three call rb.open() and would fight over the controller).

  python ~/align_pose_onboard.py
  python ~/align_pose_onboard.py --yes
"""

from __future__ import print_function

import argparse
import math
import sys

from i611_MCS import *
from i611_extend import *
from rbsys import *
from i611_common import *
from i611_io import *
from i611shm import *

# keep in sync with config/real_config.zeus.json -> "frame"
BASE_FROM_TASK_YAW_DEG = -179.96
TOOL_ALIGN_YAW_DEG = -90.0

MAX_LIN_SPEED = 120.0
MAX_JNT_SPEED = 30.0
MAX_POSE_SPEED = 50.0
DEFAULT_ACC = 0.6
THRESHOLD_DEG = 0.5


# ── pure-python rotation math (no numpy/scipy on the robot PC) ───────────
# native pose rotation = Rz(rz) @ Ry(ry) @ Rx(rx)  (see robot/backends/zeus_client.py)

def _rotz(deg):
    a = math.radians(deg); c, s = math.cos(a), math.sin(a)
    return [[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]]


def _roty(deg):
    a = math.radians(deg); c, s = math.cos(a), math.sin(a)
    return [[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]]


def _rotx(deg):
    a = math.radians(deg); c, s = math.cos(a), math.sin(a)
    return [[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]]


def _matmul(A, B):
    return [[sum(A[i][k] * B[k][j] for k in range(3)) for j in range(3)]
            for i in range(3)]


def _transpose(A):
    return [[A[j][i] for j in range(3)] for i in range(3)]


def native_to_R(rz, ry, rx):
    return _matmul(_matmul(_rotz(rz), _roty(ry)), _rotx(rx))


def R_to_native(Rm):
    """Inverse of native_to_R: decompose Rm = Rz(rz)@Ry(ry)@Rx(rx)."""
    r20 = max(-1.0, min(1.0, Rm[2][0]))
    ry = math.degrees(math.asin(-r20))
    cy = math.cos(math.radians(ry))
    if abs(cy) > 1e-6:
        rx = math.degrees(math.atan2(Rm[2][1], Rm[2][2]))
        rz = math.degrees(math.atan2(Rm[1][0], Rm[0][0]))
    else:
        rx = math.degrees(math.atan2(-Rm[1][2], Rm[1][1]))
        rz = 0.0
    return rz, ry, rx


def rotation_angle_deg(Ra, Rb):
    """Geodesic angle between two rotation matrices, degrees (trace formula)."""
    RtR = _matmul(_transpose(Ra), Rb)
    tr = RtR[0][0] + RtR[1][1] + RtR[2][2]
    c = max(-1.0, min(1.0, (tr - 1.0) / 2.0))
    return math.degrees(math.acos(c))


def target_native_R(base_yaw_deg, tool_align_deg):
    """R_base = Rz(base_yaw) @ Rx(180) @ Rz(tool_align) — the frame.rot_to_base
    composition (robot/frames.py) with R_task fixed to the top-down pose
    (rx=180, ry=0, rz=0), so R_task itself is just Rx(180)."""
    return _matmul(_matmul(_rotz(base_yaw_deg), _rotx(180.0)), _rotz(tool_align_deg))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--yes", action="store_true", help="skip the move confirmation")
    ap.add_argument("--speed", type=float, default=20.0, help="mm/s")
    ap.add_argument("--pose-speed", type=float, default=20.0)
    ap.add_argument("--acc", type=float, default=0.4)
    ap.add_argument("--override", type=float, default=80.0)
    ap.add_argument("--base-yaw", type=float, default=BASE_FROM_TASK_YAW_DEG,
                    help="override base_from_task_yaw_deg for one run")
    ap.add_argument("--tool-align", type=float, default=TOOL_ALIGN_YAW_DEG,
                    help="override tool_align_yaw_deg for one run")
    args = ap.parse_args()

    print("initializing i611 robot ...")
    rb = i611Robot()
    _BASE = Base()
    rb.open()
    IOinit()
    rb.override(args.override)
    # same TCP as zeus_server.py — keep these two files in sync
    rb.settool(1, 0.0, 0.0, 97.5, 0.0, 0.0, 0.0)
    rb.changetool(1)
    print("robot ready")

    try:
        x, y, z, rz, ry, rx = [float(v) for v in rb.getpos().pos2list()[:6]]
        R_cur = native_to_R(rz, ry, rx)
        R_target = target_native_R(args.base_yaw, args.tool_align)
        tz, ty, tx = R_to_native(R_target)
        diff_deg = rotation_angle_deg(R_cur, R_target)

        print("  현재 : x=%.2f y=%.2f z=%.2f  rz=%.2f ry=%.2f rx=%.2f"
              % (x, y, z, rz, ry, rx))
        print("  목표 : rz=%.2f ry=%.2f rx=%.2f  (base_yaw=%.2f, tool_align=%.2f 로부터 계산)"
              % (tz, ty, tx, args.base_yaw, args.tool_align))
        print("  틀어진 각도: %.2f deg" % diff_deg)

        if diff_deg < THRESHOLD_DEG:
            print("  이미 정렬 상태입니다.")
            return 0

        if not args.yes:
            sys.stdout.write("\n정렬 실행? (y/n): ")
            sys.stdout.flush()
            ans = sys.stdin.readline().strip().lower()
            if ans != "y":
                print("취소됨.")
                return 0

        speed = min(args.speed, MAX_LIN_SPEED)
        pose_speed = min(args.pose_speed, MAX_POSE_SPEED)
        acc = max(0.05, min(args.acc, 2.0))
        rb.motionparam(MotionParam(lin_speed=speed, jnt_speed=MAX_JNT_SPEED,
                                   pose_speed=pose_speed, overlap=0.0,
                                   acctime=acc, dacctime=acc))
        rb.line(Position(x, y, z, tz, ty, tx))

        ax, ay, az, arz, ary, arx = [float(v) for v in rb.getpos().pos2list()[:6]]
        print("  완료 : x=%.2f y=%.2f z=%.2f  rz=%.2f ry=%.2f rx=%.2f"
              % (ax, ay, az, arz, ary, arx))
        return 0
    except KeyboardInterrupt:
        print("\n취소됨 (Ctrl-C).")
        try:
            rb.motion_skip()
        except Exception:
            pass
        return 130
    finally:
        rb.close()


if __name__ == "__main__":
    sys.exit(main())
