"""Standalone keyboard jog for the ZEUS(i611) — runs ON THE ROBOT PC. PYTHON 2.

This merges zeus_server.py's motion/gripper primitives with zeus_jog.py's
keyboard UI into ONE process that talks to the i611 SDK directly. No TCP
layer, no client machine, no numpy/scipy — only stdlib + the i611 SDK.

Use this for manual jogging / point teaching when you're logged into the
robot PC directly. For running LLM-generated programs (run.py) you still
need the real zeus_server.py: THIS script and that one must not run at the
same time (both call rb.open() and would fight over the controller).

  python ~/zeus_jog_onboard.py --step-mm 5 --speed 30

Keys
  move  w/s +-x   a/d +-y   r/f +-z        rot  i/k +-rx  j/l +-ry  u/o +-rz
  step  [ ] halve/double   1..5 = 0.5/1/5/10/25 mm
  speed UP/DOWN linear speed +-   LEFT/RIGHT rotation speed +- (right = faster)
  grip  g open   h close      x STOP (motion_skip)
  misc  p refresh   m record point   q quit

Rotation deltas compose in the WORLD frame (same convention as
robot/backends/zeus_client.py: native pose rotation == Rz(rz)@Ry(ry)@Rx(rx),
equivalent to extrinsic-XYZ euler). Verified against that file's
_to_native/_from_native; sanity-check with small steps before trusting it
for anything precise.
"""

from __future__ import print_function

import argparse
import json
import math
import os
import select
import sys
import termios
import time
import tty

from i611_MCS import *
from i611_extend import *
from rbsys import *
from i611_common import *
from i611_io import *
from i611shm import *

# ── hard safety caps (this process is the only guard now — no server to
#    clamp behind it) ────────────────────────────────────────────────────
MAX_LIN_SPEED = 120.0     # mm/s
MAX_JNT_SPEED = 30.0      # i611 jnt_speed units
DEFAULT_ACC = 0.6         # acctime / dacctime
MAX_OVERLAP = 20.0        # mm of path blending
MAX_POSE_SPEED = 50.0

# ── gripper wiring (same as zeus_server.py — keep in sync if that changes) ─
GRIP_DOUT_PORT = 48
GRIP_OPEN_BITS = '0100'
GRIP_CLOSE_BITS = '0001'
GRIP_OPEN_SENSE = ['0', '1', '0', '0']
GRIP_CLOSE_SENSE = ['0', '0', '0', '1']

TRANS = {"w": (0, +1), "s": (0, -1), "a": (1, +1), "d": (1, -1),
         "r": (2, +1), "f": (2, -1)}
ROT = {"i": (0, +1), "k": (0, -1), "j": (1, +1), "l": (1, -1),
       "u": (2, +1), "o": (2, -1)}
PRESETS = {"1": 0.5, "2": 1.0, "3": 5.0, "4": 10.0, "5": 25.0}

HANGUL = {
    u"ㅂ": "q", u"ㅈ": "w", u"ㄷ": "e", u"ㄱ": "r", u"ㅅ": "t",
    u"ㅛ": "y", u"ㅕ": "u", u"ㅑ": "i", u"ㅐ": "o", u"ㅔ": "p",
    u"ㅁ": "a", u"ㄴ": "s", u"ㅇ": "d", u"ㄹ": "f", u"ㅎ": "g",
    u"ㅗ": "h", u"ㅓ": "j", u"ㅏ": "k", u"ㅣ": "l",
    u"ㅋ": "z", u"ㅌ": "x", u"ㅊ": "c", u"ㅍ": "v", u"ㅠ": "b",
    u"ㅜ": "n", u"ㅡ": "m",
    u"ㅃ": "q", u"ㅉ": "w", u"ㄸ": "e", u"ㄲ": "r", u"ㅆ": "t",
    u"ㅒ": "o", u"ㅖ": "p",
}


def normalize_key(ch):
    if ch in HANGUL:
        return HANGUL[ch]
    return ch.lower()


# ── pure-python rotation math (no numpy/scipy on the robot PC) ───────────
# native pose rotation = Rz(rz) @ Ry(ry) @ Rx(rx)  (see zeus_client.py docstring)

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
        # gimbal lock (ry ~ +-90): rz/rx are coupled, pick rz=0
        rx = math.degrees(math.atan2(-Rm[1][2], Rm[1][1]))
        rz = 0.0
    return rz, ry, rx


def read_gripper():
    a = din(48); b = din(49); c = din(50); d = din(51)
    return [d, c, b, a]


def grip(state, timeout_s):
    dout(GRIP_DOUT_PORT, '0000')
    if state == 'open':
        want, cmd = GRIP_OPEN_SENSE, GRIP_OPEN_BITS
    else:
        want, cmd = GRIP_CLOSE_SENSE, GRIP_CLOSE_BITS
    t0 = time.time()
    while read_gripper() != want:
        dout(GRIP_DOUT_PORT, cmd)
        if time.time() - t0 > timeout_s:
            return False
        time.sleep(0.05)
    return True


class Panel(object):
    def __init__(self, enabled):
        self.enabled = enabled
        self.n = 0

    def render(self, lines):
        if not self.enabled:
            print(lines[-2] if len(lines) >= 2 else lines[-1])
            sys.stdout.flush()
            return
        out = []
        if self.n:
            out.append("\x1b[%dA" % self.n)
        for ln in lines:
            out.append("\x1b[2K" + ln + "\n")
        self.n = len(lines)
        sys.stdout.write("".join(out))
        sys.stdout.flush()


class Jog(object):
    KEYMAP = [
        "   TRANSLATE  (native x/y/z)        ROTATE  (world rx/ry/rz)",
        "        +---+                            +---+",
        "        | W | +x                         | I | +rx",
        "   +---++---++---+    +---+         +---++---++---+    +---+",
        "   | A || S || D |    | R | +z      | J || K || L |    | U | +rz",
        "   +---++---++---+    +---+         +---++---++---+    +---+",
        "    +y   -x   -y      | F | -z       +ry  -rx  -ry     | O | -rz",
        "                      +---+                            +---+",
    ]

    def __init__(self, rb, args):
        self.rb = rb
        self.args = args
        self.step_mm = args.step_mm
        self.step_deg = args.step_deg
        self.points = []
        if os.path.isfile(args.record):
            try:
                self.points = json.load(open(args.record))
            except Exception:
                self.points = []
        self.last = "-"
        self.status = "connected"
        self.pose = [0.0] * 6
        self.joints = [0.0] * 6
        self.active = set()
        self.panel = Panel(not args.no_panel)

    _ARROW = {"A": "UP", "B": "DOWN", "C": "RIGHT", "D": "LEFT"}

    def read_burst(self):
        """One key (or arrow escape sequence), plus every repeat arriving
        within the coalesce window.

        Arrow keys arrive as ESC '[' A/B/C/D (3 bytes) — parsed as a single
        token BEFORE per-character normalize_key, so the trailing A/B/C/D
        never falls through to lowercase and gets mistaken for the a/d
        (+-y) jog keys.
        """
        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        try:
            tty.setcbreak(fd)
            raw = os.read(fd, 8)
            deadline = time.time() + self.args.coalesce_ms / 1000.0
            while time.time() < deadline:
                remain = deadline - time.time()
                if select.select([fd], [], [], remain)[0]:
                    raw += os.read(fd, 256)
                else:
                    break
            text = raw.decode("utf-8", "ignore")
            keys = []
            i = 0
            n = len(text)
            while i < n:
                if text[i] == "\x1b" and i + 2 < n and text[i + 1] == "[" \
                        and text[i + 2] in self._ARROW:
                    keys.append(self._ARROW[text[i + 2]])
                    i += 3
                elif text[i] == "\x1b":
                    i += 1  # bare ESC / truncated sequence — drop it
                else:
                    keys.append(normalize_key(text[i]))
                    i += 1
            return keys
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)

    def ask(self, msg):
        sys.stdout.write("\n" + msg)
        sys.stdout.flush()
        self.panel.n = 0
        try:
            return sys.stdin.readline().strip()
        except Exception:
            return ""

    def refresh(self):
        p = self.rb.getpos().pos2list()[:6]
        self.pose = [float(v) for v in p]
        self.joints = [float(v) for v in self.rb.getjnt().jnt2list()[:6]]

    def do_move(self, pose6, speed_mm_s, overlap_mm=0.0, acc_s=None):
        speed = min(float(speed_mm_s), MAX_LIN_SPEED)
        overlap = max(0.0, min(float(overlap_mm), MAX_OVERLAP))
        acc = max(0.05, min(float(acc_s if acc_s is not None else self.args.acc), 2.0))
        pose_speed = min(float(self.args.pose_speed), MAX_POSE_SPEED)
        self.rb.motionparam(MotionParam(lin_speed=speed, jnt_speed=MAX_JNT_SPEED,
                                        pose_speed=pose_speed, overlap=overlap,
                                        acctime=acc, dacctime=acc))
        self.rb.line(Position(pose6[0], pose6[1], pose6[2],
                              pose6[3], pose6[4], pose6[5]))

    def apply_burst(self, keys):
        d = [0.0, 0.0, 0.0]   # mm
        e = [0.0, 0.0, 0.0]   # deg, world x/y/z
        n = 0
        for k in keys:
            if k in TRANS:
                ax, sg = TRANS[k]; d[ax] += sg * self.step_mm; n += 1
            elif k in ROT:
                ax, sg = ROT[k]; e[ax] += sg * self.step_deg; n += 1
        if n == 0:
            return
        cap = self.args.max_step_mm
        d = [max(-cap, min(cap, v)) for v in d]
        e = [max(-cap, min(cap, v)) for v in e]

        x, y, z, rz, ry, rx = self.pose
        if any(e):
            R_native = native_to_R(rz, ry, rx)
            Re = native_to_R(e[2], e[1], e[0])
            target = _matmul(Re, R_native)
            rz, ry, rx = R_to_native(target)
        pose6 = [x + d[0], y + d[1], z + d[2], rz, ry, rx]
        self.do_move(pose6, self.args.speed, overlap_mm=self.args.overlap_mm)

        parts = ["%s%+.1f" % (nm, d[i]) for i, nm in enumerate("xyz") if d[i]]
        parts += ["r%s%+.1f" % (nm, e[i]) for i, nm in enumerate("xyz") if e[i]]
        self.last = "%s   (x%d)" % ("  ".join(parts), n)

    def record(self, label):
        self.points.append({"label": label, "native_pose": list(self.pose),
                            "joints": list(self.joints), "gripper": read_gripper(),
                            "ts": time.strftime("%Y-%m-%dT%H:%M:%S")})
        json.dump(self.points, open(self.args.record, "w"), indent=2)
        self.status = "recorded '%s' (%d pts)" % (label, len(self.points))

    def hl(self, text):
        for k in self.active:
            u = k.upper()
            text = text.replace("| %s |" % u, "| \x1b[7m%s\x1b[0m |" % u)
        return text

    def cap(self, k):
        u = k.upper()
        return "\x1b[7m%s\x1b[0m" % u if k in self.active else u

    def lines(self):
        a = self.args
        grip_str = "".join(read_gripper())
        rule = " " + "-" * 76
        return [
            " ZEUS jog (onboard, no server)   step %g mm / %g deg"
            % (self.step_mm, self.step_deg),
            "   speed  lin %6.1f mm/s [UP/DOWN]     rot(pose) %5.1f [LEFT/RIGHT->faster]"
            % (a.speed, a.pose_speed),
            rule,
        ] + [self.hl(r) for r in self.KEYMAP] + [
            rule,
            "   step  [ ] /2 x2    1 2 3 4 5 = 0.5 1 5 10 25 mm"
            "      overlap %gmm  acc %gs" % (a.overlap_mm, a.acc),
            "   grip  %s open  %s close      STOP %s     misc  %s refresh  %s record  %s quit"
            % (self.cap('g'), self.cap('h'), self.cap('x'),
               self.cap('p'), self.cap('m'), self.cap('q')),
            rule,
            "  native x %9.2f  y %9.2f  z %9.2f    rz %7.2f  ry %7.2f  rx %7.2f"
            % tuple(self.pose),
            "  joints " + "  ".join("%8.2f" % v for v in self.joints),
            "  grip   %s         recorded %d pt(s)" % (grip_str, len(self.points)),
            rule,
            "  last   %s" % self.last,
            "  status %s" % self.status,
        ]

    def run(self):
        self.refresh()
        self.panel.render(self.lines())
        while True:
            keys = self.read_burst()
            if not keys:
                continue
            if "q" in keys:
                return

            arrows = [k for k in keys if k in ("UP", "DOWN", "LEFT", "RIGHT")]
            rest = [k for k in keys if k not in ("UP", "DOWN", "LEFT", "RIGHT")]
            if arrows:
                for k in arrows:
                    if k == "UP":
                        self.args.speed = min(MAX_LIN_SPEED,
                                              self.args.speed + self.args.speed_step)
                    elif k == "DOWN":
                        self.args.speed = max(1.0,
                                              self.args.speed - self.args.speed_step)
                    elif k == "RIGHT":
                        self.args.pose_speed = min(MAX_POSE_SPEED,
                                                   self.args.pose_speed + self.args.rot_speed_step)
                    elif k == "LEFT":
                        self.args.pose_speed = max(1.0,
                                                   self.args.pose_speed - self.args.rot_speed_step)
                self.status = "speed lin %.1f mm/s / rot %.1f" % (
                    self.args.speed, self.args.pose_speed)
                self.active = set(arrows)
                if not rest:
                    try:
                        self.refresh()
                    except Exception as exc:
                        self.status = "state ERROR %s" % exc
                    self.panel.render(self.lines())
                    continue

            keys = rest
            self.active = set(k for k in keys if k in TRANS or k in ROT
                              or k in ("g", "h", "x", "p", "m", "[", "]")
                              or k in PRESETS)
            k0 = keys[0]
            try:
                if k0 in PRESETS:
                    self.step_mm = self.step_deg = PRESETS[k0]
                    self.status = "step %g mm" % self.step_mm
                elif k0 == "[":
                    self.step_mm = max(0.1, self.step_mm / 2.0)
                    self.step_deg = max(0.1, self.step_deg / 2.0)
                    self.status = "step %g mm" % self.step_mm
                elif k0 == "]":
                    self.step_mm = min(self.args.max_step_mm, self.step_mm * 2.0)
                    self.step_deg = min(self.args.max_step_mm, self.step_deg * 2.0)
                    self.status = "step %g mm" % self.step_mm
                elif k0 == "g":
                    grip("open", self.args.grip_timeout)
                    self.last = "grip open"; self.status = "ok"
                elif k0 == "h":
                    reached = grip("close", self.args.grip_timeout)
                    self.last = "grip close -> grasped=%s" % (not reached)
                    self.status = "ok"
                elif k0 == "x":
                    try:
                        self.rb.motion_skip()
                    except Exception:
                        pass
                    self.last = "STOP"; self.status = "stopped"
                elif k0 == "m":
                    label = self.ask("  label> ")
                    self.record(label or ("p%d" % (len(self.points) + 1)))
                elif k0 in ("p", " "):
                    self.status = "refreshed"
                else:
                    self.apply_burst(keys)
                    self.status = "ok"
            except Exception as exc:
                self.status = "ERROR %s" % exc
            try:
                self.refresh()
            except Exception as exc:
                self.status = "state ERROR %s" % exc
            self.panel.render(self.lines())


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--step-mm", type=float, default=1.0)
    ap.add_argument("--step-deg", type=float, default=1.0)
    ap.add_argument("--max-step-mm", type=float, default=25.0)
    ap.add_argument("--speed", type=float, default=30.0, help="mm/s (UP/DOWN to adjust live)")
    ap.add_argument("--pose-speed", type=float, default=20.0,
                    help="rotation speed for line moves (LEFT/RIGHT to adjust live)")
    ap.add_argument("--speed-step", type=float, default=5.0, help="mm/s per UP/DOWN press")
    ap.add_argument("--rot-speed-step", type=float, default=5.0, help="pose_speed per LEFT/RIGHT press")
    ap.add_argument("--coalesce-ms", type=float, default=120.0)
    ap.add_argument("--overlap-mm", type=float, default=3.0)
    ap.add_argument("--acc", type=float, default=0.25)
    ap.add_argument("--override", type=float, default=80.0)
    ap.add_argument("--grip-timeout", type=float, default=3.0)
    ap.add_argument("--no-panel", action="store_true")
    ap.add_argument("--record", default=os.path.join(
        os.path.dirname(os.path.abspath(__file__)), "zeus_jog_onboard_points.json"))
    args = ap.parse_args()

    print("initializing i611 robot ...")
    rb = i611Robot()
    _BASE = Base()
    rb.open()
    IOinit()
    rb.override(args.override)
    # same TCP as zeus_server.py — keep these two files in sync
    rb.settool(1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    rb.changetool(1)
    print("robot ready")

    try:
        Jog(rb, args).run()
    except KeyboardInterrupt:
        pass
    finally:
        try:
            rb.motion_skip()
        except Exception:
            pass
        rb.close()
        sys.stdout.write("\n")
    print("[jog] done. recorded points -> %s" % args.record)
    return 0


if __name__ == "__main__":
    sys.exit(main())
