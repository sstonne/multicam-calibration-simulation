#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
measure_relline_abort.py
relline(dx=+5) / relline(dx=-5) 를 Worker에서 번갈아 실행하면서
Control Loop가 15Hz로 abort() + 새 명령을 투입할 수 있는지 측정
"""

import time
import threading
import Queue
import sys
from i611_MCS import i611Robot, MotionParam
from i611_common import Robot_stop, Robot_emo

# ── 설정 ──────────────────────────────────────────────────────
CTRL_HZ  = 15
CTRL_DT  = 1.0 / CTRL_HZ   # 66.7ms
N_SEC    = 10               # 측정 시간 (초)
LIN_SPEED = 100.0
ACCTIME   = 0.001
DACCTIME  = 0.001

# ── 공유 상태 ─────────────────────────────────────────────────
shutdown  = False
cmd_queue = Queue.Queue(maxsize=1)


# ── Worker Thread ─────────────────────────────────────────────
def worker(rb):
    while not shutdown:
        try:
            item = cmd_queue.get(timeout=0.2)
        except Queue.Empty:
            continue
        if item is None:
            break
        dx, dy, dz, drz = item
        try:
            rb.relline(dx=dx, dy=dy, dz=dz, drz=drz)
        except Robot_stop:
            pass
        except Robot_emo:
            print "[Worker] 비상정지!"
            break
        except Exception:
            pass
    print "[Worker] 종료"


# ── Control Loop ──────────────────────────────────────────────
def control_loop(rb):
    global shutdown

    tick       = 0
    direction  = 1          # +1 or -1
    next_tick  = time.time()
    last_log_t = time.time()

    # 측정용
    tick_count    = 0
    abort_times   = []   # abort() 소요 시간
    loop_intervals= []   # 루프 간격
    last_loop_t   = None

    end_t = time.time() + N_SEC
    print "[Control] 시작 (%.0f초 측정)" % N_SEC

    while time.time() < end_t:
        t_loop = time.time()

        # 루프 간격 측정
        if last_loop_t is not None:
            loop_intervals.append((t_loop - last_loop_t) * 1000.0)
            if len(loop_intervals) > 200:
                loop_intervals.pop(0)
        last_loop_t = t_loop

        # abort() 시간 측정
        t0 = time.time()
        rb.abort()
        t1 = time.time()
        abort_ms = (t1 - t0) * 1000.0
        abort_times.append(abort_ms)

        # 방향 교체하며 명령 투입
        dx = 5.0 * direction
        direction *= -1

        if not cmd_queue.empty():
            try: cmd_queue.get_nowait()
            except Exception: pass
        cmd_queue.put((dx, 0.0, 0.0, 0.0))

        tick_count += 1
        tick += 1

        # 1초마다 로그
        elapsed = t_loop - last_log_t
        if elapsed >= 1.0:
            loop_hz = tick_count / elapsed
            mean_loop = sum(loop_intervals[-15:]) / max(len(loop_intervals[-15:]), 1)
            min_loop  = min(loop_intervals[-15:]) if loop_intervals else 0
            max_loop  = max(loop_intervals[-15:]) if loop_intervals else 0
            mean_abort = sum(abort_times[-15:]) / max(len(abort_times[-15:]), 1)
            min_abort  = min(abort_times[-15:]) if abort_times else 0
            max_abort  = max(abort_times[-15:]) if abort_times else 0
            print ("[tick %04d]  loop=%.1fHz  "
                   "interval=%.1f/%.1f/%.1fms(mean/min/max)  "
                   "abort=%.1f/%.1f/%.1fms(mean/min/max)") % (
                tick, loop_hz,
                mean_loop, min_loop, max_loop,
                mean_abort, min_abort, max_abort,
            )
            tick_count = 0
            last_log_t = t_loop

        # 정밀 15Hz sleep
        next_tick += CTRL_DT
        while True:
            remaining = next_tick - time.time()
            if remaining <= 0:
                break
            time.sleep(min(remaining, 0.005))

    # 최종 통계
    print ""
    print "=" * 50
    print "최종 통계 (%d 틱)" % tick
    print "  루프 간격  mean=%.3fms  min=%.3fms  max=%.3fms" % (
        sum(loop_intervals)/len(loop_intervals),
        min(loop_intervals), max(loop_intervals))
    print "  abort()    mean=%.3fms  min=%.3fms  max=%.3fms" % (
        sum(abort_times)/len(abort_times),
        min(abort_times), max(abort_times))
    target_ms = 1000.0 / CTRL_HZ
    mean_loop = sum(loop_intervals)/len(loop_intervals)
    print "  목표 %.1fms 대비 오차: %+.3fms (%.2f%%)" % (
        target_ms, mean_loop - target_ms,
        abs(mean_loop - target_ms) / target_ms * 100)
    print "=" * 50

    shutdown = True


# ── Main ──────────────────────────────────────────────────────
def main():
    global shutdown

    rb = i611Robot()
    rb.open()
    mp = MotionParam(lin_speed=LIN_SPEED, acctime=ACCTIME, dacctime=DACCTIME)
    rb.motionparam(mp)
    print "연결 완료"

    t = threading.Thread(target=worker, args=(rb,))
    t.setDaemon(True)
    t.start()

    try:
        control_loop(rb)
    except KeyboardInterrupt:
        print "\n중단"
        shutdown = True

    try: cmd_queue.put_nowait(None)
    except Exception: pass

    time.sleep(0.5)
    rb.abort()
    rb.close()
    print "종료 완료"


if __name__ == '__main__':
    main()
