#!/usr/bin/env python
# -*- coding: utf-8 -*-
u"""
Zeus ZRA Robot - Timing Benchmark
측정 항목:
  1. abort() 왕복 시간 (idle)
  2. relline() 블로킹 시간
  3. abort() + relline() 사이클 시간 (분리 측정)
  4. 실제 15Hz 루프 오버런 및 틱 간격
"""

from i611_MCS import i611Robot, MotionParam
import time
import sys

# ============================================================
# 설정값
# ============================================================
N_REPEAT   = 50
TINY_DELTA = 0.5      # 측정용 이동량 [mm]
SPEED      = 10.0     # [mm/s]
ACCT       = 0.1      # [s]
DACCT      = 0.1      # [s]
CTRL_HZ    = 15
CTRL_DT    = 1.0 / CTRL_HZ   # 66.7ms


# ============================================================
# 통계
# ============================================================
def mean(lst):
    return sum(lst) / float(len(lst))

def stdev(lst):
    m = mean(lst)
    return (sum((x - m) ** 2 for x in lst) / float(len(lst))) ** 0.5

def percentile(lst, p):
    s = sorted(lst)
    idx = int(len(s) * p / 100.0)
    return s[min(idx, len(s) - 1)]

def print_stats(label, times_ms):
    print "  [%s]" % label
    print "    n     = %d" % len(times_ms)
    print "    mean  = %.3f ms" % mean(times_ms)
    print "    stdev = %.3f ms" % stdev(times_ms)
    print "    min   = %.3f ms" % min(times_ms)
    print "    p50   = %.3f ms" % percentile(times_ms, 50)
    print "    p95   = %.3f ms" % percentile(times_ms, 95)
    print "    max   = %.3f ms" % max(times_ms)
    print ""


# ============================================================
# 측정 함수
# ============================================================

def bench_abort_idle(rb, n):
    u"""이동 없을 때 abort() TCP 왕복 시간"""
    times = []
    for _ in xrange(n):
        t0 = time.time()
        rb.abort()
        t1 = time.time()
        times.append((t1 - t0) * 1000.0)
        time.sleep(0.01)
    return times


def bench_relline_blocking(rb, n):
    u"""relline() 호출 ~ return 까지 블로킹 시간"""
    times = []
    for i in xrange(n):
        dx = TINY_DELTA if i % 2 == 0 else -TINY_DELTA
        t0 = time.time()
        rb.relline(dx=dx)
        t1 = time.time()
        times.append((t1 - t0) * 1000.0)
        time.sleep(0.02)
    return times


def bench_abort_then_relline(rb, n):
    u"""abort() + relline() 사이클 - 각각 따로 측정"""
    total_times  = []
    abort_times  = []
    relline_times = []
    for i in xrange(n):
        dx = TINY_DELTA if i % 2 == 0 else -TINY_DELTA
        t0 = time.time()
        rb.abort()
        t1 = time.time()
        rb.relline(dx=dx)
        t2 = time.time()
        abort_times.append((t1 - t0) * 1000.0)
        relline_times.append((t2 - t1) * 1000.0)
        total_times.append((t2 - t0) * 1000.0)
        time.sleep(0.02)
    return total_times, abort_times, relline_times


def bench_15hz_loop(rb, n_ticks):
    u"""실제 15Hz 루프 - 오버런 및 틱 간격 측정"""
    tick_times    = []
    command_times = []
    overruns      = 0
    next_tick     = time.time()
    prev_end      = None

    for i in xrange(n_ticks):
        dx = TINY_DELTA if i % 2 == 0 else -TINY_DELTA

        t0 = time.time()
        rb.abort()
        rb.relline(dx=dx)
        t1 = time.time()
        command_times.append((t1 - t0) * 1000.0)

        next_tick += CTRL_DT
        sleep_time = next_tick - time.time()
        if sleep_time > 0:
            time.sleep(sleep_time)
        else:
            overruns += 1
            next_tick = time.time()

        tick_end = time.time()
        if prev_end is not None:
            tick_times.append((tick_end - prev_end) * 1000.0)
        prev_end = tick_end

    return tick_times, command_times, overruns


# ============================================================
# 메인
# ============================================================

def main():
    print "=" * 60
    print "Zeus ZRA Timing Benchmark (Python 2)"
    print "=" * 60
    print ""

    print "[초기화] 로봇 연결 중..."
    try:
        rb = i611Robot()
        rb.open()
    except Exception as e:
        print "ERROR: 로봇 연결 실패 - %s" % str(e)
        sys.exit(1)
    print "        연결 완료"
    print ""

    mp = MotionParam(lin_speed=SPEED, acctime=ACCT, dacctime=DACCT)
    rb.motionparam(mp)

    try:
        # 1. abort idle
        print "[1/4] abort() idle 측정 중... (n=%d)" % N_REPEAT
        t = bench_abort_idle(rb, N_REPEAT)
        print_stats("abort() idle - 순수 TCP 왕복", t)

        # 2. relline 블로킹
        print "[2/4] relline() 블로킹 시간 측정 중... (n=%d)" % N_REPEAT
        t = bench_relline_blocking(rb, N_REPEAT)
        print_stats("relline(dx=%.1fmm) 블로킹 시간" % TINY_DELTA, t)

        # 3. abort + relline 사이클
        print "[3/4] abort+relline 사이클 측정 중... (n=%d)" % N_REPEAT
        total_t, abort_t, rl_t = bench_abort_then_relline(rb, N_REPEAT)
        print_stats("abort() 단독", abort_t)
        print_stats("relline() 단독 (abort 직후)", rl_t)
        print_stats("abort() + relline() 합계 (1틱 비용)", total_t)

        # 4. 15Hz 루프
        print "[4/4] 15Hz 루프 측정 중... (%d ticks)" % N_REPEAT
        tick_t, cmd_t, overruns = bench_15hz_loop(rb, N_REPEAT)
        print_stats("15Hz 루프 - 실제 틱 간격 (목표 66.7ms)", tick_t)
        print_stats("15Hz 루프 - 명령 처리 시간 (abort+relline)", cmd_t)
        print "  오버런: %d / %d ticks (%.1f%%)" % (
            overruns, N_REPEAT, overruns * 100.0 / N_REPEAT)
        print ""

        # 종합 판정
        print "=" * 60
        print "종합 판정"
        print "=" * 60
        cmd_mean = mean(cmd_t)
        cmd_p95  = percentile(cmd_t, 95)
        budget   = CTRL_DT * 1000.0

        print "  제어 예산 (1틱):   %.1f ms" % budget
        print "  명령 평균 소요:    %.1f ms" % cmd_mean
        print "  명령 p95 소요:     %.1f ms" % cmd_p95
        print "  여유 (평균):       %.1f ms" % (budget - cmd_mean)
        print "  여유 (p95):        %.1f ms" % (budget - cmd_p95)
        print ""

        if cmd_p95 < budget * 0.5:
            print "  판정: [OK]   15Hz 충분히 가능"
        elif cmd_p95 < budget:
            print "  판정: [WARN] 15Hz 경계선 - spike 시 오버런 위험"
        else:
            print "  판정: [FAIL] 15Hz 불가"
            print "        현실적 최대: %.1f Hz" % (1000.0 / cmd_p95)

        print ""
        print "  ※ relline() 블로킹 시간이 수백ms 이상이면"
        print "     cprmove가 동작완료까지 블로킹 중인 것입니다."

    except Exception as e:
        import traceback
        print "ERROR: %s" % str(e)
        traceback.print_exc()
    finally:
        rb.close()
        print ""
        print "연결 종료."


if __name__ == '__main__':
    main()
