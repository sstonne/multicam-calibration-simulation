#!/usr/bin/env python
# -*- coding: utf-8 -*-
u"""
server_model_V3.py
==================
Zeus ZRA 로봇 - Sim-to-Real 15Hz 스트리밍 제어
"""

import socket
import threading
import time
import json
import sys
import Queue
import signal
import atexit

from i611_MCS import i611Robot, MotionParam
from i611_common import Robot_stop, Robot_emo

# ============================================================
# 설정
# ============================================================
RECV_HOST  = '0.0.0.0'
RECV_PORT  = 12349

CTRL_HZ    = 15
CTRL_DT    = 1.0 / CTRL_HZ

LIN_SPEED  = 10.0
ACCTIME    = 0.01
DACCTIME   = 0.001

MAX_DX     = 20.0
MAX_DY     = 20.0
MAX_DZ     = 20.0
MAX_DRZ    = 5.56

STALE_SEC  = 0.5


# ============================================================
# 공유 상태
# ============================================================
class SharedState(object):
    def __init__(self):
        self.lock         = threading.Lock()
        self.latest_delta = None
        self.shutdown     = False


# ============================================================
# Receiver Thread
# ============================================================
def receiver_thread(state):
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((RECV_HOST, RECV_PORT))
    srv.listen(1)
    srv.settimeout(1.0)
    print "[Receiver] 대기 중... %s:%d" % (RECV_HOST, RECV_PORT)

    conn = None
    buf  = ''

    while not state.shutdown:
        if conn is None:
            try:
                conn, addr = srv.accept()
                conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                conn.settimeout(1.0)
                print "[Receiver] 연결됨: %s" % str(addr)
                buf = ''
            except socket.timeout:
                continue
            except Exception as e:
                if not state.shutdown:
                    print "[Receiver] accept 오류: %s" % str(e)
                continue

        try:
            chunk = conn.recv(1024)
            if not chunk:
                print "[Receiver] 연결 끊김"
                conn.close()
                conn = None
                continue
            buf += chunk

            while '\n' in buf:
                line, buf = buf.split('\n', 1)
                line = line.strip()
                if not line:
                    continue
                try:
                    recv_t = time.time()
                    d = json.loads(line)
                    if isinstance(d, list):
                        _dx, _dy, _dz, _drz = (float(d[0]), float(d[1]),
                                                float(d[2]), float(d[3]))
                        _cli_ts = None
                        _seq    = -1
                    else:
                        arr = d['delta']
                        _dx, _dy, _dz, _drz = (float(arr[0]), float(arr[1]),
                                                float(arr[2]), float(arr[3]))
                        _cli_ts = float(d['ts'])  if 'ts'  in d else None
                        _seq    = int(d['seq'])    if 'seq' in d else -1

                    delta = {
                        'dx': _dx, 'dy': _dy, 'dz': _dz, 'drz': _drz,
                        'ts': recv_t, 'cli_ts': _cli_ts, 'seq': _seq,
                    }
                    with state.lock:
                        state.latest_delta = delta
                except Exception as e:
                    print "[Receiver] 파싱 오류: %s" % str(e)

        except socket.timeout:
            pass
        except Exception as e:
            if not state.shutdown:
                print "[Receiver] 수신 오류: %s" % str(e)
            if conn:
                conn.close()
            conn = None

    if conn:
        try: conn.close()
        except Exception: pass
    try: srv.close()
    except Exception: pass
    print "[Receiver] 종료"


# ============================================================
# Worker Thread
# ============================================================
def worker_thread(rb, cmd_queue, state):
    print "[Worker] 시작"
    while not state.shutdown:
        try:
            # timeout=0.2 으로 shutdown 체크 가능하게
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
            state.shutdown = True
        except Exception:
            pass

    print "[Worker] 종료"


# ============================================================
# 안전 클램프
# ============================================================
def clamp(val, limit):
    if val >  limit: return  limit
    if val < -limit: return -limit
    return val


# ============================================================
# 제어 루프 (15Hz, 메인 스레드)
# ============================================================
def control_loop(rb, cmd_queue, state):
    print "[Control] 15Hz 루프 시작"

    next_tick     = time.time()
    last_log_t    = time.time()
    tick_count    = 0
    cmd_count     = 0
    skip_count    = 0
    dup_count     = 0
    last_cmd_t    = None
    last_seq      = -1
    cmd_intervals = []
    net_delays    = []
    fresh_delays  = []

    while not state.shutdown:
        with state.lock:
            delta = state.latest_delta

        now = time.time()

        if delta is None or (now - delta['ts']) > STALE_SEC:
            rb.abort()
            skip_count += 1
        else:
            dx  = clamp(delta['dx'],  MAX_DX)
            dy  = clamp(delta['dy'],  MAX_DY)
            dz  = clamp(delta['dz'],  MAX_DZ)
            drz = clamp(delta['drz'], MAX_DRZ)

            rb.abort()

            if not cmd_queue.empty():
                try: cmd_queue.get_nowait()
                except Exception: pass
            cmd_queue.put((dx, dy, dz, drz))

            if delta['cli_ts'] is not None:
                net_ms = (delta['ts'] - delta['cli_ts']) * 1000.0
                net_delays.append(net_ms)
                if len(net_delays) > 30: net_delays.pop(0)

            fresh_ms = (now - delta['ts']) * 1000.0
            fresh_delays.append(fresh_ms)
            if len(fresh_delays) > 30: fresh_delays.pop(0)

            seq = delta['seq']
            if seq != -1 and seq == last_seq:
                dup_count += 1
            last_seq = seq

            if last_cmd_t is not None:
                cmd_intervals.append(now - last_cmd_t)
                if len(cmd_intervals) > 30: cmd_intervals.pop(0)
            last_cmd_t = now
            cmd_count += 1

        tick_count += 1

        elapsed = now - last_log_t
        if elapsed >= 1.0:
            loop_hz = tick_count / elapsed
            cmd_hz  = cmd_count  / elapsed
            parts = ["loop=%.1fHz" % loop_hz,
                     "cmd=%.1fHz"  % cmd_hz,
                     "skip=%d"     % skip_count,
                     "dup=%d"      % dup_count]
            if cmd_intervals:
                mean_ms = sum(cmd_intervals) / len(cmd_intervals) * 1000.0
                min_ms  = min(cmd_intervals) * 1000.0
                max_ms  = max(cmd_intervals) * 1000.0
                parts.append("interval=%.1f/%.1f/%.1fms(mean/min/max)" % (
                    mean_ms, min_ms, max_ms))
            if net_delays:
                parts.append("net=%.1fms" % (sum(net_delays)/len(net_delays)))
            if fresh_delays:
                parts.append("fresh=%.1fms" % (sum(fresh_delays)/len(fresh_delays)))
            print "[Control] " + "  ".join(parts)
            tick_count = cmd_count = skip_count = dup_count = 0
            last_log_t = now

        # sleep을 잘게 쪼개서 shutdown 플래그 체크
        next_tick += CTRL_DT
        while not state.shutdown:
            remaining = next_tick - time.time()
            if remaining <= 0:
                break
            time.sleep(min(remaining, 0.05))

        if not state.shutdown and (time.time() - next_tick) > CTRL_DT:
            next_tick = time.time()

    print "[Control] 루프 종료"


# ============================================================
# 정상 종료 (rb.close() 까지 반드시 완료)
# ============================================================
def do_shutdown(state, cmd_queue, rb):
    print "\n[Main] 종료 중..."

    # 1. 모든 스레드에 종료 신호
    state.shutdown = True

    # 2. Worker의 cmd_queue.get() 블로킹 해제
    try: cmd_queue.put_nowait(None)
    except Exception: pass

    # 3. 로봇 동작 중단
    try:
        rb.abort()
    except Exception as e:
        print "[Main] abort 오류 (무시): %s" % str(e)

    # 4. Worker 스레드가 끝날 때까지 대기 (최대 1초)
    time.sleep(1.0)

    # 5. rb.close() - 여기서 rel_permission() 이 호출됨
    #    이게 호출돼야 다음 실행 시 E11 이 안 남
    try:
        rb.close()
        print "[Main] rb.close() 완료 (permission 반납됨)"
    except Exception as e:
        print "[Main] rb.close() 오류: %s" % str(e)

    print "[Main] 종료 완료"


# ============================================================
# 메인
# ============================================================
def main():
    print "=" * 60
    print "server_model_V3 - Zeus ZRA 15Hz Streaming Controller"
    print "=" * 60

    print "[Main] 로봇 연결 중..."
    try:
        rb = i611Robot()
        rb.open()
    except Exception as e:
        print "ERROR: %s" % str(e)
        sys.exit(1)
    print "[Main] 연결 완료"

    mp = MotionParam(lin_speed=LIN_SPEED, acctime=ACCTIME, dacctime=DACCTIME)
    rb.motionparam(mp)

    state     = SharedState()
    cmd_queue = Queue.Queue(maxsize=1)

    # SIGINT (Ctrl+C) 핸들러:
    # signal 핸들러는 메인 스레드에서만 실행되므로
    # state.shutdown = True 만 세팅하고 실제 종료는 메인 루프가 담당
    def sig_handler(signum, frame):
        print "\n[Main] Ctrl+C 감지"
        state.shutdown = True

    signal.signal(signal.SIGINT,  sig_handler)
    signal.signal(signal.SIGTERM, sig_handler)

    t_recv = threading.Thread(target=receiver_thread, args=(state,))
    t_recv.setDaemon(True)
    t_recv.start()

    t_work = threading.Thread(target=worker_thread, args=(rb, cmd_queue, state))
    t_work.setDaemon(True)
    t_work.start()

    # 제어 루프 실행 (state.shutdown=True 가 되면 루프 탈출)
    try:
        control_loop(rb, cmd_queue, state)
    except Exception as e:
        import traceback
        print "[Main] ERROR: %s" % str(e)
        traceback.print_exc()
        state.shutdown = True

    # 루프가 끝나면 반드시 정상 종료 수행
    do_shutdown(state, cmd_queue, rb)


if __name__ == '__main__':
    main()
