#!/usr/bin/env python
# -*- coding: utf-8 -*-
u"""
server.py  -  Zeus ZRA 통합 서버 (단일 포트, 요청/응답)

[사이클]
  클라이언트가 delta를 보내면:
    1. delta 파싱
    2. abort()
    3. relline 투입 → Worker
    4. shm 읽기
    5. 좌표 응답
  → 15Hz 타이밍은 클라이언트가 결정. 서버는 오는 대로 처리.

[Ctrl+C 종료 순서]
  sig_handler
    → state.shutdown = True
    → srv 소켓 close  (accept() blocking 즉시 해제)
    → conn 소켓 close (recv()   blocking 즉시 해제)
  메인루프 탈출
    → do_shutdown()
        → cmd_queue.put(None)  (Worker get() 해제)
        → rb.abort()
        → t_work.join(3s)
        → rb.close()
"""

import math
import os
import socket
import threading
import time
import json
import sys
import Queue
import signal

from i611_MCS import i611Robot, MotionParam
from i611_common import Robot_stop, Robot_emo
from i611shm import shm_read

# ============================================================
# 설정
# ============================================================
HOST     = '0.0.0.0'
PORT     = 12348

LIN_SPEED  = 10.0
ACCTIME    = 0.01
DACCTIME   = 0.001

MAX_DX   = 20.0
MAX_DY   = 20.0
MAX_DZ   = 20.0
MAX_DRZ  = 5.56

LOG_PATH = "zeus_pose_log.csv"


# ============================================================
# 공유 상태
# ============================================================
class State(object):
    def __init__(self):
        self.shutdown = False


# ============================================================
# 유틸
# ============================================================
def read_ee_pose():
    info = shm_read(0x3000, 20).split(',')
    return [
        float(info[0]) * 1000.0,
        float(info[1]) * 1000.0,
        float(info[2]) * 1000.0,
        math.degrees(float(info[3])),
        math.degrees(float(info[4])),
        math.degrees(float(info[5])),
    ]


def clamp(val, limit):
    if val >  limit: return  limit
    if val < -limit: return -limit
    return val


def open_log(path):
    need_header = (not os.path.exists(path)) or (os.path.getsize(path) == 0)
    f = open(path, "a", 1024 * 1024)
    if need_header:
        f.write("t_epoch_s,x_mm,y_mm,z_mm,rz_deg,ry_deg,rx_deg\n")
        f.flush()
    return f


# ============================================================
# Worker Thread  (relline 블로킹 격리)
# ============================================================
def worker_thread(rb, cmd_queue, state):
    print "[Worker] 시작"
    while not state.shutdown:
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
            state.shutdown = True
        except Exception as e:
            print "[Worker] relline 오류: %s" % str(e)
    print "[Worker] 종료"


# ============================================================
# 세션 처리  (클라이언트 1개)
# ============================================================
def handle_session(conn, rb, cmd_queue, state):
    print "[Session] 시작"

    # recv() 가 Ctrl+C 시 0.5s 안에 풀리도록
    conn.settimeout(0.5)

    logfile      = open_log(LOG_PATH)
    last_flush_t = time.time()
    last_log_t   = time.time()

    buf          = b''
    step         = 0
    last_seq     = -1
    dup_count    = 0
    skip_count   = 0
    cycle_times  = []
    last_cycle_t = None

    try:
        while not state.shutdown:
            # ── (1) 한 줄 수신 (\n 까지) ──────────────────────
            while b'\n' not in buf:
                if state.shutdown:
                    return
                try:
                    chunk = conn.recv(1024)
                except socket.timeout:
                    continue          # shutdown 체크 후 재시도
                except Exception as e:
                    print "[Session] recv 오류: %s" % str(e)
                    return
                if not chunk:
                    print "[Session] 클라이언트 연결 끊김"
                    return
                buf += chunk

            # 사이클 타이밍 측정
            t_cycle = time.time()
            if last_cycle_t is not None:
                cycle_times.append((t_cycle - last_cycle_t) * 1000.0)
                if len(cycle_times) > 30:
                    cycle_times.pop(0)
            last_cycle_t = t_cycle

            line, buf = buf.split(b'\n', 1)
            line = line.strip()

            dx = dy = dz = drz = 0.0
            cli_ts = None
            seq    = -1
            valid  = False

            if line:
                try:
                    d = json.loads(line.decode('utf-8'))
                    arr    = d['delta']
                    dx, dy, dz, drz = (float(arr[0]), float(arr[1]),
                                       float(arr[2]), float(arr[3]))
                    cli_ts = float(d['ts'])  if 'ts'  in d else None
                    seq    = int(d['seq'])   if 'seq' in d else -1
                    valid  = True
                except Exception as e:
                    print "[Session] 파싱 오류: %s  line=%r" % (str(e), line[:80])

            # ── (2) abort ─────────────────────────────────────
            rb.abort()

            # ── (3) relline 투입 ──────────────────────────────
            if valid:
                dx  = clamp(dx,  MAX_DX)
                dy  = clamp(dy,  MAX_DY)
                dz  = clamp(dz,  MAX_DZ)
                drz = clamp(drz, MAX_DRZ)

                # 큐가 차 있으면 오래된 명령 버림
                try: cmd_queue.get_nowait()
                except Queue.Empty: pass
                cmd_queue.put((dx, dy, dz, drz))

                if seq != -1 and seq == last_seq:
                    dup_count += 1
                last_seq = seq
            else:
                skip_count += 1

            # ── (4) EE 좌표 읽기 ──────────────────────────────
            now = time.time()
            try:
                pos = read_ee_pose()
            except Exception as e:
                print "[Session] shm 오류: %s" % str(e)
                pos = [0.0] * 6

            # ── (5) 좌표 응답 ─────────────────────────────────
            net_ms = (now - cli_ts) * 1000.0 if cli_ts else 0.0
            resp = json.dumps({
                'x': pos[0], 'y': pos[1], 'z': pos[2],
                'rz': pos[3], 'ry': pos[4], 'rx': pos[5],
                'ts': now,
                'net_ms': round(net_ms, 2),
            }) + '\n'
            try:
                conn.sendall(resp.encode('utf-8'))
            except Exception as e:
                print "[Session] 전송 오류: %s" % str(e)
                return

            # ── CSV 로그 ──────────────────────────────────────
            logfile.write("{:.6f},{:.3f},{:.3f},{:.3f},{:.3f},{:.3f},{:.3f}\n".format(
                now, pos[0], pos[1], pos[2], pos[3], pos[4], pos[5]))
            if now - last_flush_t >= 1.0:
                logfile.flush()
                last_flush_t = now

            step += 1

            # ── 1초마다 로그 ──────────────────────────────────
            elapsed = now - last_log_t
            if elapsed >= 1.0:
                hz = step / elapsed
                if cycle_times:
                    timing = "%.1f/%.1f/%.1fms(mean/min/max)" % (
                        sum(cycle_times)/len(cycle_times),
                        min(cycle_times), max(cycle_times))
                else:
                    timing = "n/a"
                print "[Server] hz=%.1f  cycle=%s  skip=%d  dup=%d" % (
                    hz, timing, skip_count, dup_count)
                step = skip_count = dup_count = 0
                last_log_t = now

    finally:
        try: logfile.flush(); logfile.close()
        except Exception: pass
        print "[Session] 종료"


# ============================================================
# 종료
# ============================================================
def do_shutdown(state, cmd_queue, rb, t_work):
    print "[Shutdown] 시작..."

    # Worker 종료 신호
    state.shutdown = True
    try: cmd_queue.put_nowait(None)
    except Exception: pass

    # 로봇 동작 중단
    try:
        rb.abort()
        print "[Shutdown] abort() 완료"
    except Exception as e:
        print "[Shutdown] abort() 오류: %s" % str(e)

    # Worker 스레드 종료 대기
    print "[Shutdown] Worker 종료 대기 중..."
    t_work.join(timeout=3.0)
    if t_work.is_alive():
        print "[Shutdown] Worker 강제 종료 (3s 초과)"
    else:
        print "[Shutdown] Worker 정상 종료"

    # 로봇 연결 해제 (permission 반납 → E11 방지)
    try:
        rb.close()
        print "[Shutdown] rb.close() 완료"
    except Exception as e:
        print "[Shutdown] rb.close() 오류: %s" % str(e)

    print "[Shutdown] 완료"


# ============================================================
# 메인
# ============================================================
def main():
    print "=" * 60
    print "server.py  -  Zeus ZRA 통합 서버"
    print "  port %d : 단일 포트 (delta 수신 / 좌표 응답)" % PORT
    print "=" * 60

    # 로봇 연결
    try:
        rb = i611Robot()
        rb.open()
    except Exception as e:
        print "ERROR: 로봇 연결 실패: %s" % str(e)
        sys.exit(1)
    print "[Main] 로봇 연결 완료"

    mp = MotionParam(lin_speed=LIN_SPEED, acctime=ACCTIME, dacctime=DACCTIME)
    rb.motionparam(mp)

    state     = State()
    cmd_queue = Queue.Queue(maxsize=1)

    # 소켓 참조 (sig_handler에서 강제 close용)
    _srv  = [None]
    _conn = [None]

    def sig_handler(signum, frame):
        print "\n[Main] Ctrl+C - 종료 시작"
        state.shutdown = True
        # accept() / recv() blocking 즉시 해제
        if _srv[0]:
            try: _srv[0].close()
            except Exception: pass
        if _conn[0]:
            try: _conn[0].close()
            except Exception: pass

    signal.signal(signal.SIGINT,  sig_handler)
    signal.signal(signal.SIGTERM, sig_handler)

    # Worker 스레드 (데몬 아님 → join으로 확실히 종료)
    t_work = threading.Thread(target=worker_thread, args=(rb, cmd_queue, state))
    t_work.daemon = False
    t_work.start()

    # 서버 소켓
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((HOST, PORT))
    srv.listen(1)
    srv.settimeout(1.0)
    _srv[0] = srv
    print "[Main] 클라이언트 대기 중... %s:%d" % (HOST, PORT)

    while not state.shutdown:
        try:
            conn, addr = srv.accept()
        except socket.timeout:
            continue
        except Exception:
            # sig_handler가 srv.close() 했을 때 여기서 빠져나옴
            break

        conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        _conn[0] = conn
        print "[Main] 클라이언트 연결됨: %s" % str(addr)

        handle_session(conn, rb, cmd_queue, state)

        _conn[0] = None
        try: conn.close()
        except Exception: pass

        if not state.shutdown:
            print "[Main] 재연결 대기 중..."

    try: srv.close()
    except Exception: pass

    do_shutdown(state, cmd_queue, rb, t_work)


if __name__ == '__main__':
    main()
