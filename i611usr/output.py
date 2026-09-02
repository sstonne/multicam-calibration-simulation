#!/usr/bin/python
# -*- coding: utf-8 -*-
"""
server_output.py : 로봇 EE 상태를 15Hz로 클라이언트에 전송

변경 사항:
  - time.sleep(0.066) 방식 → next_tick 기반 정밀 15Hz 제어
    (sleep(0.066)은 OS 스케줄링 오차로 실제 Hz가 불안정함)
  - 전송 간격 실측 및 1초마다 Hz 로그 출력
  - 전송 데이터에 서버 타임스탬프(ts) 추가
    (클라이언트에서 네트워크 지연 측정 가능)
"""

import os
import time
import math
import socket
import json

from i611shm import shm_read

HOST     = '0.0.0.0'
PORT     = 12348
LOG_PATH = "zeus_pose_log.csv"

TARGET_HZ = 15
TARGET_DT = 1.0 / TARGET_HZ   # 66.7ms


# ──────────────────────────────────────────────────────────────────────────────
def read_position_info():
    """shm에서 EE pose 읽기. 반환: [x_mm, y_mm, z_mm, rz_deg, ry_deg, rx_deg]"""
    info = shm_read(0x3000, 20).split(',')
    return [
        float(info[0]) * 1000.0,          # X: m → mm
        float(info[1]) * 1000.0,          # Y
        float(info[2]) * 1000.0,          # Z
        math.degrees(float(info[3])),     # Rz: rad → deg
        math.degrees(float(info[4])),     # Ry
        math.degrees(float(info[5])),     # Rx
    ]


def send_position(conn, pos, ts):
    """JSON 한 줄로 전송. ts = 서버 송신 시각 (클라이언트 지연 측정용)"""
    data = {
        'x':  pos[0], 'y':  pos[1], 'z':  pos[2],
        'rz': pos[3], 'ry': pos[4], 'rx': pos[5],
        'ts': ts,     # 추가: 서버 송신 타임스탬프
    }
    conn.sendall((json.dumps(data) + '\n').encode('utf-8'))


def open_log(path):
    need_header = (not os.path.exists(path)) or (os.path.getsize(path) == 0)
    f = open(path, "a", 1024 * 1024)
    if need_header:
        f.write("t_epoch_s,dt_ms,hz,x_mm,y_mm,z_mm,rz_deg,ry_deg,rx_deg\n")
        f.flush()
    return f


# ──────────────────────────────────────────────────────────────────────────────
def main():
    server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_sock.bind((HOST, PORT))
    server_sock.listen(1)
    print "Server listening on {}:{} ...".format(HOST, PORT)

    conn, addr = server_sock.accept()
    print "Client connected: {}".format(addr)
    try:
        conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    except Exception:
        pass

    logfile      = open_log(LOG_PATH)
    last_flush_t = time.time()
    last_log_t   = time.time()

    # 타이밍
    next_tick    = time.time()
    prev_send_t  = None
    intervals    = []   # 최근 30샘플
    overruns     = 0
    tick_count   = 0

    try:
        while True:
            t = time.time()

            # ── 로봇 상태 읽기 ────────────────────────────────────────────────
            pos = read_position_info()

            # ── 전송 ─────────────────────────────────────────────────────────
            send_position(conn, pos, t)

            # ── 전송 간격 측정 ────────────────────────────────────────────────
            if prev_send_t is not None:
                interval_ms = (t - prev_send_t) * 1000.0
                intervals.append(interval_ms)
                if len(intervals) > 30:
                    intervals.pop(0)
            prev_send_t = t
            tick_count += 1

            # ── CSV 로그 ──────────────────────────────────────────────────────
            dt_ms = intervals[-1] if intervals else 0.0
            hz    = 1000.0 / dt_ms if dt_ms > 0 else 0.0
            logfile.write("{:.6f},{:.3f},{:.2f},{:.3f},{:.3f},{:.3f},{:.3f},{:.3f},{:.3f}\n".format(
                t, dt_ms, hz,
                pos[0], pos[1], pos[2], pos[3], pos[4], pos[5]
            ))
            if (t - last_flush_t) >= 1.0:
                logfile.flush()
                last_flush_t = t

            # ── 1초마다 Hz 로그 ───────────────────────────────────────────────
            elapsed = t - last_log_t
            if elapsed >= 1.0:
                actual_hz = tick_count / elapsed
                if intervals:
                    mean_ms = sum(intervals) / len(intervals)
                    min_ms  = min(intervals)
                    max_ms  = max(intervals)
                    interval_str = "mean=%.1fms min=%.1fms max=%.1fms" % (
                        mean_ms, min_ms, max_ms)
                else:
                    interval_str = "n/a"
                print "[server_output] hz=%.1f  %s  overrun=%d" % (
                    actual_hz, interval_str, overruns)
                tick_count = overruns = 0
                last_log_t = t

            # ── 정밀 15Hz sleep (next_tick 기반) ─────────────────────────────
            next_tick += TARGET_DT
            sleep_time = next_tick - time.time()
            if sleep_time > 0:
                time.sleep(sleep_time)
            else:
                # 오버런: 처리가 TARGET_DT 초과
                overruns += 1
                next_tick = time.time()

    except KeyboardInterrupt:
        print "Server stopped."
    except Exception as e:
        print "Exception:", e
    finally:
        try:
            logfile.flush()
            logfile.close()
        except Exception:
            pass
        try:
            conn.close()
        except Exception:
            pass
        try:
            server_sock.close()
        except Exception:
            pass


if __name__ == "__main__":
    main()
