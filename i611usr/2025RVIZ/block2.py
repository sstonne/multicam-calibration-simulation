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
import sys
import json
from robot_env_comm import RobotEnvServer

# 로그 기록
log_filename = "block_server_log.txt"
class Logger(object):
    def __init__(self, filename):
        self.terminal = sys.stdout
        self.log = open(filename, "w")
    def write(self, message):
        self.terminal.write(message)
        self.log.write(message)
    def flush(self):
        self.terminal.flush()
        self.log.flush()
sys.stdout = Logger(log_filename)

# 서버 시작
server = RobotEnvServer()
server.start()
server.send_command({"action":"start"})
print("[SERVER] Waiting for client ready...")

# 로봇 동작 (데모 출력용)
def pick_block(pose):
    print "[SERVER] Move robot to pick block:", pose

def stopover(pose):
    print "[SERVER] Stopover before place:", pose

def place_block(pose):
    print "[SERVER] Move robot to place block:", pose

def main():
    print "Start main loop"
    while True:
        data = server.receive_data()
        if not data:
            print("No data from client.")
            continue

        action = data.get("action")
        status = data.get("status", "")

        # --- 최초 연결 ---
        if action == "start" and status == "ready":
            print "[SERVER] Client ready → request first block"
            server.send_command({"action":"pick_block"})

        # --- 블록 좌표 받음 ---
        elif action == "pick_block" and status == "success":
            target = data.get("target", {})
            color = data.get("color")
            print "[SERVER] Pick target from client:", target, "color:", color
            pick_block(target)
            server.send_command({"action":"stopover"})

        # --- 블록 없을 때 ---
        elif action == "pick_block" and status == "fail":
            print "[SERVER] No more blocks. Shutting down."
            break

        # --- stopover 경유 ---
        elif action == "stopover" and status == "success":
            target = data.get("target", {})
            print "[SERVER] Stopover target from client:", target
            stopover(target)
            server.send_command({"action":"place_block"})

        # --- place 블록 ---
        elif action == "place_block" and status == "success":
            target = data.get("target", {})
            print "[SERVER] Place target from client:", target
            place_block(target)
            # 완료 후 다시 블록 요청
            server.send_command({"action":"pick_block"})

        else:
            print "[SERVER] Unknown or unhandled action:", action, status

if __name__ == '__main__':
    try:
        rb = i611Robot()
        _BASE = Base()
        rb.open()
        main()
        server.close()
    except KeyboardInterrupt:
        print('KeyboardInterrupt')
        rb.exit(0)
        rb.close()
    except Exception, e:
        print('error: ', e.__class__.__name__, ':', e)
        rb.exit(0)
        rb.close(0)

