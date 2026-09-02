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

def check_gripper():
    a,b,c,d = din(48), din(49), din(50), din(51)
    return [d,c,b,a]

def gripper(onoff):
    dout(48,'0000')
    if onoff == 'open':
	while check_gripper() != ['0','1','0','0']:
	    dout(48,'0100')
    
    elif onoff == 'close':
	while check_gripper() != ['0', '0','0','1']:
	    dout(48,'0001')
    else:
	exit(0)
	
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
    up = Joint(47.52, -68.28, -30.57,-180.00,  81.15,  47.52)
    move_x = Position(-673.41, 501.04, 234.27, -90.00,  -0.00,-180.00)
    rb.move(up)
    rb.line(move_x)
    

def place_block(pose):
    print "[SERVER] Move robot to place block:", pose
    gripper('open')
    rb.move(pose)
    gripper('close')

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

        # --- 블록 �get ---
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

        # --- stopover ---
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

