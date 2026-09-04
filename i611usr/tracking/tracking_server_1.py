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
from server_comm import RobotEnvServer

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

def comm():
    raw = server.receive_data()
    if not raw:
        print("No data from client.")
        return None
    if isinstance(raw, basestring): 
        try:
            data = json.loads(raw)
        except Exception as e:
            print("JSON decode error:", e, raw)
            return None
    else:
        data = raw
    return data

def send_gripper_and_wait(server, state, mode="full", speed=255, timeout=3.0, poll=0.05):
    global gripper_state

    cmd = {"action": "gripper", "state": state, "mode": mode, "speed":speed}
    server.send_command(cmd)
    print "[SERVER] ->", cmd

    t0 = time.time()
    while time.time() - t0 < timeout:
        data = comm()
        if not data:
            time.sleep(poll)
            continue

        print "[SERVER] <-", data
        if data.get("action") == "gripper":
            status = data.get("status")
            gripper_state = data.get("gripper_state")

            if status in ("end", "notConnected", "error"):
                return data  # 최종 응답
        time.sleep(poll)

    return {"status": "error", "action": "gripper", "detail": "timeout"}

def main():
    start_cmd = {"action":"start"}
    server.send_command(start_cmd)
    data = comm()

if __name__ == '__main__':
    try:
        rb = i611Robot()
        _BASE = Base()
        rb.open()
	IOinit(rb)

	m = MotionParam(jnt_speed=100, lin_speed=150, pose_speed=100, overlap=20, acctime=0.4, dacctime=0.4)
	rb.motionparam(m)                                                                                              
        rb.override(80)                                                                                                
                                                                                                                               
        rb.settool(1,0.0,0.0, 0.0, 0.0, 0.0, 0.0)                                            
        rb.changetool(1)                                                                        
                                      
        # start server
	server = RobotEnvServer()
	server.start()
	print("[SERVER] Waiting for client ready...")
	
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

