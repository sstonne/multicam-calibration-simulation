# -*- coding: utf-8 -*-
#!/usr/bin/python

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
import select  
from server_comm import RobotEnvServer   # 서버 통신 클래스

# ------------------------
# 로그 기록
# ------------------------
log_filename = "face_tracking_server_log.txt"
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

# ------------------------
# 서버 시작
# ------------------------
server = RobotEnvServer()
server.start()
server.send_command({"action": "start"})
print "[SERVER] Waiting for client ready..."

# ------------------------
# 얼굴 좌표 트래킹 동작
# ------------------------
def track_face(rb, x, y, z):
    """ 얼굴 좌표(X, Y, Z)를 받아서 로봇 이동 (부드럽게 추종) """
    try:
        m = MotionParam(jnt_speed=20, lin_speed=150, overlap=70)
        rb.motionparam(m)

        target = Position(x, y, z, -90, 0, 90)
        print "[SERVER] Tracking to:", (x, y, z)

        rb.asyncm(1)      # 비동기 모드 → 끊김 없이 이동
        rb.line(target)   # 직선 경로 이동

        return True
    except Exception, e:
        print "[SERVER] Error in track_face:", e
        return False

# ------------------------
# 메인 루프
# ------------------------
def main(rb):
    print "Start main loop"
    last_move_time = 0
    client_ready = False
    rb.settool(1, 0.0, 0.0, 245.0, 0.0, 0.0, 0.0) #이거 수정 들어가야될 것 같음
    rb.changetool(1)
    rb.settool(2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    try:
        print "[SERVER] Moving to initial pose..."
        m = MotionParam(jnt_speed=20, lin_speed=150, overlap=0)
        rb.motionparam(m)

        # 조인트 값(J1~J6)을 네 로봇 기준으로 바꿔줘야 함
        # 예: home pose
        init_pose = Joint(65.09, 0.78, -119.57, -44.25, 37.21, -52.32)  # 예시 (각도 단위: deg)

        rb.move(init_pose)
        print "[SERVER] Reached initial pose."
    except Exception, e:
        print "[SERVER] Failed to move to initial pose:", e

    while True:
        # --- 클라이언트 준비됐다면 주기적으로 face_detect 요청 ---
        if client_ready:
            server.send_command({"action": "face_detect"})
            time.sleep(0.2)

        # --- 클라이언트 응답 확인 (select로 non-blocking 체크) ---
        try:
            rlist, _, _ = select.select([server.conn], [], [], 0.01)
            if not rlist:
                continue
            data = server.receive_data()
        except Exception, e:
            print "[SERVER] Error receiving data:", e
            continue

        if not data:
            continue

        action = data.get("action")
        status = data.get("status", "")

        # --- 최초 연결 ---
        if action == "start" and status == "ready":
            print "[SERVER] Client ready → waiting for face data"
            client_ready = True
            
        elif action == "stop":
            print "[SERVER] Stop signal received from client → shutting down."
            server.send_command({"action": "stop_ack"})
            break
        # --- 얼굴 좌표 수신 ---
        elif action == "face_detect":
            x = data.get("x")
            y = data.get("y")
            z = data.get("z", 500)

            # === 온도, 고열 판단 여부 수신 ===
            temp = data.get("temperature")
            fever = data.get("fever", False)

            if temp is not None:
                print "[SERVER] Temperature: {:.2f} °C".format(temp)
                if fever:
                    print "[SERVER] !!! 고열로 감지되었습니다."
                # --- 클라이언트에서 stop 신호 수신 ---
        

            # ============================

            if x is not None and y is not None:
                now = time.time()
                if now - last_move_time < 0.1:
                    continue
                last_move_time = now

                x_offset = x + 200   # 오프셋 적용
                print "[SERVER] Face detected at ({}, {}, {})".format(x_offset, y, z)

                success = track_face(rb, x_offset, y, z)

                try:
                    rb.changetool(2)
                    current_pose = rb.getpos() # 이거 수정함 원래 rb.where()
                    pose_dict = current_pose.pos2dict()
                    rb.changetool(1)

                    if "parent" in pose_dict:
                        del pose_dict["parent"]   # Base 객체 제거
                    
                except Exception, e:
                    print "[SERVER] Error getting robot pose:", e
                    current_pose = None

                if success:
                    server.send_command({
                        "action": "face_track",
                        "status": "success",
                        "target": {"x": x_offset, "y": y, "z": z},
                        "robot_pose": pose_dict,
                        "temperature": temp,
                        "fever": fever
                    })
                else:
                    server.send_command({
                        "action": "face_track",
                        "status": "fail",
                        "robot_pose": pose_dict,
                        "temperature": temp,
                        "fever": fever
                    })
            else:
                print "[SERVER] Invalid face data:", data

        else:
            # Unknown action 로그 추가
            print "[SERVER] Unknown or unhandled action:", action, status

# ------------------------
# 실행부
# ------------------------
if __name__ == '__main__':
    try:
        rb = i611Robot()
        _BASE = Base()
        rb.open()
     

        main(rb)

        server.close()
        rb.close()
    except KeyboardInterrupt:
        print 'KeyboardInterrupt'
        rb.exit(0)
        rb.close()
    except Exception, e:
        print 'error: ', e.__class__.__name__, ':', e
        rb.exit(0)
        rb.close(0)


