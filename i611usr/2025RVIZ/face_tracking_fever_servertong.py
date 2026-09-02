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


class FaceTrackingFeverServer:
    """외부 rb 객체를 받아서 실행하는 얼굴 추적 + 열 측정 서버"""
    

    def __init__(self, rb, log_filename="face_tracking_server_log.txt"):
        # 로그 파일 설정
        sys.stdout = Logger(log_filename)

        # 외부에서 전달받은 로봇 객체
        self.rb = rb
        self.INIT_JOINT = Joint(65.09, 0.78, -119.57, -44.25, 37.21, -52.32)
        # 서버 초기화 (시작 신호는 run()에서 보냄)
        self.server = RobotEnvServer()
        self.server.start()
        print "[SERVER] Socket opened — waiting for start command later..."

    # ------------------------
    # 얼굴 좌표 트래킹 동작
    # ------------------------
    def track_face(self, x, y, z):
        """얼굴 좌표(X, Y, Z)를 받아서 로봇 이동 (부드럽게 추종)"""
        try:
            m = MotionParam(jnt_speed=20, lin_speed=150, overlap=50)
            self.rb.motionparam(m)

            target = Position(x, y, z, -90, 0, 90)
            print "[SERVER] Tracking to:", (x, y, z)

            self.rb.asyncm(1)      # 비동기 모드 → 끊김 없이 이동
            self.rb.line(target)   # 직선 경로 이동
            return True

        except Exception, e:
            print "[SERVER] Error in track_face:", e
            return False

    # ------------------------
    # 메인 루프
    # ------------------------
    def main(self):
        rb = self.rb
        server = self.server

        print "Start main loop"
        last_move_time = 0
        client_ready = True

        while True:
            # --- 클라이언트 준비됐다면 주기적으로 face_detect 요청 ---
            if client_ready:
                server.send_command({"action": "face_detect"})
                time.sleep(0.2)
		print "[SERVER]send"
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

            # --- 얼굴 좌표 수신 ---
            elif action == "face_detect":
                x = data.get("x")
                y = data.get("y")
                z = data.get("z", 500)

                # === 온도, 고열 판단 여부 수신 ===
                temp = data.get("temperature")
                fever = data.get("fever", False)

                if temp is not None:
                    print "[SERVER] Temperature: %.2f °C" % temp
                    if fever:
                        print "[SERVER] !!! 고열로 감지되어 약 탐색모드로 전환"
                        # TODO: 고열 시 약 탐색 모드 실행
                # ============================

                if x is not None and y is not None:
                    now = time.time()
                    if now - last_move_time < 0.1:
                        continue
                    last_move_time = now

                    x_offset = x + 200   # 오프셋 적용
                    print "[SERVER] Face detected at (%s, %s, %s)" % (x_offset, y, z)

                    success = self.track_face(x_offset, y, z)

                    try:
                        rb.changetool(2)
                        current_pose = rb.getpos()
                        pose_dict = current_pose.pos2dict()
                        rb.changetool(1)
                        if "parent" in pose_dict:
                            del pose_dict["parent"]
                    except Exception, e:
                        print "[SERVER] Error getting robot pose:", e
                        pose_dict = None

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

            elif action == "stop":
                print "[SERVER] Stop command received from client"
                break

            else:
                print "[SERVER] Unknown or unhandled action:", action, status

    # ------------------------
    # 실행부
    # ------------------------
    def run(self):
        try:
	    print "[SERVER] Sending start command to client..."
            self.server.send_command({"action": "start"})
            self.rb.motionparam(MotionParam(jnt_speed=20, lin_speed=100, overlap=20))
            print "[SERVER] Moving to initial pose..."
            self.rb.move(self.INIT_JOINT)
            time.sleep(1)
            print "[SERVER] Waiting for client ready..."
            self.main()
        except KeyboardInterrupt:
            print 'KeyboardInterrupt'
        except Exception, e:
            print 'error:', e.__class__.__name__, ':', e
        finally:
            self.server.close()
            print "[SERVER] Closed cleanly "

