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


def comm(server):
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

   
class FaceTrackingFeverServer:
    """외부 rb 객체를 받아서 실행하는 얼굴 추적 + 열 측정 서버"""
    
    def __init__(self, rb, server, log_filename="face_tracking_server_log.txt"):
        # 로그 파일 설정
        sys.stdout = Logger(log_filename)

        self.rb = rb
        self.server = server
        self.INIT_JOINT = Joint(65.09, 0.78, -119.57, -44.25, 37.21, -52.32)

    # ------------------------
    # 얼굴 좌표 트래킹 동작
    # ------------------------
    def track_face(self, x, y, z):
        """얼굴 좌표(X, Y, Z)를 받아서 로봇 이동 (부드럽게 추종)"""
        try:
            m = MotionParam(jnt_speed=20, lin_speed=150, overlap=70)
            self.rb.motionparam(m)

            target = Position(x, y, z, -90, 0, 90)
            print "[SERVER] Tracking to:", (x, y, z)

            self.rb.asyncm(1)      # 비동기 모드 → 끊김 없이 이동
            self.rb.line(target)   # 직선 경로 이동
            return True

        except Exception, e:
            print "[SERVER] Error in track_face:", str(e)
            return False

    # ------------------------
    # 메인 루�
    # ------------------------
    def main(self):
        #server = self.server
        print "Start facetraking main loop"
	test = Joint( 65.96,  -0.54,-122.31, -39.69,  39.73, -57.57)
	self.rb.move(test)

        last_move_time = 0
        client_ready = False

        self.server.send_command({"action": "facetracking_start"})
        data  = comm(self.server)
        action = data.get("action")

        if action == "facetracking_start":
            client_ready = True
            print '[SERVER] client ready!'

        while True:
            # --- 클라이언트 준비됐다면 주기적으로 face_detect 요청 ---
            if client_ready:
	        print '[SERVER] client ready, face tracking...'
                #rb.changetool(2)
                current_pose = self.rb.getpos()
                pose_list = current_pose.pos2list()
                pose_list = pose_list[0:6]
                #rb.changetool(1)
                
                self.server.send_command({"action": "face_detect", "robot_pose":pose_list})
            
                face_tracking_data = comm(self.server)
                action = face_tracking_data.get("action")
                face_position = face_tracking_data.get("face_position")
         	last_time = face_tracking_data.get("time")
                
                if action == "face_detect" and last_time < 30:
                    x = face_tracking_data.get("x")
                    y = face_tracking_data.get("y")
                    z = face_tracking_data.get("z")
		    print '[server] x,y,z', x, y, z
        	    if x == 0:
			print('no face')	 
			continue
	            else:
                        x_offset = x + 500   # 오프셋 적용
                        print "[SERVER] Face detected at (%s, %s, %s)" % (x_offset, y, z)
                        m = MotionParam(jnt_speed=20, lin_speed=150, overlap=70)
	                self.rb.motionparam(m)

	                target = Position(x_offset, y, z, -90, 0, 90)
           		print "[SERVER] Tracking to:", (x, y, z)

                        self.rb.asyncm(1)      # 비동기 모드 → 끊김 없이 이동
                        self.rb.line(target)   # 직선 경로 이동
                        action = "face_detecting"
		else:
		    print "[SERVER] 10 초 지남"
		    break



    # ------------------------
    # 실행부
    # ------------------------
    def run(self):
        try:
            print "[SERVER] Sending start command to client..."
            self.rb.motionparam(MotionParam(jnt_speed=20, lin_speed=100, overlap=20))
            print "[SERVER] Moving to initial pose..."
            self.rb.move(self.INIT_JOINT)
            time.sleep(1)
            print "[SERVER] Waiting for client ready..."
            self.main()
        except KeyboardInterrupt:
            print 'KeyboardInterrupt'
        except Exception, e:
            print 'error:', e.__class__.__name__, ':', str(e)
        finally:
	    print 'face tracking close..'
 
