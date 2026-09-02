# -*- coding: utf-8 -*-
import socket
import json
import time
from i611_MCS import *
from i611_io import *

HOST = '0.0.0.0'
PORT = 12346
position_list = []
def send_command_to_client(conn, command_message):                
    try:                                             
        conn.sendall(command_message.encode('utf-8'))             
        print "Command sent to client: {}".format(command_message)
                                                     
    except socket.error as e:                                     
        print "Error sending command to client: {}".format(e)

def receive_data_from_client(conn):                               
        try:                                                      
                data = conn.recv(4096).decode('utf-8')
                if data:                                          
                        messages = data.splitlines()                       
                        for message in messages:                                                   
                                try:                                                               
                                        received_data = json.loads(message)
                                        print "Received data from client: {}".format(received_data)
                                        return received_data                                       
                                except json.JSONDecodeError as e:          
                                        print("JSON decod")                                        
                                                                                                   
        except socket.error as e:                                          
                print "Error receiving data from client : {}".format(e)                            
        return None  

def get_position():
    """ 현재 로봇의 TCP 좌표를 리스트로 반환 """
    pose = rb.getpos()
    position_values = pose.pos2list()  # ✅ pos2list() 사용하여 리스트 변환
    return position_values[:3]  # ✅ x, y, z 값만 반환

def move_robot(target_joints):
    """ 로봇을 지정된 joint 위치로 이동 """
    rb.move(target_joints)
    time.sleep(0.2)  # ✅ 속도 최적화

def get_robot_tcp():
    """ 현재 로봇 TCP 위치 반환 """
    curr_pos = get_position()  # ✅ 리스트 반환
    return curr_pos  # ✅ [x, y, z] 형태로 반환

def start_server():
    global position_list
    """ 클라이언트와 소켓 통신 및 로봇 제어 """
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind((HOST, PORT))
    s.listen(1)
    print "서버 실행 중... 클라이언트 연결 대기 중"

    conn, addr = s.accept()
    print "클라이언트 연결됨: {}".format(addr)

    # ✅ 속도 조절 추가
    m = MotionParam(jnt_speed=10, lin_speed=10, pose_speed=20, overlap=20, acctime=0.2, dacctime=0.2)
    rb.motionparam(m)
    rb.override(10) 
    rb.use_mt(True)

    # 실제 로봇 이동 좌표 리스트
    target_joints = [
        Joint(78.36, 15.67, 103.00, -9.07, 151.07, -125.65),
        Joint(79.54, 15.70, 103.65, -6.51, 150.53, -123.38),
        Joint(80.93, 16.81, 102.84, -3.64, 150.34, -120.87),
        Joint(83.37, 16.98, 103.93, 1.24, 149.13, -116.64),
        Joint(83.22, 15.92, 105.36, 0.94, 148.76, -116.90),
        Joint(83.22, 17.23, 107.12, 0.87, 145.69, -116.99),
        Joint(83.37, 19.66, 107.28, 1.06, 143.09, -116.86),
        Joint(84.62, 19.77, 107.66, 3.10, 142.57, -115.25),
        Joint(88.44, 20.20, 108.55, 9.08, 140.94, -110.64),
        Joint(88.42, 21.70, 109.93, 8.53, 138.10, -111.34),
        Joint(88.29, 20.56, 115.72, 7.66, 133.51, -112.42),
        Joint(89.63, 23.53, 117.64, 8.83, 128.54, -112.19),
        Joint(92.46, 22.24, 120.77, 12.12, 126.41, -110.45),
        Joint(92.45, 23.26, 121.30, 11.88, 124.90, -110.85),
        Joint(95.31, 23.72, 121.26, 15.24, 124.11, -109.03),
        Joint(95.27, 25.82, 122.19, 14.70, 121.18, -109.98),
        Joint(95.53, 24.31, 125.19, 14.78, 119.71, -110.27),
        Joint(97.76, 24.69, 125.01, 17.28, 119.20, -109.09),
        Joint(97.72, 26.97, 125.84, 16.76, 116.23, -110.14),
        Joint(99.16, 29.59, 126.36, 17.89, 113.05, -110.52),
        Joint(99.84, 33.11, 131.95, 17.67, 104.31, -113.22),
        Joint(99.82, 35.91, 132.29, 17.43, 101.31, -114.20),
        Joint(101.37, 36.07, 132.01, 19.02, 101.33, -113.85),
        Joint(102.57, 36.65, 130.18, 20.33, 102.42, -113.17),
        Joint(101.67, 37.99, 125.60, 19.68, 105.54, -112.25),
        Joint(104.41, 39.62, 124.99, 22.41, 104.32, -111.90),
        Joint(105.48, 42.39, 129.65, 22.95, 97.38, -114.61),
        Joint(109.73, 44.17, 128.18, 27.20, 96.85, -114.22),
        Joint(112.77, 43.45, 132.00, 30.11, 93.97, -115.43),
        Joint(119.15, 45.93, 134.01, 36.41, 90.09, -117.67),
        Joint(123.67, 48.59, 134.76, 40.98, 87.51, -119.91),
        Joint(123.71, 54.22, 133.77, 41.25, 84.02, -122.97),
        Joint(127.62, 54.31, 131.04, 45.01, 86.25, -121.49),
        Joint(130.75, 54.01, 134.68, 48.35, 84.24, -124.19),
        Joint(130.69, 47.01, 135.83, 47.99, 88.14, -119.82),
        Joint(136.49, 47.92, 130.40, 53.77, 91.03, -116.36),
        Joint(130.80, 33.33, 135.53, 48.62, 97.45, -109.38),
        Joint(138.17, 32.00, 139.91, 55.71, 94.62, -111.03),
        Joint(148.40, 28.11, 152.79, 65.67, 89.67, -118.53),
        Joint(145.53, 27.49, 155.27, 62.82, 88.78, -120.16)]

    robot_coords = []
    tcp_coords = []

    for joints in target_joints:
    	move_robot(joints)
    	robot_tcp = get_robot_tcp()  # 현재 로봇 TCP 좌표
	position_list.append(robot_tcp)
	

	# 클라이언트에게 마커 검출 요청 + 현재 로봇의 TCP 좌표 전송
	#conn.sendall(json.dumps({"command": "detect_marker", "robot_tcp": robot_tcp}))
	send_command_to_client(conn, "detect_marker")
	received_data = receive_data_from_client(conn)	
	if received_data:
            print received_data

        print "===================================================================="
        print "좌표 이동 완료."

    
    conn.close()
    s.close()

if __name__ == "__main__":
    rb = i611Robot()
    rb.open()
    IOinit(rb)
    start_server()
    print(position_list)
    rb.close()
