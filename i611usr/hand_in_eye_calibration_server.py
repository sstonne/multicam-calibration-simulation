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
    position_values = pose.pos2list()  # pos2list() 사용하여 리스트 변환
    return position_values[:3]  # x, y, z 값만 반환

def move_robot(target_joints):
    """ 로봇을 지정된 joint 위치로 이동 """
    rb.move(target_joints)
    time.sleep(0.2)  # 속도 최적화

def get_robot_tcp():
    """ 현재 로봇 TCP 위치 반환 """
    curr_pos = get_position()  # 리스트 반환
    return curr_pos  # [x, y, z] 형태로 반환

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

    # 속도 조절 추가
    m = MotionParam(jnt_speed=10, lin_speed=100, pose_speed=20, overlap=20, acctime=0.2, dacctime=0.2)
    rb.motionparam(m)
    rb.override(100)  # �기존 90 → 100으로 변경하여 속도 최적화
    rb.use_mt(True)

    # 실제 로봇 이동 좌표 리스트
    target_joints = [
        Joint(360.15013923267327, 18.814511138613859, 203.14812809405939, -91.167698019801975, -3.4490253712871288, 0.83980507425742568)
	Joint(363.57897586633663, 34.308787128712872, 196.02892945544554, -91.167698019801975, -3.4483292079207919, 0.83980507425742568)
	Joint(346.61486695544551, 25.167001856435643, 196.02521658415841, -91.167465965346523, 8.7296565594059405, 0.83910891089108908)
	Joint(396.48406559405942, 25.361231435643564, 196.02243193069307, -91.167233910891085, -28.101794554455445, 0.83957301980198018)
	Joint(396.48128094059405, 25.363087871287128, 193.09784962871285, -91.167465965346523, -28.1015625, 27.571086014851485)
	Joint(329.40176361386136, 12.408183787128712, 198.39495668316832, -84.336014851485146, 34.57077660891089, -10.447323638613861)
	Joint(307.88637066831683, 12.452042079207921, 189.62608292079207, -84.336014851485146, 43.184870049504951, -10.446859529702969)
    ]
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
