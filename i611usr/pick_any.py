# -*- coding: utf-8 -*-                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              
from i611_MCS import *                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        
from teachdata import *                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       
from i611_extend import *                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     
from rbsys import *                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           
from i611_common import *                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     
from i611_io import *                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         
from i611shm import *                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         
import time                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   
import socket
import json

HOST = '0.0.0.0'
PORT = 12346
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 
def send_command_to_client(conn, command_message):
    try:
	if isinstance(command_message, list):
	    command_message = json.dumps(command_message)
	    
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


def send_pose_command(conn):
    send_command_to_client(conn, "goal_pose")
    received_data = receive_data_from_client(conn)
    if received_data:
	goal_pose = [received_data['value_x'], received_data['value_y'],received_data['value_z'], received_data['value_rz'], received_data['value_rx']]
	print('goal_pose is:')
	print(goal_pose)
	
 	return goal_pose

def send_ompl_command(conn):
    send_command_to_client(conn, 'ompl')
    received_data = received_data_from_client(conn)
    if received_data:
	joint_lists = [received_data['joint_list']]
	
	return joint_lists

def check_gripper():
	a = din(48)
	b = din(49)
	c = din(50)
	d = din(51)
	result = [d,c,b,a]
	return result


def gripper(onoff):
    dout(48,'0000')
    if onoff == 'open':
        while check_gripper() !=  ['0','1','0','0']:
            dout(48,'0100')
			
    elif onoff == 'close':
        while check_gripper() !=  ['0','0','0','1']:
            dout(48,'0001')
    else:
        exit(0)
        
def get_curr_position():  # return current position
    print('position')
    pose = rb.getpos()
    position_values = pose.pos2list()
    # print(position_values)
    x = position_values[0]
    y = position_values[1]
    z = position_values[2]
    rz = position_values[3]
    ry = position_values[4]
    rx = position_values[5]
    
    current_pose = Position(x, y, z, rz, ry, rx)
    return current_pose

def offset_curr_position(x,y,z,rz,rx):
    print('offset curr position()')
    curr_pos = get_curr_position()
    offset_pos = curr_pos.offset(dx=x, dy=y, dz=z, drz=rz, drx=rx)	
    goal_joint_IK = rb.Position2Joint(offset_pos)
    
    if goal_joint_IK != 0:
        print('goal joint IK value is : ')
	print(goal_joint_IK.jnt2list())
	#rb.move(goal_joint_IK)
	return goal_joint_IK.jnt2list()
    else:
	print('this position is unreachable..') 

def main(conn):             
    try:                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          
        m = MotionParam(jnt_speed=50, lin_speed=50, pose_speed=50, overlap=0, acctime=0.4, dacctime=0.4)
        
        rb.motionparam(m)
        rb.override(50)
        rb.use_mt(True)
	        
	goal_pos = send_pose_command(conn)
	print('hello..')
	print('goal pose is : ', goal_pos)
	
	goal_jnt = offset_curr_position(goal_pos[0], goal_pos[1], goal_pos[2], goal_pos[3], 0)
	
	#ompl_result = send_ompl_command(conn)
	
    except Robot_emo as e:                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 
        print(e)
        rb.exit(0)       
        rbs.cmd_reset()                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             
                        
    except Robot_error as e:                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                
        print(e)
        rb.exit(0)       
        rbs.cmd_reset()  

    except Robot_fatalerror as e:                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           
        print(e)
        rb.exit(0)       
        rbs.cmd_reset()  

    except Exception as e:                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                
        print(e)                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      
        rb.exit(0)  

    except KeyboardInterrupt:                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             
        rb.exit(0)                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    
        print('Key Interrupt') 

    finally:                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              
        rb.close()    
        rbs.close()                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                
        rb.exit(0)  


def start_server():                                                                                                                             
    try:                                                                                                                            
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)  # Allow port reuse
        s.bind((HOST, PORT))
        s.listen(1)
        print "Server started. Waiting for client connection..."

        conn, addr = s.accept()
        print "Connected to client {}".format(addr)

        main(conn)
    except socket.error as e:
            print "Socket error: {}".format(e)
    finally:
            s.close()



if __name__ == '__main__':        
    try:
        rbs = RobSys()
        rbs.open()
        rb = i611Robot() #i611 로봇 생성자                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               
        _BASE = Base() #좌표계의 정의   

        rb.open() #로봇과의 연결 시작 초기화                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       
        IOinit(rb) #I/O 입출력 기능의 초기화   
        
        start_server()
    except Exception as e:                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                
        print(e)                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      
        rb.exit(0)   

    except Robot_emo:                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                
        rb.exit(0)       
        rbs.cmd_reset()                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             
                        
    except Robot_error:                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               
        rb.exit(0)       
        rbs.cmd_reset()  

    except Robot_fatalerror:                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           
        rb.exit(0)       
        rbs.cmd_reset()  

    finally:                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              
        rb.close()    
        rbs.close()                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                
        rb.exit(0)       
