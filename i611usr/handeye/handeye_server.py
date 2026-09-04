#!/usr/bin/python/setting.py
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
PORT = 12348
position_list = []                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 

def send_command_to_client(conn, command_message):
    if conn is None:
        print("[ERROR] Connection is None — cannot send command")
        return

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


def send_joint_command(conn):
    send_command_to_client(conn, "capture")
    received_data = receive_data_from_client(conn)
    if received_data:
        goal_joint = [received_data['d1'], received_data['d2'],received_data['d3'], received_data['d4'], received_data['d5'],received_data['d6']]
        print('goal_joint :')
        print(goal_joint)
        
 	return goal_joint
	
def send_quit_command(conn):
    send_command_to_client(conn, "quit")
    return 0

def get_curr_position():  # return current position
    global position_list
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
   
    position_list.append([x,y,z,rz,ry,rx])    
    
    return position_list

def main(conn):             
    global position_list
    try:                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          
        m = MotionParam(jnt_speed=70, lin_speed=50, pose_speed=50, overlap=0, acctime=1.0, dacctime=1.0)
        
        rb.motionparam(m)
        rb.override(50)
        rb.settool(1,0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        #rb.changetool(1)  

        rb.settool(2,0.0, 35.0, 330.0, 0.0, 0.0, 0.0)
        #rb.changetool(2)  

        rb.settool(3,0.0, 0.0, 150.0, 0.0, 0.0, 0.0)
        rb.changetool(3)  
        rb.use_mt(True)
	
        while True:	        
            goal = send_joint_command(conn)
            print('hello..')
            print('goal joint is : ', goal)

            if goal == None:
                print('EOF')
                send_quit_command(conn)
                print(position_list)

                break

            goal = Joint(goal[0], goal[1], goal[2], goal[3], goal[4], goal[5])
	    print('get goal joint')
            rb.move(goal)	
	    print('move goal joint')
            get_curr_position()	
    
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
	print("Client conneted: ", addr)
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

