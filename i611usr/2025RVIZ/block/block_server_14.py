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

def main():
    m1 = MotionParam(jnt_speed=100, lin_speed=1500, pose_speed=100, overlap=20, acctime=0.4, dacctime=0.4, posture=2)
    m2 = MotionParam(jnt_speed=100, lin_speed=1000, pose_speed=100, overlap=20, acctime=0.4, dacctime=1, posture=2)
    m3 = MotionParam(jnt_speed=100, lin_speed=500, pose_speed=100, overlap=0, acctime=0.4, dacctime=1, posture=2)
    rb.motionparam(m2)                                                                                              

    start_cmd = {"action":"start"}
    server.send_command(start_cmd)
    data = comm()

    yolo_detection_viewpoint = Joint(-22.09, -18.58, -49.15,-180.00, 112.28, -22.09)
    letter_waypoint = Joint(17.29, -34.24, -64.28,-180.00,  81.47,  17.29 )

    print "Start main loop"
    while True:

	# 1. Move robot to the block detection pose (over the stacked blocks)
	rb.move(yolo_detection_viewpoint)
	print('move robot to the block detection pose')
	curr_pose = rb.getpos()
	yolo_viewpoint_pose = curr_pose.pos2list()

	# 2. Request block yolo detection and grasping coordinates (in robot frame) from client
        server.send_command({"action":"detectblock", "robot_pose":yolo_viewpoint_pose[0:6]})
	print("server send command 'detect_block' to client.")
	data = comm()
	action = data.get("action")
	print(action)
	status = data.get("status")
	
	# 2-1. robot motion to sweep the area once (to clear/ move blocks)                            
        if action == "detect_block" and status == "fail":                                             
	    print("no detect...???? what!!!!")
            continue          

	# 2.2 get block grasping coordinates from client
        if action == "detect_block" and status == "success":
	    block_position = data.get("target")
	    block_d_target = data.get("d_target")
	    block_orientation = data.get("orientation")
            print ("[SERVER] Pick target from client:", block_position)
	    tdx,tdy,tdz,tdrz,tdry = block_position
	    ddx,ddy,ddz,ddrz = block_d_target

	    print '[SERVER] abs position(x,y,z,rz,ry) by yolo detector: '
	    print (tdx,tdy,tdz,tdrz,tdry)
	   
	    print '[SERVER] delta position(x,y,z,rz) by yolo detector: '	
	    print (ddx,ddy,ddz,ddrz)
	    
	    rb.motionparam(m3)
	   	 
            # abs x to relline ..
	    d_tddx = tdx + 100 - 95
	    rb.relline(dx = d_tddx, dy = ddy, dz = -250)
	    print '[1ST MOVING] move the robot to Tracking view point~'
		
	    curr_pose = rb.getpos()	
	    tracking_view_pose = curr_pose.pos2list()

	    server.send_command({"action":"get_tracking", "robot_pose":tracking_view_pose[0:6]})
	   
	    data = comm()
	    action = data.get("action")
	    tracking_status = data.get("status")
	    tracking_d_target = data.get("d_target")
	    t_ddx, t_ddy, t_ddz, t_ddrz = tracking_d_target
	    
            rb.motionparam(m2)                                                                                              
	   
	    x_offset = 34
	    y_offset = 2.5

	    padding = -7  # for test!!

	    rb.relline(dx = t_ddx, dy = t_ddy, dz = t_ddz+padding, drz = ddrz)
	    print('[TRACKING] move the robot to pick block~')
	    time.sleep(0.3)
		
	    # 3. Lift the block to the minumun safe z-height
	    rb.relline(dz=50)

	    print('Lift the block to the minimun safe z-height')

        # 4. check block class (None/ Lying/ Standing) 
        server.send_command({"action":"block_class"})
	block_class = comm()
	block_class_value = block_class.get("status")
	print "[BLOCK CLASS VALUE]", block_class_value
	if block_class_value == "lying":
	    print('block pick success~~ block state is lying!!')
	else: 
	    print('block pick fail or standing..')
	    rb.move(yolo_detection_viewpoint)
	    continue

	# 9. Request target block placement position from client
	server.send_command({"action":"placement"})
	placement_data = comm()
	placement_target = placement_data.get("target")

        rb.move(letter_waypoint)    

	letter_x, letter_y, letter_rz = placement_target[0], placement_target[1], placement_target[2]
	
	# 10. move robot to the target placement position
	letter_p = Position(letter_x, letter_y, 40, letter_rz, -0.0,-180.0)
	
	rb.line(letter_p)
        rb.relline(dz = -30)

	print "!!!!!!!!! place block !!!!!!!!!"
	server.send_command({"action":"suction", "state":"end"})
	time.sleep(0.5)

	print 'put the block on letter..'

	server.send_command({"action":"quota"})
	rb.relline(dz = 30)
	
if __name__ == '__main__':
    try:
        rb = i611Robot()
        _BASE = Base()
        rb.open()
	IOinit(rb)
	m = MotionParam(jnt_speed=100, lin_speed=150, pose_speed=100, overlap=20, acctime=0.4, dacctime=0.4)
	rb.motionparam(m)                                                                                              
        rb.override(80)                                                                                                
                                                                                                                               
        rb.settool(1,0.0,35.0, 330.0, 0.0, 0.0, 0.0)                                            
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

