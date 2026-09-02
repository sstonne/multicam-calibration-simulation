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
    m1 = MotionParam(jnt_speed=100, lin_speed=1000, pose_speed=100, overlap=20, acctime=0.4, dacctime=0.4, posture=2)
    m2 = MotionParam(jnt_speed=100, lin_speed=1000, pose_speed=100, overlap=20, acctime=0.4, dacctime=1, posture=2)
    m3 = MotionParam(jnt_speed=100, lin_speed=500, pose_speed=100, overlap=0, acctime=0.4, dacctime=1, posture=2)
    rb.motionparam(m1)                                                                                              

    start_cmd = {"action":"start"}
    server.send_command(start_cmd)
    data = comm()

    yolo_detection_viewpoint = Joint(-22.09, -18.58, -49.15,-180.00, 112.28, -22.09)
    yolo_viewpoint_pose = []
    letter_waypoint = Joint(17.29, -34.24, -64.28,-180.00,  81.47,  17.29 )
    letter_waypoint = Joint(21.55, -27.15, -80.32,-179.99,  72.55,  21.55)

    current_color = None
    print "Start main loop"
    while True:

	# 1. Move robot to the block detection pose (over the stacked blocks)
        server.send_command({"action":"detectblock"})
	
	data = comm()
	action = data.get("action")
	status = data.get("status")
	print "detectblock result -> action:", action, " status:", status
	
	# 2. Request block yolo detection and grasping coordinates (in robot frame) from client
        server.send_command({"action":"initial_block"})
	print("server send command 'initial_block' to client.")

	data = comm()
	if not data:
	    print("[SERVER] no response after retry initial_block.")
	    continue
	   
	action = data.get("action")
	status = data.get("status")
	print "action : ",action,"   status : ", status
	
	if action == "viewpoint":
	    print('move to yolo entire view point...')
	    rb.move(yolo_detection_viewpoint)
 
	    curr_pose = rb.getpos()
	    yolo_viewpoint_pose = curr_pose.pos2list()

	    server.send_command({"action":"yolo_viewpoint", "robot_pose":yolo_viewpoint_pose[0:6]})
	
	    data = comm()

            if not data:
		print("[SERVER] no response after yolo viewpoint")
		continue

	    action = data.get("action")
	    status = data.get("status")
	    print("[SERVER][RETRY] after yolo_viewpoint -> ", action, status)

	# 2-1. robot motion to sweep the area once (to clear/ move blocks)                            
        if action == "initial_block" and status == "fail":                                             
	    print("[NO BLOCK] no detect...???? what!!!!")
            #server.send_command({"action":"yolo_stop"})
            continue          

	# 2.2 get block grasping coordinates from client
        if action == "initial_block" and status == "success":

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
	    
	    rb.motionparam(m1)
	   
	    exact_detect_viewpoint = Position(tdx+70, tdy, yolo_viewpoint_pose[2]-250, yolo_viewpoint_pose[3], yolo_viewpoint_pose[4], yolo_viewpoint_pose[5])
	    rb.line(exact_detect_viewpoint)
	   	 
	    print '[1ST MOVING] move the robot to Tracking view point~'
		
	    curr_pose = rb.getpos()	
	    exact_view_pose = curr_pose.pos2list()

	    server.send_command({"action":"exact_block", "robot_pose":exact_view_pose[0:6]})
	   
	    data = comm()
	    action = data.get("action")
	    tracking_status = data.get("status")

	    if action == "exact_block" and status == "success":
		tracking_d_target = data.get("d_target")
	        t_ddx, t_ddy, t_ddz, t_ddrz = tracking_d_target
	  	current_block_color = data.get("color")
			
		base_xyz = data.get("target")
		b_dx, b_dy, b_dz, b_drz, b_dry = base_xyz
	   
                rb.motionparam(m1)                                                                                              

	        x_offset = 34
	        y_offset = 2.5

	        z_offset = -7  # for test!!

		if b_dz < -16.5:
 		    print "[EXACT MOVE] depth value is below the lower limit.."
		    limit_z = -110 
		    rb.relline(dx = t_ddx, dy = t_ddy, dz = limit_z+20, drz = t_ddrz)
		    
	        else:
		    print "[EXACT MOVE] depth value is within the normal range."
		    rb.relline(dx = t_ddx, dy = t_ddy, dz = t_ddz+z_offset+20, drz = t_ddrz)
		
		rb.relline(dz = -20)

	        print('[TRACKING] move the robot to pick block~')
	        time.sleep(0.1) # for stable suction
		
	        # 3. Lift the block to the minumun safe z-height
	        rb.relline(dz=100)
		
		#server.send_command({"action":"yolo_stop"})

        	print('Lift the block to the minimun safe z-height')

	    if action == "exact_block" and status == "fail":
		print("[NO BLOCK] no block... in exact_block")
		break

            # 4. check block class (None/ Lying/ Standing) 
            server.send_command({"action":"block_class"})
	    block_class = comm()
	    block_class_value = block_class.get("status")
	    print "[BLOCK CLASS VALUE]", block_class_value

            if block_class_value == "lying":
	        print('block pick success~~ block state is lying!!')
		
	        placement_target = block_class.get("placement_target")
	        print "placement_target is..", placement_target

                rb.move(letter_waypoint)    

	        letter_x, letter_y, letter_rz = placement_target[0], placement_target[1], placement_target[2]
	        print "[PLCEMENT] letter_x , y, z is : ",letter_x, letter_y, letter_rz
	
	        # 10. move robot to the target placement position
	        letter_p = Position(letter_x, letter_y, 40, letter_rz, -0.0,-180.0)
	
	        rb.line(letter_p)
                rb.relline(dz = -30)

	        print "!!!!!!!! place block !!!!!!!!!"
	        server.send_command({"action":"suction", "state":"end"})
	        time.sleep(0.3)

	        print 'put the block on letter..'

	        rb.relline(dz = 30)
	        if current_block_color == "purple":
	            rb.relline(dx = -100, dy = 350)
	
	        if current_block_color == "yellow":
	            rb.relline(dx = -50, dy = 250)
	        if current_block_color == "blue":
	            rb.relline(dx = -50, dy =150)
		
	    elif block_class_value in ["standing", "fail"]:
	        print("[SERVER] block pick failed or standing → retrying detection...")
	        time.sleep(0.5)
		

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

