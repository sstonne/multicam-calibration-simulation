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

gripper_state = None

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
    global gripper_state
    m1 = MotionParam(jnt_speed=100, lin_speed=1500, pose_speed=100, overlap=20, acctime=0.4, dacctime=0.4)
    rb.motionparam(m1)                                                                                              

    start_cmd = {"action":"start"}
    server.send_command(start_cmd)
    data = comm()
    gripresp = send_gripper_and_wait(server,"reset")
    print "[SERVER] reset resp :", gripresp
    print 'reset success'

    gripresp = send_gripper_and_wait(server,"activate")
    print "[SERVER] activate resp :", gripresp

    #yolo_detection_viewpoint = Joint(-12.77, -18.43, -53.36,-180.00, 108.21, -12.77)
    yolo_detection_viewpoint = Joint(-11.54, -20.10, -45.39,-182.16, 114.17, -10.43)
    waypoint_viewpoint = Joint(-42.92, -29.59, -76.13,-180.00,  74.29, -42.92)
    z_lifted_waypoint = Joint(-41.01, -28.66,-101.56,-180.00,  49.79, -41.01 )

    lying_waypoint = Joint(-41.01, -31.08,-103.26,-180.00,  45.67, -41.01)
    standing_waypoint = Joint(-37.31, -22.98,-138.08, -75.36,  17.49,-139.09)

    gripper_mode = "full"
    outer_continue = False
	   	
    print "Start main loop"
    while True:

	# 1. Move robot to the block detection pose (over the stacked blocks)
	rb.move(yolo_detection_viewpoint)
	send_gripper_and_wait(server, "open","full")
	print('move robot to the block detection pose')

	# 2. Request block yolo detection and grasping coordinates (in robot frame) from client
        server.send_command({"action":"detectblock"})
	print("server send command 'detect_block' to client.")
	data = comm()
	action = data.get("action")
	print(action)
	status = data.get("status")
	gripper_mode = data.get("orientation")	
	
	
	# 2-1. robot motion to sweep the area once (to clear/ move blocks)
        if action == "detect_block" and status == "fail":
	    print('robot motion to sweep the area once')
	    sweep1 = Joint(-14.96, -20.47,-120.72,-198.01,  51.02,  49.93)
	    sweep2 = Joint(7.76, -50.34, -63.15,-188.81,  81.89,  62.19)
 	    rb.move(sweep1, sweep2)
	    continue
	
	# 2.2 get block grasping coordinates from client
        if action == "detect_block" and status == "success":
	    send_gripper_and_wait(server, "open",gripper_mode)
	    #send_gripper_and_wait(server, "open","full")
	    block_position = data.get("target")
	    block_d_target = data.get("d_target")
            print ("[SERVER] Pick target from client:", block_position)
	    tdx,tdy,tdz,tdrz,tdry = block_position
	    ddx,ddy,ddz,ddrz = block_d_target

	    print '[SERVER] abs position(x,y,z,rz,ry) by yolo detector: '
	    print (tdx,tdy,tdz,tdrz,tdry)
	   
	    print '[SERVER] delta position(x,y,z,rz) by yolo detector: '	
	    print (ddx,ddy,ddz,ddrz)
	    
	    #rb.reljntmove(dj6 = -ddrz)
	    rb.relline(dz = -280)
	    rb.relline(dx = ddx, dy = ddy, drz = ddrz)
	    print 'ddz + 437 is :', ddz+437
	    m2 = MotionParam(jnt_speed=100, lin_speed=500, pose_speed=100, overlap=20, acctime=0.4, dacctime=1)
            rb.motionparam(m2)                                                                                              
	    if ddz+487 < -182:
	 	rb.relline(dz = -182)
	    else:
	    	rb.relline(dz = ddz+587)
	    # add ddry moving..

	    rb.motionparam(m1)

	    print('move the robot to pick yolo block~')
	    send_gripper_and_wait(server, "close")
		
	    # 3. Lift the block to the minumun safe z-height
	    rb.relline(dz=100)
	    print('Lift the block to the minimun safe z-height')

	# 5. Place the block in the correct orientation
	picked_block_pose = data.get("height")
	picked_block_pose2 = gripper_state
	print "picked block pose, picked block pose2 = ", picked_block_pose, picked_block_pose2
	
	
	print(picked_block_pose)
	if picked_block_pose == "lying":
	    # robot motion control code to place block
	    rb.move(z_lifted_waypoint)
	    rb.move(lying_waypoint)
	    send_gripper_and_wait(server,"open",gripper_mode, speed=255)
	    rb.sleep(0.3)
            print "Block placed in waypoint place.."

	elif picked_block_pose =="standing":
	    # robot motion control code to place block in correct is required..
	    rb.move(z_lifted_waypoint)
	    rb.move(standing_waypoint)
	    send_gripper_and_wait(server,"open",gripper_mode, speed=100)
	    rb.sleep(0.3)
            print "Block placed in waypoint place.."
	    
	else:
	    print 'error in yolo height'
	
	# 6. lift the robot to detect center of block
	rb.move(waypoint_viewpoint)
        send_gripper_and_wait(server,"open","full")
	rb.join()

	# 7. Request block center and rz(d6) in waypoint
	while True:
            server.send_command({"action":"waypoint"})
	    data = comm()
 	
	    if not data.get("status") == "success":
		outer_continue = True
		break

	    try:
		print "data.get(status) is success in waypoint"
	        waypoint_target = data.get("target") # relative coordinate
	        w_x, w_y, w_d6 = waypoint_target
	        rb.relline(dx = w_x, dy = w_y, dz = -177)
	        rb.reljntmove(dj6 = -w_d6)
	        send_gripper_and_wait(server,"close")
		print "gripper close in waypoint"
	        rb.relline(dz=50)
	        print "rb.relline(dz=50)"
		break
	    except Exception, e:
		print "[SERVER] waypoint value is unreach.."
		continue
	
	if outer_continue:
	    outer_continue = False
	    continue

	# 9. Request target block placement position from client
	server.send_command({"action":"placement"})
	placement_data = comm()
	placement_target = placement_data.get("target")

	letter_x, letter_y, letter_rz = placement_target[0], placement_target[1], placement_target[2]
	
	# 10. move robot to the target placement position
	letter_p = Position(letter_x, letter_y, 50, letter_rz, -0.0,-180.0)
	rb.line(letter_p)
        rb.relline(dz = -52)
	send_gripper_and_wait(server,'open',gripper_mode, speed=60)
	time.sleep(0.3)
	print 'put the block on letter..'
	server.send_command({"action":"quota"})
	rb.relline(dz = 20)
	
if __name__ == '__main__':
    try:
        rb = i611Robot()
        _BASE = Base()
        rb.open()
	IOinit(rb)
	m = MotionParam(jnt_speed=100, lin_speed=1500, pose_speed=100, overlap=20, acctime=0.4, dacctime=0.4)
	rb.motionparam(m)                                                                                              
        rb.override(80)                                                                                                
                                                                                                                               
        rb.settool(1, 0.0, 0.0, 245.0, 0.0, 0.0, 0.0)                                                                 
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

