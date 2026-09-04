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
    m1 = MotionParam(jnt_speed=100, lin_speed=1500, pose_speed=100, overlap=20, acctime=0.4, dacctime=0.4, posture=2)
    m2 = MotionParam(jnt_speed=100, lin_speed=1000, pose_speed=100, overlap=20, acctime=0.4, dacctime=1, posture=2)
    rb.motionparam(m1)                                                                                              

    start_cmd = {"action":"start"}
    server.send_command(start_cmd)
    data = comm()
    gripresp = send_gripper_and_wait(server,"reset")
    print "[SERVER] reset resp :", gripresp
    print 'reset success'

    gripresp = send_gripper_and_wait(server,"activate")
    print "[SERVER] activate resp :", gripresp

    yolo_detection_viewpoint = Joint(-15.01, -16.47, -71.13,-180.00,  92.41, -11.51)
    waypoint_viewpoint = Joint(-45.58, -38.19, -77.33,-180.00,  64.47, -45.58)
    z_lifted_waypoint = Joint(-38.78, -28.64,-101.65,-180.00,  49.70, -38.78)

    lying_waypoint = Joint(-38.78, -32.08,-103.95,-180.00,  43.95, -38.78)
    standing_waypoint = Joint(-30.72, -25.06,-146.69, -31.47,  42.86,-176.40 )

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
	    block_orientation = data.get("orientation")
            print ("[SERVER] Pick target from client:", block_position)
	    tdx,tdy,tdz,tdrz,tdry = block_position
	    ddx,ddy,ddz,ddrz = block_d_target

	    print '[SERVER] abs position(x,y,z,rz,ry) by yolo detector: '
	    print (tdx,tdy,tdz,tdrz,tdry)
	   
	    print '[SERVER] delta position(x,y,z,rz) by yolo detector: '	
	    print (ddx,ddy,ddz,ddrz)
	    
	    #rb.reljntmove(dj6 = -ddrz)
	    rb.relline(dz = -200)
	    rb.relline(dx = ddx+5, dy = ddy, drz = ddrz)
	    print 'ddz + 415 is :', ddz+415
	    print 'ddz + 280 + 135 is :', ddz+280+135

            rb.motionparam(m2)                                                                                              
	    # yolo viewpoint - 335 = ground.. -> 335 - 200 = 135
	   
	    if ddz+335 < -135:
	 	rb.relline(dz = -135)
	    
	    else:
		if block_orientation == 'short':
		    print "block orientation is short"
	            rb.relline(dz = ddz+200+245)
		else:
		    print "block orientation is long"
		    rb.relline(dz = ddz + 200 + 235)
	    # add ddry moving..

	    rb.motionparam(m1)

	    print('move the robot to pick yolo block~')
	    send_gripper_and_wait(server, "close")
		
	    # 3. Lift the block to the minumun safe z-height
	    rb.relline(dz=100)
	    print('Lift the block to the minimun safe z-height')

            # gripper state check                                                                                                              
	    t1 = time.time()
            verify_resp = send_gripper_and_wait(server, "check_gripper")                                                                
            print('[SERVER] !!!!!!!!second check!!!!!!!!!!:', verify_resp)                                                      
            verify_state = verify_resp.get("gripper_state")                                                                     
                                                                                                                                
            if verify_state in ("fully close", "two_blocks"):                                                                   
                if verify_state == "two_blocks":                                                                                
                    print("[SERVER] Multiple blocks detected. Opening gripper and retrying...")                                 
                    send_gripper_and_wait(server, "open", gripper_mode, speed=200)                                              
                else:                                                                                                           
                    print("[SERVER] Gripper closed, but no block was grasped. Retrying...")                                     
                print("[SERVER] Sending stop_yolo command to client to reset camera...")                                        
                server.send_command({"action": "stop_yolo"})                                                                    
                data = comm()                                                                                                   
                print("[SERVER] stop_yolo response:", data)                                                                     
                rb.relline(dz=150)                                                                                              
                continue        

	    t2 = time.time()
	    print('time for gripper state check :', t2 - t1)

	# 5. Place the block in the correct orientation
	picked_block_pose = data.get("height")
	picked_block_pose2 = gripper_state
	print "picked block pose, picked block pose2 = ", picked_block_pose, picked_block_pose2
	
	print(picked_block_pose)
	if picked_block_pose2 == "lying":
	    # robot motion control code to place block
	    rb.move(z_lifted_waypoint)
	    rb.move(lying_waypoint)
	    send_gripper_and_wait(server,"open",gripper_mode, speed=255)
	    rb.sleep(0.1)
            print "Block placed in waypoint place.."

	elif picked_block_pose2 =="standing":
	    # robot motion control code to place block in correct is required..
	    rb.move(z_lifted_waypoint)
	    rb.move(standing_waypoint)
	    send_gripper_and_wait(server,"open",gripper_mode, speed=255)
	    rb.sleep(0.1)
            print "Block placed in waypoint place.."
	    
	else:
	    print 'error in yolo height'
	
	send_gripper_and_wait(server,"nowait_open",'full',speed =255)	
	# 6. lift the robot to detect center of block
	rb.move(waypoint_viewpoint)
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
	        rb.relline(dx = w_x + 5, dy = w_y, dz = -80)
	        rb.reljntmove(dj6 = -w_d6)
	        send_gripper_and_wait(server,"close")
		print "gripper close in waypoint"
		letter_waypoint = Joint(26.60, -31.68, -72.58,-179.99,  75.74,  26.60)
		rb.move(letter_waypoint)
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
	letter_j = rb.Position2Joint(letter_p)	
	
	#rb.line(letter_p)
	rb.move(letter_j)
        rb.relline(dz = -50)
	send_gripper_and_wait(server,'open',gripper_mode, speed=60)
	time.sleep(0.1)
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

