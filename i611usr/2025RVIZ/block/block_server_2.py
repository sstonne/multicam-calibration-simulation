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

#def comm():
#    data = server.receive_data()
#    if not data:
#	print "No data from client."
#	return data
def comm():
    raw = server.receive_data()
    if not raw:
        print("No data from client.")
        return None
    if isinstance(raw, basestring):  # Python2 기준
        try:
            data = json.loads(raw)
        except Exception as e:
            print("JSON decode error:", e, raw)
            return None
    else:
        data = raw
    return data

def gripper(state, mode="full"):
    # state example = ["reset", "activate", "open", "close", "connect_close"]
    server.send_command({"action":"gripper", "state":state, "mode":mode})
    while True:
	print('1')
        gripper_data = comm()
	print(gripper_data)
	print('2')
        gripper_status = gripper_data.get("status")
	print '3'
	if gripper_status == "end":
	    print '4'
	    return 'gripper_end'
	   
	print('no end..')
	rb.sleep(0.05)

def main():
    start_cmd = {"action":"start"}
    server.send_command(start_cmd)
    data = comm()
    gripper("reset")
    print 'reset success'

    gripper("activate")
    print 'activate success'
    yolo_detection_viewpoint = Joint(-12.77, -18.43, -53.36,-180.00, 108.21, -12.77)
    z_lifted_waypoint = Joint(112.58, -41.02, -75.27,-179.99,  63.71, 112.58)
    waypoint_viewpoint = Joint(112.65, -29.77, -75.83,-180.00,  74.39, 112.65)

    gripper_mode = "full"
    print "Start main loop"
    while True:

	# 1. Move robot to the block detection pose (over the stacked blocks)
	rb.move(yolo_detection_viewpoint)
	gripper("open","short")
	print('move robot to the block detection pose')

	# 2. Request block yolo detection and grasping coordinates (in robot frame) from client
        server.send_command({"action":"detectblock"})
	print("server send command 'detect_block' to client.")
	data = comm()
	action = data.get("action")
	status = data.get("status")
	gripper_mode = data.get("orientation")	
	
	# 2-1. robot motion to sweep the area once (to clear/ move blocks)
        if action == "detect_block" and status == "fail":
	    print('robot motion to sweep the area once')
	    sweep1 = Joint(-25.69,  -1.53,-125.33,-180.00,  53.13, -25.69)
	    sweep2 = Joint(6.05, -25.30, -92.97,-180.00,  61.74,   6.05)
 	    rb.move(sweep1, sweep2)
	    continue
	
	# 2.2 get block grasping coordinates from client
        if action == "detect_block" and status == "success":
            gripper("open", gripper_mode)
	    block_position = data.get("target")
            print ("[SERVER] Pick target from client:", block_position)
	    tdx,tdy,tdz,tdrz,tdry = block_position
	    print (tdx,tdy,tdz,tdrz,tdry)
	    block_pose_byYolo = Position(tdx,tdy,tdz,tdrz,tdry,-180)
	    rb.line(block_pose_byYolo)
	    print('move the robot to pick yolo block~')
	    gripper("close")
	
	    # 3. Lift the block to the minumun safe z-height
	    rb.relline(dz=50)
	    print('Lift the block to the minimun safe z-height')

	# 4. Move the block to waypoint 
	rb.move(z_lifted_waypoint)

	# 5. Place the block in the correct orientation
	picked_block_pose = data.get("height")
	if picked_block_pose == "lying":
	    # robot motion control code to place block in correct is required..
	    print "the block is picked in vertical.." 
	    # robot motion code 
	    gripper("open", gripper_mode)
	    print "Block placed in waypoint place.."
	else:
	    rb.relline(dz=-130)
	    gripper("open", gripper_mode) 
            print "Block placed in waypoint place.."
	
	# 6. lift the robot to detect center of block
	rb.move(waypoint_viewpoint)
	gripper("open")

	# 7. Request block center and rz(d6) in waypoint
        server.send_command({"action":"waypoint"})
	data = comm()
	
        # 8. Precise grasping of the block at the waypoint (Assume center-point detection always succeeds and proceed)
	waypoint_target = data.get("target") # relative coordinate
	w_x, w_y, w_d6 = waypoint_target
	rb.relline(dx = w_x, dy = w_y, dz = -160)
	rb.reljntmove(dj6 = -w_d6)
	gripper("close")
	rb.relline(dz=50)

	# 9. Request target block placement position from client
	server.send_command({"action":"placement"})
	data = comm()
	
	# 10. move robot to the target placement position
	rb.relline(dx = target.get("x"), dy = target.get("y"), drz = -target.get("rz"))
        #rb.relline(dz = -17)
	gripper("open")
	rb.relline(dz = 200)
	
if __name__ == '__main__':
    try:
        rb = i611Robot()
        _BASE = Base()
        rb.open()
	IOinit(rb)
	m = MotionParam(jnt_speed=15, lin_speed=20, pose_speed=100, overlap=30, acctime=0.1, dacctime=0.1)
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

