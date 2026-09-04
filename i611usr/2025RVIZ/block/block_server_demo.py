#!/usr/bin/python
# -*- coding: utf-8 -*-

# block_server_demo.py
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

# log data 
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

def check_gripper():
    a,b,c,d = din(48), din(49), din(50), din(51)
    return [d,c,b,a]

def gripper(onoff):
    dout(48,'0000')
    if onoff == 'open':
	while check_gripper() != ['0','1','0','0']:
	    dout(48,'0100')
    
    elif onoff == 'close':
	while check_gripper() != ['0', '0','0','1']:
	    dout(48,'0001')
    else:
	exit(0)
	
# server start
server = RobotEnvServer()
server.start()
server.send_command({"action":"start"})
print("[SERVER] Waiting for client ready...")

def comm():
    data = server.receive_data()
    if not data:
	print "No data from client."
	return data

    return data

def main():
    yolo_detection_viewpoint = Joint(69.75, -45.61, -59.81,-180.00,  74.58,  69.75)
    print "Start main loop"
    while True:

	# 1. Move robot to the block detection pose (over the stacked blocks)
	rb.move(yolo_detection_viewpoint)
	gripper("open")

	# 2. Request block yolo detection and grasping coordinates (in robot frame) from client
	server.send_command({"action":"detect_block"})
	data = comm()
	action = data.get("action")
	status = data.get("status")

	# 2-1. robot motion to sweep the area once (to clear/ move blocks)
        if action == "detect_block" and status == "fail":
 	    # rb.move(sweep)
	    continue
	
	# 2.2 get block grasping coordinates from client
        if action == "detect_block" and status == "success":
            target = data.get("target", {})
            color = data.get("color")
            print "[SERVER] Pick target from client:", target, "color:", color
	    grasping_coord = Position(
		target.get("x"),
		target.get("y"),
		target.get("z"),
		target.get("rz"),
		target.get("ry"),
		target.get("rx")
		)

 	    rb.move(grasping_coord)
	    gripper("close")
	
	# 3. Lift the block to the minumun safe z-height
	rb.relline(dz=50)

	# 4. Move the block to waypoint 
	z_lifted_waypoint = Joint(74.37, -58.72, -31.49,-179.98,  89.82,  74.50) # need to change..
	rb.move(z_lifted_waypoint)

	# 5. Place the block in the correct orientation
	picked_block_pose = data.get("orientation")
	if picked_block_pose == "vertical":
	    # robot motion control code to place block in correct is required..
	    print "the block is picked in vertical.." 
	    # robot motion
	    gripper("open")
	else:
	    rb.relline(dz=-50)
	    gripper("open")
	
	# 6. lift the robot to detect center of block
	rb.relline(dz=200)

	# 7. Request block center and rz(d6) in waypoint
	curr_position = pose.pos2list()[0:6]
        server.send_command({"action":"waypoint"})
	data = comm()
	
        # 8. Precise grasping of the block at the waypoint (Assume center-point detection always succeeds and proceed)
	target = data.get("target", {}) # relative coordinate
	rb.relline(dx = target.get("x"), dy = target.get("y"), dz = -200)
	rb.reljntmove(dj6 = target.get("d6"))
	gripper("close")
	rb.relline(dz=50)

	# 9. Request target block placement position from client
	server.send_command({"action":"placement"})
	data = comm()
	
	# 10. move robot to the target placement position
	rb.relline(dx = target.get("x"), dy = target.get("y"), drz = target.get("rz"))
	rb.relline(dz = -50)
	gripper("open")
	rb.relline(dz = 200)
	

if __name__ == '__main__':
    try:
        rb = i611Robot()
        _BASE = Base()
        rb.open()
	m = MotionParam(jnt_speed=30, lin_speed=100, pose_speed=100, overlap=0, acctime=0.1, dacctime=0.1)
	rb.motionparam(m)                                                                                              
        rb.override(80)                                                                                                
                                                                                                                               
       #rb.settool(1, 0.0, 0.0, 100.0, 0.0, 0.0, 0.0)                                                                 
       #rb.changetool(1)                     
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

