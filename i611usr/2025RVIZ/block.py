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
import json
import socket
from robot_env_comm import RobotEnvServer

server = RobotEnvServer()
server.start()

server.send_command({"action":"start"})
data = server.receive_data()
print "Received Data from server", data


# define block place position 
with open('block_goals.json', 'r') as f:
	goals = json.load(f)
print("Loaded goals:", goals.keys())

used_ids = set()

''' select the goal which has not been used yet, 
belogs to the list for the given color, and mathces the given priority.
Once selected, its id is recored in the set of used goals so that it cannot be assigned again.'''

def assign_goal(color):
	if color in ["purple", "blue"]:
		candidates = []
		for g in goals.get("purple_blue", []):
			if g["color"] == color and g["id"] not in used_ids:
				candidates.append(g)
	else:
		candidates = [g for g in goals.get(color, []) if g["id"] not in used_ids]

	candidates.sort(key=lambda g: float(g["priority"]))
	
	if candidates:
		goal = candidates[0]
		used_ids.add(goal['id'])
		print '[SERVER {color} block is placed {id}'.format(color=color, id=goal["id"])
		return goal["pose"]
	
	print '[SERVER] No moere goals left for clolr:', color
	return None

def is_goal_full(color):
	if color in ["purple","blue"]:
		for goal in goals.get("purple_blue", []):
			if goal["color"] == color and goal['id'] not in used_ids:
				return False
		return True

def get_full_colors():
	full_colors = []
	for color in ["red", "yellow", "purple", "blue", "green", "pink"]:
		if is_goal_full(color):
			full_colors.append(color)
	return full_colors
			

# change gripper tcp offset
def ofs():
	print 'change the tool offset'
	

def gripper(onoff):
	dout(48,'0000')
	if onoff == 'open':
		while check_gripper() != ['0','1','0','0']:
			dout(48, '0100')
	elif onoff == 'close':
		while check_gripper() != ['0','0','0','1']:
			dout(48, '0001')
	else:
		exit(0)
		
def pick_block():
	print 'move robot to pick block'

def place_block():
	print 'move robot to place block'

def stopover():
	print 'stopover robot before goal place' 

current_block_color = None

def main():
	print 'start main'
	
	while True:
		fulls = get_full_colors()
		server.send_command({"action":"request_update", "full_colors":fulls})
	#	print "[SERVER] Send request_update:", fulls
		
		# --end condition--
		if set(fulls) == set(goals.keys()):
			print "[SERVER] All Goals are full. Shutting down."
			print "[SERVER] Used goal IDs:", sorted(list(used_ids))
			break
	
		data = server.receive_data()
	
		if not data:
			print('no data')
			continue

		action = data.get("action")

	        if action == "request_update":
			fulls = get_full_colors()
        		server.send_command({"action": "pick_block","full_colors":fulls})

		elif action == "pick_block":
			status = data.get("status")
			if status == "fail":
				print "[SERVER] No more blocks from client."
				break
			
			target = data.get('target',{})
			current_block_color = data.get('color')
			print "Pick target:", target
			server.send_command({"action":"stopover"})
	
		elif action == 'stopover':
			target = data.get('target', {})
			print "[SERVER] Stopover target from client:", target

			place_pose = assign_goal(current_block_color)
			if place_pose:
				server.send_command({"status": "success", "action": "place_block", "target" : place_pose})
				print '----------------------------place block -------------------------------------'
				# move to place_pose  	
			else:
				fulls = get_full_colors()
				server.send_command({"action":"request_update","full_colors":fulls})
			


if __name__ == '__main__':                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   
	try:
		rb=i611Robot()
		_BASE=Base()
		rb.open()
		main()
		server.close()
	
        except KeyboardInterrupt:                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    
                print('keyboardInterrupt')      
                rb.exit(0)                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     
                rb.close()                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           
        except Robot_poweroff:                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       
                print('Robot power off')                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             
                rb.exit(0)                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           
                rb.close(0)                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          
        except Robot_stop:                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           
                print('Robot stop')                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  
                rb.exit(0)                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           
                rb.close(0)                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          
	except Exception, e:
                print('error: ', e.__class__.__name__, ':', e)
                rb.exit(0)                                    
                rb.close(0)                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          
