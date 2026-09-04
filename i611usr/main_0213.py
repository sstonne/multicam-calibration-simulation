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
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 
eject_count = 0
order_num = 0
yolo_count = 0 
none_count = 0
NONE_THRESHOLD=2

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

def send_pipette_command(conn):
    send_command_to_client(conn, "pipette")
    received_data = receive_data_from_client(conn)
    if received_data:
        print received_data
        x = received_data['value_x']
        y = received_data['value_y']
        return x, y

def send_app_command(conn):
    global order_num
    send_command_to_client(conn,"app")
    received_data = receive_data_from_client(conn)
    if received_data:
        namelist = received_data['name']
        iterlist = received_data['iter']
        bottle_count = received_data['bottle_count']
        print(namelist,iterlist,bottle_count)
        return namelist,iterlist,bottle_count
    


def send_bicker_2d_command(conn):
    send_command_to_client(conn, "bicker_2d")
    received_data = receive_data_from_client(conn)
    if received_data:
        print "Received data from client: {}".format(received_data)
        x = received_data['value_x']
        y = received_data['value_y']
        print(x,y)
        return x,y
    else:
        x = 0
        y = 0
        return x,y
first_none_position = None
def send_yolo_command(conn):
    global yolo_count, none_count,tip_count,first_none_position
    send_command_to_client(conn, "yolo_tip")
    received_data = receive_data_from_client(conn)
    none_position_count = None
    if received_data:
        print "Received data from client: {}".format(received_data)
        value = received_data['value']
        print(value)
 	yolo_count +=1

	if value !=1:
      	    none_count += 1
	    if none_count == 1:
		print('get first none position')
		first_none_position = position()

        hole_number = yolo_count - none_count
	print 'hole_number :',hole_number, 'yolo_count',yolo_count, 'none_count', none_count

	if value == 1 and none_count >= NONE_THRESHOLD:
            print "Tip detected after {} none case. Starting filling process.".format(none_count)
		
	    # none count�큼 filling rack 실행
	    for none in range(none_count):
		send_command_to_client(conn,'fill_rack')
		rack_data = receive_data_from_client(conn)
	        if rack_data:
		    print "Received rack data: {}".format(rack_data)
		    x_value = rack_data['x']
		    y_value = rack_data['y']
		    rz_value = rack_data['rz']
	            
		    print('none', none)
		    if none == 0:
			print('none is 0')
		    	curr_pos = position()
	       	    	up = curr_pos.offset(dz = 200)
		    	rotate = rb.Position2Joint(curr_pos)
		    	rt1 = rotate.offset(dj4 =-40)	
              	    	rt2 = rt1.offset(dj5 = 50)
		    	rb.line(up)
		    	rb.move(rt1)
		    	rb.move(rt2)
		
	  	    # ready_to_pick_tips = Joint(67.51,  23.16, 101.95,-180.00, -54.89,-112.49)       
		    center_position_before_gripper_close = Position(304.14,-168.33, 192.30,  90.00,   0.00,-180.00)
		    center_position_before_gripper_close_J = Joint(77.76,  20.96, 133.88,-180.00, -25.16,-102.24)
	   	    center_position_above_300 = Position(304.14,-168.33, 492.30,  90.00,   0.00,-180.00)
		    center_joint_above_300 = Joint(77.76,  -3.69, 104.07,-180.00, -79.62,-102.24)
			
		    rb.move(center_joint_above_300)
	
	            gripper('open')
	            curr_pos = position()
		    move_pick = curr_pos.offset(dx =x_value, dy =-y_value, drz = rz_value)
		    rb.line(move_pick)
		    down = move_pick.offset(dz =- 300)	
		    rb.line(down)
		    gripper('close')
		    time.sleep(2)
		    rb.line(move_pick)
		    rb.move(center_joint_above_300)
		    print('MOVE TO CENTER ABOVE 300')
		    curr_pose = position()
		    rx_rotate = curr_pos.offset(drx=-90)
		    rb.line(rx_rotate)
		    print('ROTATE to insert')
	
		    # move to rack 
		    # first_rack_hole+ down 70 => �open 
		    first_rack_hole_P = Position(302.14,  22.68,  99.34,  90.00,   0.00,  90.00)
		    first_rack_hole_J = Joint(71.81,  55.20, 150.10, -19.97,-113.95,-278.39)
		    first_rack_hole_above_70 = Position(302.14,  22.68, 169.34,  90.00,   0.00,  90.00)
		    first_rack_hole_above_70_J = Joint(71.59,  36.84, 150.82, -18.56, -97.26,-272.43)
				
		    ready_to_hole = first_rack_hole_P.offset(dz = 200)
		    rb.line(ready_to_hole)
		
		    rack_x, rack_y = 0, 0                                                                
		
		    none_position_count = none_count - 1
		    if none_position_count <= 10:
	             	rack_y = none_position_count * 10.744 
		    elif none_position_count <= 20:
		        rack_x = -10.744
			rack_y = (none_position_count * -10.744) -5.372
		    elif none_position_count <= 30:
		        rack_x =- 10.744
			rack_y = (none_position_count * 10.744)
		    rack_hole_position = first_rack_hole_above_70.offset(dx = rack_x, dy = rack_y)

		    rb.line(rack_hole_position)
		    down_for_insert = rack_hole_position.offset(dz =- 70)
		    rb.line(down_for_insert)
		    time.sleep(1)
		    gripper('open')
	            time.sleep(1)
		    curr_pos = position()
		    up = curr_pos.offset(dz = 270)
		    rb.Position2Joint(up)
		    rb.move(up)
		    
		    
		else:
		    print("No rack data received from client. Stopping filling process.")	
		    break

	    	none_count -= 1

	    rotate_for_pipette_gripper = up.offset(drx=90, dry =-90)	
	    rb.line(rotate_for_pipette_gripper)

	    print("Filling complete. Returning to YOLO detection.")
	    # back to pipette-tip and return value 1
	    print('go first_none_position: ')
	    print(first_none_position)
	    rb.line(first_none_position)
            value = 1
	    return value
        elif value == 1:
	    none_count = 0
	    return value
    
def send_bicker_3d_command(conn):
    send_command_to_client(conn, "bicker_3d")
    received_data = receive_data_from_client(conn)
    if received_data:
        print "Received data from client: {}".format(received_data)
        TT = received_data['a']
        RE = received_data['b']
        V = received_data['c']
        H = received_data['d']
        print(TT,RE,V,H)
        return TT,RE,V,H

def send_motor_command(conn):
	send_command_to_client(conn,"motor")
	received_data = receive_data_from_client(conn)
	if received_data:
		print "Received data from client motor : {}".format(received_data)
        Motorstate = received_data['state']
        print(Motorstate)
        return Motorstate


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
        
def position():  # return current position
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

tip_count = 0
tip_check = 0

def tip(n, conn):
    global tip_check
    # x, y, zip and tip up 
    global tip_count

    gripper('close')
    print 'tip'
    rb.move(n)    
    # set 2d camera for tip setting
   
    while tip_check == 0:
        time.sleep(1) 
        value = send_yolo_command(conn)
        if value == 1:  # tip O 
            tip_check = 1
            tip_count += 1
            print 'value == 1'
	    
            Tip_X, Tip_Y = send_pipette_command(conn) 
	    print 'Tip_x :', Tip_X, 'Tip_y :', Tip_Y
        
            curr_pos_1 = position()
            tip_pos = curr_pos_1.offset(dx=Tip_X, dy=Tip_Y)
            time.sleep(1)    
            rb.line(tip_pos)

        elif value != 1:  # tip x
            print 'value == 0 , tip x_ by yolo'
            #tip_n = rb.Joint2Position(n)
    	    tip_n = position()
            tip_count += 1
	    print('tip x tip count :')
	    print(tip_count)
            X, Y = 0, 0
            if tip_count <= 10:
                Y = 10.744
            else:
                group = (tip_count - 1) // 10
                position_in_group = tip_count % 10

                if group % 2 == 1:
                    if position_in_group == 1:
                        X = -10.744
                        Y = -5.372
                    else:
                        X = 0
                        Y = -10.744
                elif group % 2 == 0:
                    if position_in_group == 1:
                        X = -10.744
                        Y = -5.372
                    else:
                        X = 0
                        Y = 10.744

            tip_n = tip_n.offset(dx=X, dy=Y)
            tip_n = rb.Position2Joint(tip_n)
            rb.move(tip_n)
            # tip(tip_n, conn)
    tip_check = 0

    # Final steps to release the tip
    curr_pos = position()
    gripper('open')
    insert = curr_pos.offset(dz=-65.5)    
    rb.line(insert)    
    time.sleep(1)

    up = insert.offset(dz=100)
    print('up after attach tip to pipette')
    rb.move(up)


def aspirate():
	print('aspirate')
	curr_pos = position()
	gripper('close')	 
	down = curr_pos.offset(dz =-230)
	rb.line(down)
	gripper('open')
	time.sleep(2)
	up = down.offset(dz = 230)
	rb.line(up)

def dispense():
	print('dispense')
	curr_pos = position() 		
	for i in range(3):
		gripper('close')	
		time.sleep(0.5)
		gripper('open')
		time.sleep(0.5)
	gripper('close')
 
def pick(glass):
	curr_pos = position() 		
	rotate = curr_pos.offset(dj6=-90)
	before = glass.offset(dy=-100,dz=100)
	rb.line(before)
	rb.line(glass)
		
def ejector(length):
	global eject_count
	gripper('close')
	eject_count += 1
	back = Joint(166.24,  -5.54, 109.66, -95.29,-110.13,-105.06 ) # middle glass y=-200
	ready = Joint(200.40,  17.95,  87.86, -96.80,-112.68,-107.16)
	ready_p = rb.Joint2Position(ready)

	doking = ready_p.offset(dx =-75, dy =70)
	eject = doking.offset(dz = 25)
	rb.move(ready)
	time.sleep(1)
	rb.line(doking)
	time.sleep(1)
	rb.line(eject)
	time.sleep(1)
	down = eject.offset(dz=-20)
	doc = down.offset(dx=75,dy=-70)

	rb.line(down)
	rb.line(doc)
	rb.move(ready)
	up = Joint(199.58,  12.31,  79.70, -90.91,-114.43, -92.20 )
	#rb.move(back)
	rb.move(up)


# if you want tip to cap , Joint 120.99,  13.77,  87.43, -95.06,-113.94,-102.29 / dj4 -90 / 
# and go to ready 194.84,  15.23,  84.83,-179.89, -80.15,-72.2
Hya = 0
Tea = 0
Vai = 0
Ret = 0
def main(conn):             
    global Hya,Tea,Vai,Ret,order_num,eject_count
    try:                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          
        # the data to be received later via app communication. 
        namelist,iterlist,bottle_count = send_app_command(conn) 
        length = len(namelist) # ejector -> tip have length - 1
        order_num = bottle_count
        m = MotionParam(jnt_speed=20, lin_speed=10, pose_speed=20, overlap=0, acctime=0.4, dacctime=0.4)
        
        rb.motionparam(m)
        rb.override(90)
        rb.use_mt(True)
        
	# mount gripper on robot
	ready_for_mount_0 = Joint(86.62,   8.66, 133.13, -89.17, -91.58,-141.78)
	rb.move(ready_for_mount_0)
	print 'ready_for_mount_0'

	ready_for_mount_1 = Joint(119.58,  32.99, 130.97, -57.97, -83.14,-166.46)
	rb.move(ready_for_mount_1)
	
	print('ready_for_mount_1')
	
	gripper('open')
	ready_for_mount_2 = Joint(114.85,  34.08, 118.60, -66.95, -80.50,-155.01)
	ready_for_mount_2_p = Position(312.86, 258.13, 176.71, -90.01, -88.25,   0.09)
	rb.move(ready_for_mount_2)
	print('ready_for_mount_2')
	curr_pos = position()	
	step1 = curr_pos.offset(dx = 23)
	step2 = rb.Position2Joint(step1)
	step2 = step2.offset(dj6= 5)
	step3 = rb.Joint2Position(step2)	
	step3 = step3.offset(dz =-5)
	step4 = rb.Position2Joint(step3)
	step4 = step4.offset(dj6 =-5)
	step5 = rb.Joint2Position(step4)
	step5 = step5.offset(dz =1)
	step6 = step5.offset(dx =10)
	step7 = step6.offset(dz =-1)
	step8 = step7.offset(dx =-30)
	
	step9 = Joint( 95.69,  10.62, 130.36, -82.08, -85.87,-141.40 )
	
	rb.line(step1)
	print('1')
	rb.line(step2)
	print('2')
	rb.line(step3)
	print '3'
	rb.move(step4)
	print '4'
	rb.line(step5)
	print '5'
	rb.move(step6)
	print '6'
	rb.line(step7)
	print '7'
	rb.line(step8)
	print '8'
	rb.line(step9)
	print '9'

        # 3d bicker x, y 
        T,R,V,H = send_bicker_3d_command(conn) 
        print('H',H[0])
        print('T',T[0])
        print('R',R[0])
        print('V',V[0])
        
        # set bicker axis via depth 	
        HBicker = Position(x = H[0], y= H[1], z=465.49,rz=171.24,ry =-89.99,rx=94.20, posture = 3)
        TBicker = Position(x = T[0], y= T[1], z=465.49,rz=171.24,ry =-89.99,rx=94.20, posture = 3)
        RBicker = Position(x = R[0], y= R[1], z=465.49,rz=171.24,ry =-89.99,rx=94.20, posture = 3)
        VBicker = Position(x = V[0], y= V[1], z=465.49,rz=171.24,ry =-89.99,rx=94.20, posture = 3)

        # the date to be received vi depth camera
        
        goal_glasses = []
        for i in range(bottle_count):
            glass_1 = Joint(162.59,  19.23,  80.65, -94.47,-114.05, -99.24)
            glass_2 = Joint(168.70,  17.64,  82.93, -93.49,-108.00, -99.53)
            glass_3 = Joint(175.06,  16.72,  84.22, -92.32,-101.74, -99.58)
            glasses = [glass_1, glass_2, glass_3]
            goal_glasses.append(glasses[i])
	# glass_1_p = Position(90.29, 475.13, 441.02,  97.42, -88.40, -90.39)
	# first_tip_position_setting

	tip_n= Joint(74.59,  26.95,  95.83, -98.70,-102.84,-122.40)
	tip_n_P = Position(458.80, -72.08, 293.00,   8.78, -88.60, -98.72)
	first_tipJ= tip_n
        tip_count = 0 
        for n, c in zip(namelist,iterlist):
            print("n, c in zip (namelist, iterlist): ")
	    print(n, c)
            tip_count += 1 
            tip(tip_n,conn)
            print('---tip---')

            for glass in goal_glasses:
                for i in range(c):
                    print(c)
                    gripper('close')
                    if n == 'Hyaluronic acid':
                        print('hya')
                        Motor = send_motor_command(conn)		
			#Motor ="Done"
                        if Motor == "Done":
                            if Hya == 0:
                                rb.line(HBicker)
                                print('H',HBicker)
                                X, Y = send_bicker_2d_command(conn)
                                curr_pos = position()
                                set = curr_pos.offset(dx = X, dy = Y)
                                rb.line(set)
                                Hya_J = rb.Position2Joint(set)
                                Hya += 1
                                
                            rb.move(Hya_J)
                    elif n == 'Tea tree':
                        print('tea')
                        Motor = send_motor_command(conn)				
			#Motor = "Done"
                        if Motor == "Done":
                            if Tea == 0:
                                rb.line(TBicker)
                                print('T',TBicker)
                                X, Y = send_bicker_2d_command(conn)
                                curr_pos = position()
                                set = curr_pos.offset(dx = X, dy = Y)
                                rb.line(set)
                                Tea_J = rb.Position2Joint(set)
                                Tea += 1
                                rb.move(Tea_J)
            
                            rb.move(Tea_J)
                    elif n == "Vitamin":
                        print('vaitamin')
                        Motor = send_motor_command(conn)	
			#Motor = 'Done'
                        if Motor =="Done":

                            if Vai == 0:
                                print('vita = 0')
                                rb.line(VBicker)
                                print('V',VBicker)
                                X, Y = send_bicker_2d_command(conn)
                                curr_pos = position()
                                set = curr_pos.offset(dx = X, dy = Y)
                                rb.line(set)
                                Vai_J = rb.Position2Joint(set)
                                Vai += 1
                                rb.move(Vai_J)			
                            rb.move(Vai_J)

                    elif n == 'Retinol':
			
                        print('RE')
                        Motor = send_motor_command(conn)	
			#Motor = "Done"
                        if Motor =="Done":
                            if Ret == 0:
                                rb.line(RBicker)
                                print('R',RBicker)
                                X, Y = send_bicker_2d_command(conn)
                                curr_pos = position()
                                set = curr_pos.offset(dx = X, dy = Y)
                                rb.line(set)
                                Ret_J = rb.Position2Joint(set)
                                Ret += 1
                                rb.move(Ret_J)			
                            rb.move(Ret_J)

                    aspirate()
                    rb.move(glass)	
                    dispense()
                                

            ejector(length)
            if eject_count < length:
                    curr_pos = position()
                    curr_pos = rb.Position2Joint(curr_pos)
                    middle = curr_pos.offset(dj1 = -100)
                    rb.move(middle) # ejector -> tip 
                    time.sleep(2)

                    tip_n = rb.Joint2Position(tip_n)				
                    X, Y  = 0, 0
                    if tip_count <= 10:
                        Y = 10.744 
                    else:
                        group = (tip_count - 1) // 10
                        position_in_group = tip_count % 10

                        if group % 2 == 1:
                            if position_in_group == 1:
                                X = -10.744
                                Y = -5.372
                            else:	
                                X = 0
                                Y = -10.744
                        elif group % 2 == 0:
                            if position_in_group == 1:
                                X = -10.744
                                Y = -5.372
                            else:
                                X = 0
                                Y = 10.744
                                
                    tip_n = tip_n.offset(dx=X,dy=Y)
                    tip_n = rb.Position2Joint(tip_n)
	rb.line(first_tipJ)                
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
