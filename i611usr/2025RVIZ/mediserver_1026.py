# -*- coding: utf-8 -*-
#!/usr/bin/python

from i611_MCS import *
from i611_extend import *
from i611_common import *
from i611_io import *
from i611shm import *
from rbsys import *
reload(sys)
sys.setdefaultencoding('utf-8')
from server_comm import RobotEnvServer
import time, sys, json, select

# ------------------------
# 로그 기록
# ------------------------
log_filename = "medicine_handover_server_log.txt"

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

# ------------------------
# 서버 시작
# ------------------------
server = RobotEnvServer()
server.start()
print "[SERVER] Server started, waiting for client connection..."
server.send_command({"action": "start"})

# 카메라 고정 관찰 자세 (예: 약상자 전체를 내려다보는 위치)
#CAMERA_VIEW_POS = [24.88, 523.15, 418.38, -90.00,  -0.00, 180.00] #겁나 뚱뚱한거
CAMERA_VIEW_POS = [24.87, 523.13, 478.43, -90.00,   0.01,-180.00] # 앞으로 1센치 나온거
#CAMERA_VIEW_JNT = [-13.73, -19.25, -88.71,-180.00,  72.03, -13.73]  
CAMERA_VIEW_JNT = [-13.73, -17.60, -81.51,-180.00,  80.87, -13.73]
DEFAULT_PARAM = MotionParam(jnt_speed=20, lin_speed=50, overlap=50)
#버림 위치 
DISCARD_JOINTS= [
    [14.16, -35.74, -95.54,-180.00,  48.72,  14.16],  #앞으로만 나온거
    [9.73, -47.87, -70.38,-180.00,  61.75,   9.73], 
    [16.94, -22.84,-120.78,-180.00,  36.38,  16.94], 
    [-2.63, -56.36, -52.22,-180.00,  71.42,  -2.63]
]
discard_index = 0  # 현재 쓸 버림 위치 인덱스
has_discarded = False  # 약을 버린 적이 있는지 추적

# ------------------------
# Pose 전송 함수
# ------------------------
def send_robot_pose(rb, state="idle"):
    """현재 로봇 pose를 클라이언트로 전송 + 현재 상태(state) 포함"""
    try:
        #rb.changetool(2)
        current_pose = rb.getpos()  # 원래 rb.where() 사용 가능
        pose_dict = current_pose.pos2dict()
        rb.changetool(1)

        if "parent" in pose_dict:
            del pose_dict["parent"]

        msg = {
            "action": "update_pose",
            "state": state,
            "robot_pose": pose_dict
        }
        server.send_command(msg)
        print "[SERVER] Pose updated (state=%s):" % state, pose_dict
    except Exception, e:
        print "[SERVER]!!!!!!!!!!Error sending pose:", e

# ------------------------
# 로봇 관련 함수
# ------------------------
def move_to_target(rb, x, y, z, rz, state="moving"):
    """YOLO에서 받은 좌표로 로봇 이동"""
    try:
        rb.motionparam(DEFAULT_PARAM)
        target = rb.relline(dx = x, dy = y, dz = z, drz = rz)
        print "[SERVER] → Moving to:", (x, y, z, rz)
        rb.asyncm(2)
        #rb.line(target)
        time.sleep(1)
        return True
    except Exception, e:
        print "[SERVER] Error moving:", e
        return False
    
def move_to_camera_view(rb):
    """약상자 전체를 볼 수 있는 고정 카메라 위치로 이동"""
    try:
        rb.motionparam(DEFAULT_PARAM)
        target = Joint(*CAMERA_VIEW_JNT)
        print "[SERVER] → Moving to camera view position:", CAMERA_VIEW_POS
        rb.asyncm(2)
        rb.move(target)
        time.sleep(0.5)
        send_robot_pose(rb, "camera_view_ready")
        return True
    except Exception, e:
        print "[SERVER] Error moving to camera view:", e
        return False

def pick_and_give(rb, target_pos):
    """약을 집고 전달하는 동작"""
    global has_discarded 

    HANDOVER_JOINT = [ 18.38, -57.58, -44.65,-181.57,  76.37, -13.72]

    x, y, z, rz = target_pos
    print "[SERVER] Pick and give at ({},{},{})".format(x, y, z)
    server.send_command({"action": "open_gripper", "data": {"open_mm": 85}})
    #time.sleep(0.8)

    # 약 상단으로 이동
    move_to_target(rb, x, y, z + 100, rz, "approaching_target")

    # 내려가서 픽
    rb.relline(dz = -100)
    server.send_command({"action": "open_gripper",
    "data": {"open_mm": 20, "force":0}})
    time.sleep(1)
    #send_robot_pose(rb, "grip_closed")

    # 들어올리기
    rb.relline(dz = 50)
    time.sleep(1)
    print "[SERVER] Re-checking grip status..."
    server.send_command({"action": "open_gripper",
    "data": {"open_mm": 0, "force":0}})
    time.sleep(1)
    server.send_command({"action": "check_grip_status"})
    grip_ok = wait_for_grip_confirmation(timeout=3.0)

    if not grip_ok:
        print "[SERVER] ⚠ Grip failed — medicine not held. Retrying..."
        move_to_camera_view(rb)
        server.send_command({"action": "start_detect"})
        return  # 이번 픽업 중단, 다시 YOLO 감지
    rb.relline(dz = 150)
    # 전달 위치 
    target_joint = Joint(*HANDOVER_JOINT)
    rb.motionparam(DEFAULT_PARAM)
    rb.asyncm(2)
    rb.move(target_joint)
    time.sleep(0.5)

    server.send_command({"action": "open_gripper", "data": {"open_mm": 85}})
    time.sleep(0.5)
    #send_robot_pose(rb, "handover_done")
    print "[SERVER] Handover complete"

    if has_discarded:
        print "[SERVER] Restoring previously discarded medicines..."
        rb.relline(dz = 100)
        restore_discarded_items(rb)
        has_discarded = False  # 복구 완료 후 초기화
        #send_robot_pose(rb, "finished")
        server.send_command({"action": "stop"})
        print "[SERVER] Task done."
        return 
    else:
        print "[SERVER] No discarded items. Returning to camera view."
        move_to_camera_view(rb)
        #send_robot_pose(rb, "finished")
        server.send_command({"action": "stop"})
        return 

def restore_discarded_items(rb):
    """버려둔 약들을 다시 담는 동작"""
    ClOSE_JOINT = [-13.73, -26.60,-100.74,-180.00,  52.64, -13.73]
    print "[SERVER] Restoring discarded medicines..."
    global discard_index, has_discarded
    has_discarded = True
    # 최근 버림 위치부터 역순으로 회수 (가장 마지막에 놓은 약부터)
    #for idx in reversed(range(len(DISCARD_JOINTS))):
    for idx in reversed(range(discard_index)):
        joint_pos = DISCARD_JOINTS[idx]
        print "[SERVER] → Moving to discarded position #%d: %s" % (idx+1, joint_pos)

        server.send_command({"action": "open_gripper", "data": {"open_mm": 85}})
        rb.motionparam(DEFAULT_PARAM)
        rb.move(Joint(*joint_pos))
        #time.sleep(0.5)

        # 그리퍼 닫아서 약 잡기
        server.send_command({"action": "open_gripper",
    "data": {"open_mm": 0, "force":0}})
        time.sleep(1)

        # 약을 집은 상태에서 들어올리기
        rb.relline(dz=200)
        #time.sleep(0.3)

        # 카메라 뷰로 돌아와 놓기
        finish_joint = Joint(*ClOSE_JOINT)
        rb.motionparam(DEFAULT_PARAM)
        rb.asyncm(2)
        rb.move(finish_joint)
        server.send_command({"action": "open_gripper", "data": {"open_mm": 85}})
        time.sleep(0.5)
        print "[SERVER] Medicine #%d restored to camera view." % (idx+1)

    print "[SERVER] All discarded medicines restored."
    #send_robot_pose(rb, "restore_done")

    # 회수 완료 후 discard_index 초기화
    discard_index = 0

def wait_for_grip_confirmation(timeout=3.0):
    """클라이언트에서 그리퍼 상태 회신 대기"""
    start = time.time()
    while time.time() - start < timeout:
        try:
            rlist, _, _ = select.select([server.conn], [], [], 0.1)
            if not rlist:
                continue
            data = server.receive_data()
            if data and data.get("action") == "grip_status":
                return data.get("data", {}).get("held", False)
        except Exception, e:
            print "[SERVER] Error waiting for grip confirmation:", e
    print "[SERVER] Grip status timeout"
    return False


def remove_obstacle(rb, target_pos):
    """가장 가까운 약 제거"""
    global discard_index, has_discarded
    x, y, z, rz = target_pos
    print "[SERVER] Removing obstacle at ({},{},{})".format(x, y, z)
    server.send_command({"action": "open_gripper", "data": {"open_mm": 85}})
    #time.sleep(0.8)

    move_to_target(rb, x, y, z + 100, rz, "approaching_obstacle")
    rb.relline(dz = -100)
    server.send_command({"action": "open_gripper",
    "data": {"open_mm": 20, "force":0}})
    time.sleep(1)
    rb.relline(dz = 50)
    time.sleep(1)
    print "[SERVER] Re-checking grip status..."
    server.send_command({"action": "open_gripper",
    "data": {"open_mm": 0, "force":0}})
    time.sleep(1)
    server.send_command({"action": "check_grip_status"})
    grip_ok = wait_for_grip_confirmation(timeout=3.0)

    if not grip_ok:
        print "[SERVER] Grip failed — medicine not held. Retrying..."
        move_to_camera_view(rb)
        server.send_command({"action": "start_detect"})
        return  # 이번 픽업 중단, 다시 YOLO 감지
    rb.relline(dz = 150)
    target_joint = DISCARD_JOINTS[discard_index]
    rb.motionparam(DEFAULT_PARAM)
    target = Joint(*target_joint)
    print "[SERVER] → Moving to discard position:",target_joint
    rb.asyncm(2)
    rb.move(target)  # 절대 조인트 이동
    #time.sleep(0.5)

    server.send_command({"action": "open_gripper", "data": {"open_mm": 85}})
    #time.sleep(1)
    #send_robot_pose(rb, "obstacle_removed")
    print "[SERVER] Obstacle removed"
    has_discarded = True
    discard_index = (discard_index + 1) % len(DISCARD_JOINTS)

    move_to_camera_view(rb)
    time.sleep(1)
    # YOLO 재탐지 시작
    server.send_command({"action": "start_detect"})
    print "[SERVER] YOLO re-detection started."

# ------------------------
# 메인 루프
# ------------------------
def main(rb):
    print "[SERVER] Main loop started."
    #rb.settool(1, 0.0, 0.0, 245.0, 0.0, 0.0, 0.0)
    #rb.changetool(1)
    #rb.settool(2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

    client_ready = False

    while True:
        try:
            rlist, _, _ = select.select([server.conn], [], [], 0.01)
            if not rlist:
                continue
            data = server.receive_data()
        except Exception, e:
            print "[SERVER] Error receiving data:", e
            continue

        if not data:
            continue

        action = data.get("action")
        d = data.get("data", {})

        # --- 클라이언트 준비 ---
        if action == "set_symptom":
            symptom = d.get("symptom")
            required_class = d.get("required_class")
            print "[SERVER] Symptom='{}' → Required medicine='{}'".format(symptom, required_class)
            move_to_camera_view(rb)
            print "[SERVER] Waiting 2 seconds before YOLO detection..."
            time.sleep(2.0)
            client_ready = True
            server.send_command({"action": "start_detect"})
            send_robot_pose(rb, "ready")
            continue

        # --- YOLO 탐지 결과 수신 ---
        elif action == "handover_detected":
            sub_action = d.get("action")
            cls_name = d.get("cls")
            xyz = d.get("xyz", [0,0,0])
            #xyz[2] +=230
            rz = d.get("drz", 0)
            z_corrected = xyz[2] + 215  # 245mm 내려서 그리퍼 중심 기준 맞추기
	    xyz[2] = z_corrected
            print "[SERVER] YOLO Detected: {} ({}) → Action: {}".format(cls_name, xyz, sub_action)

            if sub_action == "pick_and_give":
                pick_and_give(rb, [xyz[0], xyz[1], xyz[2], rz])
            elif sub_action == "remove":
                remove_obstacle(rb, [xyz[0], xyz[1], xyz[2], rz])
            else:
                print "[SERVER] Unknown sub-action:", sub_action
            continue
                # --- 클라이언트에서 보낸 복구 이동 명령 ---

        elif action == "move_recover":
            print "[SERVER] Recovery move detected → dz = +30mm, dx = +10mm(fixed)"
            try:
                rb.motionparam(DEFAULT_PARAM)
                rb.relline(dz=30, dx =10)
                time.sleep(0.5)
                print "[SERVER] Robot moved up by +30mm for better detection view."
            except Exception, e:
                print "[SERVER] Recovery move failed:", e

            # 이동 완료 후 다시 YOLO 탐지 시작
            server.send_command({"action": "start_detect"})
            print "[SERVER] YOLO re-detection started after recovery."
            continue

        elif action == "stop":
            print "[SERVER] Stop command received"
            send_robot_pose(rb, "stopped")
            break

        else:
            print "[SERVER] Unknown or unhandled action:", action

# ------------------------
# 실행부
# ------------------------
if __name__ == "__main__":
    try:
        rb = i611Robot()
        rb.open()
        main(rb)
        rb.close()
        server.close()
    except KeyboardInterrupt:
        print 'KeyboardInterrupt → Exiting'
        rb.exit(0)
        rb.close()
    except Exception, e:
        print 'Error:', e.__class__.__name__, ':', e
        rb.exit(0)
        rb.close()

