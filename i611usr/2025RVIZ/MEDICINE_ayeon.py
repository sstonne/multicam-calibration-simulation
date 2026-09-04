# -*- coding: utf-8 -*-
#!/usr/bin/python
from i611_MCS import *
from i611_extend import *
from i611_common import *
from i611_io import *
from i611shm import *
from rbsys import *
import sys, time, json, select
from server_comm import RobotEnvServer


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


def comm(server):
    raw = server.receive_data()
    if not raw:
        print "[SERVER] No data from client."
        return None
    if isinstance(raw, basestring):
        try:
            data = json.loads(raw)
        except Exception as e:
            print "[SERVER] JSON decode error:", e
            return None
    else:
        data = raw
    return data


class MedicineHandoverServer:
    """FACETRACKING 구조 기반 — 약 전달 및 제거 서버"""

    def __init__(self, rb, server, log_filename="medicine_handover_server_log.txt"):
        sys.stdout = Logger(log_filename)
        self.rb = rb
        self.server = server
        print "[SERVER] MedicineHandoverServer initialized."

        # 로봇 모션 기본 파라미터
        self.DEFAULT_PARAM = MotionParam(jnt_speed=20, lin_speed=50, overlap=50)

        # 카메라 뷰 자세
        self.CAMERA_VIEW_JNT = [-13.73, -17.60, -81.51, -180.00, 80.87, -13.73]
        self.CAMERA_VIEW_POS = [24.87, 523.13, 478.43, -90.00, 0.01, -180.00]

        # 폐기 위치 목록
        self.DISCARD_JOINTS = [
            [14.16, -35.74, -95.54, -180.00, 48.72, 14.16],
            [9.73, -47.87, -70.38, -180.00, 61.75, 9.73],
            [16.94, -22.84, -120.78, -180.00, 36.38, 16.94],
            [-2.63, -56.36, -52.22, -180.00, 71.42, -2.63]
        ]

        self.discard_index = 0
        self.has_discarded = False


    # =====================================================
    # pose 전송
    # =====================================================
    def send_robot_pose(self, state="idle"):
        rb = self.rb
        server = self.server
        try:
            current_pose = rb.getpos()
            pose_dict = current_pose.pos2dict()
            rb.changetool(1)
            pose_dict.pop("parent", None)
            msg = {"action": "update_pose", "state": state, "robot_pose": pose_dict}
            server.send_command(msg)
            print "[SERVER] Pose updated:", pose_dict
        except Exception, e:
            print "[SERVER] Error sending pose:", e


    # =====================================================
    # 카메라 뷰로 이동
    # =====================================================
    def move_to_camera_view(self):
        rb = self.rb
        server = self.server
        try:
            rb.motionparam(self.DEFAULT_PARAM)
            target = Joint(*self.CAMERA_VIEW_JNT)
            print "[SERVER] → Moving to camera view position:", self.CAMERA_VIEW_POS
            rb.asyncm(2)
            rb.move(target)
            time.sleep(0.5)
            self.send_robot_pose("camera_view_ready")
            return True
        except Exception, e:
            print "[SERVER] Error moving to camera view:", e
            return False


    # =====================================================
    # 타겟으로 이동 (상대이동)
    # =====================================================
    def move_to_target(self, x, y, z, rz, state="moving"):
        rb = self.rb
        try:
            rb.motionparam(self.DEFAULT_PARAM)
            z_lower_limit = -260.0
            if z < z_lower_limit:
                print "[WARN] dz %.1f below limit → limiting to %.1f" % (z, z_lower_limit)
                z = z_lower_limit
            print "[SERVER] → Moving to:", (x, y, z, rz)
            rb.relline(dx=x, dy=y, dz=z, drz=rz)
            rb.asyncm(2)
            time.sleep(1)
            return True
        except Exception, e:
            print "[SERVER] Error moving:", e
            return False


    # =====================================================
    # 픽 앤 기브 동작
    # =====================================================
    def pick_and_give(self, target_pos):
        rb = self.rb
        server = self.server
        HANDOVER_JOINT = [18.38, -57.58, -44.65, -181.57, 76.37, -13.72]

        x, y, z, rz = target_pos
        print "[SERVER] Pick and give at (%.1f, %.1f, %.1f)" % (x, y, z)

        server.send_command({"action": "open_gripper", "data": {"open_mm": 85}})
        self.move_to_target(x, y, z + 100, rz)
        rb.relline(dz=-100)
        server.send_command({"action": "open_gripper", "data": {"open_mm": 20, "force": 0}})
        time.sleep(1)
        rb.relline(dz=50)
        time.sleep(1)
        server.send_command({"action": "open_gripper", "data": {"open_mm": 0, "force": 0}})
        time.sleep(1)
        server.send_command({"action": "check_grip_status"})

        grip_ok = self.wait_for_grip_confirmation()
        if not grip_ok:
            print "[SERVER] Grip failed — retrying..."
            self.move_to_camera_view()
            server.send_command({"action": "start_detect"})
            return

        rb.relline(dz=150)
        rb.motionparam(self.DEFAULT_PARAM)
        rb.asyncm(2)
        rb.move(Joint(*HANDOVER_JOINT))
        time.sleep(0.5)
        server.send_command({"action": "open_gripper", "data": {"open_mm": 85}})
        time.sleep(0.5)
        print "[SERVER] Handover complete"

        if self.has_discarded:
            print "[SERVER] Restoring previously discarded medicines..."
            rb.relline(dz=100)
            self.restore_discarded_items()
            self.has_discarded = False
            server.send_command({"action": "stop"})
            print "[SERVER] Task done."
        else:
            print "[SERVER] No discarded items. Returning to camera view."
            self.move_to_camera_view()
            server.send_command({"action": "stop"})


    # =====================================================
    # 폐기된 약 복원
    # =====================================================
    def restore_discarded_items(self):
        rb = self.rb
        server = self.server
        CLOSE_JOINT = [-13.73, -26.60, -100.74, -180.00, 52.64, -13.73]

        for idx in reversed(range(self.discard_index)):
            joint_pos = self.DISCARD_JOINTS[idx]
            print "[SERVER] Restoring medicine #%d" % (idx + 1)
            server.send_command({"action": "open_gripper", "data": {"open_mm": 85}})
            rb.motionparam(self.DEFAULT_PARAM)
            rb.move(Joint(*joint_pos))
            server.send_command({"action": "open_gripper", "data": {"open_mm": 0, "force": 0}})
            time.sleep(1)
            rb.relline(dz=200)
            rb.motionparam(self.DEFAULT_PARAM)
            rb.asyncm(2)
            rb.move(Joint(*CLOSE_JOINT))
            server.send_command({"action": "open_gripper", "data": {"open_mm": 85}})
            time.sleep(0.5)
        print "[SERVER] All discarded medicines restored."
        self.discard_index = 0


    # =====================================================
    # 그립 확인
    # =====================================================
    def wait_for_grip_confirmation(self, timeout=3.0):
        server = self.server
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
                print "[SERVER] Error waiting for grip status:", e
        print "[SERVER] Grip status timeout"
        return False


    # =====================================================
    # 장애물 제거
    # =====================================================
    def remove_obstacle(self, target_pos):
        rb = self.rb
        server = self.server
        x, y, z, rz = target_pos
        print "[SERVER] Removing obstacle at (%.1f, %.1f, %.1f)" % (x, y, z)

        server.send_command({"action": "open_gripper", "data": {"open_mm": 85}})
        self.move_to_target(x, y, z + 100, rz)
        rb.relline(dz=-100)
        server.send_command({"action": "open_gripper", "data": {"open_mm": 20, "force": 0}})
        time.sleep(1)
        rb.relline(dz=50)
        server.send_command({"action": "open_gripper", "data": {"open_mm": 0, "force": 0}})
        time.sleep(1)
        server.send_command({"action": "check_grip_status"})
        grip_ok = self.wait_for_grip_confirmation()

        if not grip_ok:
            print "[SERVER] Grip failed — retrying..."
            self.move_to_camera_view()
            server.send_command({"action": "start_detect"})
            return

        rb.relline(dz=150)
        target_joint = Joint(*self.DISCARD_JOINTS[self.discard_index])
        rb.motionparam(self.DEFAULT_PARAM)
        rb.move(target_joint)
        server.send_command({"action": "open_gripper", "data": {"open_mm": 85}})
        print "[SERVER] Obstacle removed"
        self.has_discarded = True
        self.discard_index = (self.discard_index + 1) % len(self.DISCARD_JOINTS)
        self.move_to_camera_view()
        server.send_command({"action": "start_detect"})


    # =====================================================
    # 메인 루프
    # =====================================================
    def main(self):
        rb = self.rb
        server = self.server
        print "[SERVER] Main loop started."
        client_ready = False
        self.server.send_command({"action": "start_detect"})
        print 'self.server.send_command({"action": "start_detect"})'
        data  = comm(self.server)
        print 'comm(self.server)'
        action = data.get("action")
    

        if action == "start_detect":
            client_ready = True
            print '[SERVER] client ready!'

        while True:
            if client_ready:
                print '[SERVER] client ready, start_detect..'
                action = data.get("action")
                d = data.get("data", {})

                if action == "set_symptom":
                    symptom = d.get("symptom")
                    required_class = d.get("required_class")
                    print "[SERVER] Symptom='%s' → Required='%s'" % (symptom, required_class)
                    self.move_to_camera_view()
                    time.sleep(2.0)
                    client_ready = True
                    server.send_command({"action": "start_detect"})
                    self.send_robot_pose("ready")

                elif action == "handover_detected":
                    sub_action = d.get("action")
                    cls_name = d.get("cls")
                    xyz = d.get("xyz", [0, 0, 0])
                    rz = d.get("drz", 0)
                    xyz[2] += 215
                    print "[SERVER] YOLO Detected: %s → %s" % (cls_name, sub_action)
                    if sub_action == "pick_and_give":
                        self.pick_and_give([xyz[0], xyz[1], xyz[2], rz])
                    elif sub_action == "remove":
                        self.remove_obstacle([xyz[0], xyz[1], xyz[2], rz])

                elif action == "move_recover":
                    print "[SERVER] Recovery move (dz=+30, dx=+10)"
                    try:
                        rb.motionparam(self.DEFAULT_PARAM)
                        rb.relline(dz=30, dx=10)
                        time.sleep(0.5)
                        print "[SERVER] Recovery done."
                    except Exception, e:
                        print "[SERVER] Recovery failed:", e
                    server.send_command({"action": "start_detect"})

                elif action == "stop":
                    print "[SERVER] Stop command received."
                    self.send_robot_pose("stopped")
                    break

                else:
                    print "[SERVER] Unknown action:", action


    # =====================================================
    # 실행
    # =====================================================
    def run(self):
        print "[SERVER] Running MedicineHandoverServer..."
        try:
            self.rb.motionparam(self.DEFAULT_PARAM)
            self.main()
        except KeyboardInterrupt:
            print "[SERVER] Interrupted by user."
        except Exception, e:
            print "[SERVER] Error:", e
        finally:
            self.server.close()
            print "[SERVER] Closed cleanly."

