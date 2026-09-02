# -*- coding: utf-8 -*-
# server_main.py (Python 2)
from server_comm import RobotEnvServer
import time

if __name__ == "__main__":
    server = RobotEnvServer(host="0.0.0.0", port=12346)
    server.start()

    try:
        # 1. 연결 확인
        start_cmd = {"command": "start"}
        print(">>> [Server] Sending start command: {}".format(start_cmd))
        server.send_command(start_cmd)

        response = server.receive_data()
        print("<<< [Server] Received: {}".format(response))
        time.sleep(1)

        # 2. object_detection_xyz 요청
        detect_cmd = {"command": "object_detection_xyz"}
        print(">>> [Server] Sending detection request: {}".format(detect_cmd))
        server.send_command(detect_cmd)

        response = server.receive_data()
        print("<<< [Server] Received: {}".format(response))
        time.sleep(1)

        # 3. transform_coordinate 요청과 로봇 좌표 전달, client응답 x
        robot_state = {"command": "transform_coordinate",
                       "value": [1.2, 2.3, 3.4, 0.1, 0.2, 0.3]}
        print(">>> [Server] Sending transform request: {}".format(robot_state))
        server.send_command(robot_state)
	time.sleep(1)
	
        # 4. state 요청
        state_cmd = {"command": "state"}
        print(">>> [Server] Sending: {}".format(state_cmd))
        server.send_command(state_cmd)

        response = server.receive_data()
        if response:
            print("<<< [Server] Full response: {}".format(response))

            # status와 coords 따로 출력
            status = response.get("status")
            coords = response.get("coords")
            print("<<< [Server] Status: {}".format(status))
            print("<<< [Server] Coords: {}".format(coords))

    except KeyboardInterrupt:
        print("\n[Server] Interrupted by user.")
    finally:
        server.close()

