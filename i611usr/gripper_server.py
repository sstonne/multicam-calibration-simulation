#!/usr/bin/python
# -*- coding: utf-8 -*-
import time
import json
from server_comm import RobotEnvServer

def decode_message(raw):
    """수신된 raw를 dict로 변환 (str/json/dict 모두 수용)"""
    if raw is None:
        return None
    if isinstance(raw, dict):
        return raw
    if isinstance(raw, basestring):   # Python2: str/unicode 모두 처리
        try:
            return json.loads(raw)
        except Exception as e:
            print "[SERVER] JSON decode error:", e, raw
            return None
    print "[SERVER] Unsupported message type:", type(raw)
    return None

def comm_once(server):
    raw = server.receive_data()
    data = decode_message(raw)
    if not data:
        print "[SERVER] No/invalid data from client."
    return data

def send_gripper_and_wait(server, state, mode="full", timeout=8.0, poll=0.05):
    """
    클라이언트에 그리퍼 명령 전송 후 end/notConnected/error 응답까지 대기
    """
    cmd = {"action": "gripper", "state": state, "mode": mode}
    server.send_command(cmd)
    print "[SERVER] ->", cmd

    t0 = time.time()
    while time.time() - t0 < timeout:
        data = comm_once(server)
        if not data:
            time.sleep(poll)
            continue

        print "[SERVER] <-", data
        if data.get("action") == "gripper":
            status = data.get("status")
            if status in ("end", "notConnected", "error"):
                return data  # 최종 응답
        time.sleep(poll)

    return {"status": "error", "action": "gripper", "detail": "timeout"}

def main():
    # 필요 시 핸드셰이크
    # server.send_command({"action":"start"})
    # ready_msg = comm_once(server)
    # print "[SERVER] ready:", ready_msg

    print "[SERVER] open short"
    print send_gripper_and_wait(server, "open", mode="short")

    print "[SERVER] open long"
    print send_gripper_and_wait(server, "open", mode="long")

    print "[SERVER] open full"
    print send_gripper_and_wait(server, "open", mode="full")

    print "[SERVER] close"
    print send_gripper_and_wait(server, "close")

if __name__ == '__main__':
    server = RobotEnvServer()
    server.start()
    print "[SERVER] Waiting for client... (client connects and listens)"
    try:
        main()
    finally:
        server.close()
        print "[SERVER] closed"

