ient_comm.py (Python 3)
import socket
import json

class RobotEnvClient:
    def __init__(self, host="192.168.1.23", port=12346):
        self.host = host
        self.port = port
        self.s = None

    def connect(self):
        try:
            self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.s.connect((self.host, self.port))
            print(f"Connected to server {self.host}:{self.port}")
        except socket.error as e:
            print(f"Connection error: {e}")

    def close(self):
        if self.s:
            self.s.close()
            print("Connection closed.")

    def send_data(self, data):
        try:
            if isinstance(data, (dict, list)):
                data = json.dumps(data)
            print("[DEBUG] sending:", repr(data))
            self.s.sendall((data + '\n').encode('utf-8'))
            print("[DEBUG] sendall() success")
            #print(f"Data sent to server: {data}")
        except socket.error as e:
            print(f"Error sending data: {e}")

    def receive_command(self):
        try:
            data = self.s.recv(4096).decode('utf-8')
            if data:
                messages = data.splitlines()
                for message in messages:
                    try:
                        command = json.loads(message)
                        print(f"Received command from server: {command}")
                        return command
                    except ValueError as e:
                        print(f"JSON Decode Error: {e}")
        except socket.error as e:
            print(f"Error receiving command: {e}")
        return None


if __name__ == "__main__":
    client = RobotEnvClient(host="127.0.0.1", port=12346)
    client.connect()

    try:
        while True:
            command = client.receive_command()
            if command:
                # 서버로부터 받은 명령(command)을 처리하는 부분
                # 예: 로봇 센서 데이터/상태를 다시 서버로 전송
                response = {"status": "ok", "received_command": command}
                client.send_data(response)
    except KeyboardInterrupt:
        print("\nClient terminated by user.")
    finally:
        client.close()

