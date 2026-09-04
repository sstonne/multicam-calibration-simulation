import socket, json
    			
class RobotEnvServer:
    def __init__(self, host="0.0.0.0", port=12346):
        self.host = host
        self.port = port
        self.s = None
        self.conn = None
        self.addr = None

    def start(self):
        try:
            self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.s.bind((self.host, self.port))
            self.s.listen(1)
            print "Server started. Waiting for client connection..."

            self.conn, self.addr = self.s.accept()
            #print "Connected to client {}".format(self.addr)

        except socket.error as e:
            print "Socket error: {}".format(e)

    def close(self):
        if self.s:
            self.s.close()
            print "Server closed."

    def send_command(self, command_message):
        try:
            if isinstance(command_message, (dict,list)):
                command_message = json.dumps(command_message)

            self.conn.sendall((command_message + '\n').encode('utf-8'))
            #print "Command sent to client: {}".format(command_message)

        except socket.error as e:
            print "Error sending command to client: {}".format(e)

    def receive_data(self):
        try:
            data = self.conn.recv(4096).decode('utf-8')
            if data:
                messages = data.splitlines()
                for message in messages:
                    try:
                        received_data = json.loads(message)
                        print "Received data from client: {}".format(received_data)
                        return received_data
                    except ValueError as e: 
                        print "JSON Decode Error: {}".format(e)
        except socket.error as e:
            print "Error receiving data from client: {}".format(e)
        return None
