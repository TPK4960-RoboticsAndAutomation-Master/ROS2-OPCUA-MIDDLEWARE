import socket
from threading import Thread

class Client:
    def __init__(self):
        HOST = '127.0.0.1'  # The server's hostname or IP address
        PORT = 50008        # The port used by the server

        self.soc = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.soc.connect((HOST, PORT))

        self.online = True

    def start(self):
        Thread(target = self.receive).start()
        Thread(target = self.send).start()

    def receive(self):
        while True:
            data = self.soc.recv(1024)
            if len(data) == 0:
                self.online = False
                break
            print('Received', repr(data), len(data))

    def send(self):
        while True:
            if not self.online:
                break
            msg = input("Enter message: ")
            self.soc.sendall(msg.encode("UTF-8"))


if __name__ == '__main__':
    cli = Client()
    cli.start()