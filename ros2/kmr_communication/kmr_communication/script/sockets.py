#!/usr/bin/env python3

import threading
import time
import os
import socket

def cl_black(msge): return '\033[30m' + msge + '\033[0m'
def cl_red(msge): return '\033[31m' + msge + '\033[0m'
def cl_green(msge): return '\033[32m' + msge + '\033[0m'
def cl_orange(msge): return '\033[33m' + msge + '\033[0m'
def cl_blue(msge): return '\033[34m' + msge + '\033[0m'
def cl_purple(msge): return '\033[35m' + msge + '\033[0m'
def cl_cyan(msge): return '\033[36m' + msge + '\033[0m'
def cl_lightgrey(msge): return '\033[37m' + msge + '\033[0m'
def cl_darkgrey(msge): return '\033[90m' + msge + '\033[0m'
def cl_lightred(msge): return '\033[91m' + msge + '\033[0m'
def cl_lightgreen(msge): return '\033[92m' + msge + '\033[0m'
def cl_yellow(msge): return '\033[93m' + msge + '\033[0m'
def cl_lightblue(msge): return '\033[94m' + msge + '\033[0m'
def cl_pink(msge): return '\033[95m' + msge + '\033[0m'
def cl_lightcyan(msge): return '\033[96m' + msge + '\033[0m'


class Socket:
    def __init__(self, ip, port, node_name, node):
        self.BUFFER_SIZE = 4096
        self.isconnected = False
        self.node = node
        self.node_name = node_name
        self.ip = ip
        self.port = port
        self.server_address = (ip, port)
        self.conn = None

        threading.Thread(target=self.connect_to_socket).start()

    def close(self):
        self.isconnected = False

    def init_socket(self, socket_type, option, option_value):
        print(cl_cyan('Starting up node:'), self.node_name, 'IP:', self.ip, 'Port:', self.port)
        try:
            self.conn = socket.socket(socket.AF_INET, socket_type)
            self.conn.setsockopt(socket.SOL_SOCKET, option, option_value)
            self.conn.bind(self.server_address)
        except:
           print(cl_red('Error: ') + "Connection for KUKA cannot assign requested address:", self.ip, self.port)

    def shutdown(self):
        print("SHUTTING DOWN")
        self.node.publish_status(0)
        self.conn.close()
        self.isconnected = False
        print(cl_lightred('Connection is closed!'))
        self.node.tear_down()

class TCPSocket(Socket):
    def __init__(self, ip, port, node_name, node):
        super().__init__(ip, port, node_name, node)

    def connect_to_socket(self):
        self.init_socket(socket.SOCK_STREAM, socket.SO_REUSEADDR, 1)

        self.conn.listen(3) # Server can accept 3 connections
        while (not self.isconnected):
            try:
                self.client_socket, client_address = self.conn.accept()
                self.conn.settimeout(0.01)
                self.isconnected = True
                self.node.publish_status(1)
                print(cl_green("Connected successfully!"))
            except:
                t=0

        time.sleep(1) 

        while self.isconnected:
            try:
                data = self.receive_message()
            except:
                t = 0

        #self.client_socket.shutdown(socket.SHUT_RDWR)
        #self.client_socket.close()
        #self.shutdown()

    def shutdown_(self):
        self.client_socket.shutdown(socket.SHUT_RDWR)
        self.client_socket.close()
        self.shutdown()
    
    def send(self, cmd):
        try:
            self.client_socket.sendall((cmd + '\r\n').encode("utf-8"))
        except:
            print(cl_red('Error: ') + "sending message thread failed")

    def receive_message(self):
        msg = self.client_socket.recv(1024).decode("utf-8")
        print(cl_lightblue(msg))
        if msg == "shutdown":
            self.close()
        return msg

class UDPSocket(Socket):
    def __init__(self, ip, port, node_name, node):
        super().__init__(ip, port, node_name, node)

    def connect_to_socket(self):
        self.init_socket(socket.SOCK_DGRAM, socket.SO_RCVBUF, 1048576)

        while (not self.isconnected):
            try:
                data, self.client_address = self.receive_message()
                self.isconnected = True
                self.node.publish_status(1)
                print(cl_green("Connected successfully!"))
                self.send("Hello from server!")
            except:
                t=0

        while self.isconnected:
            try:
                data, _ = self.receive_message()
            except:
                t = 0

        self.shutdown()


    def send(self, cmd):
        try:
            self.conn.sendto((cmd + '\r\n').encode("utf-8"), self.client_address)
        except:
            print(cl_red('Error: ') + "sending message thread failed")

    def receive_message(self):
        data, server = self.conn.recvfrom(self.BUFFER_SIZE)
        msg = data.decode("utf-8")
        print(cl_lightblue(msg))
        if msg == "shutdown":
            self.close()
        return msg, server
    