#!/usr/bin/env python3

import sys
import rclpy
import argparse
from std_msgs.msg import String, Float64
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
from rclpy.executors import MultiThreadedExecutor
from sockets import TCPSocket, UDPSocket

def cl_red(msge): return '\033[31m' + msge + '\033[0m'
def cl_green(msge): return '\033[32m' + msge + '\033[0m'


class LbrCommandNode(Node):
    def __init__(self,connection_type,robot):
        super().__init__('lbr_command_node')
        self.name = 'lbr_command_node'
        self.robot = robot
        self.declare_parameter('port')
        self.declare_parameter('id')
        self.id = self.get_parameter('id').value
        port = int(self.get_parameter('port').value)
        if robot == 'KMR':
            self.declare_parameter('KMR/ip')
            ip = str(self.get_parameter('KMR/ip').value)
        else:
            ip=None

        if connection_type == 'TCP':
            self.soc = TCPSocket(ip, port, self.name, self)
        elif connection_type == 'UDP':
            self.soc = UDPSocket(ip, port, self.name, self)
        else:
            self.soc=None
        
        # Make a listener for relevant topics
        sub_manipulator_vel = self.create_subscription(String, 'manipulator_vel', self.manipulator_vel_callback, 10)
        sub_shutdown = self.create_subscription(String, 'lbr_shutdown', self.shutdown_callback, 10)

        # Publishers
        self.lbr_status_publisher = self.create_publisher(String, 'lbr_status', 10)

        while not self.soc.isconnected:
            pass
            
        self.get_logger().info('Node is ready')

    def shutdown_callback(self, data):
        self.soc.send("")
        self.soc.close()

    def manipulator_vel_callback(self, data):
        msg = 'setLBRmotion ' + data.data
        self.soc.send(msg)

    def publish_status(self, status):
        """
            'status' is either 0 (offline) or 1 (online).
        """
        msg = String()
        msg.data = self.id + ":" + self.robot + ":lbr:" + str(status)
        self.lbr_status_publisher.publish(msg)

    def tear_down(self):
        try:
            self.destroy_node()
            rclpy.shutdown()
            print(cl_green("Successfully tore down lbr node"))
        except:
            print(cl_red('Error: ') + "rclpy shutdown failed")


def main(argv=sys.argv[1:]):
    parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('-c', '--connection')
    parser.add_argument('-ro', '--robot')
    args = parser.parse_args(remove_ros_args(args=argv))

    while True:
        rclpy.init(args=argv)
        lbr_command_node = LbrCommandNode(args.connection,args.robot)

        rclpy.spin(lbr_command_node)


if __name__ == '__main__':
    main()