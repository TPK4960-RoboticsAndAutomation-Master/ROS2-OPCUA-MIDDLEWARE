#!/usr/bin/env python3

import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, TransformStamped
from rclpy.utilities import remove_ros_args
import argparse

from sockets import TCPSocket, UDPSocket


def cl_red(msge): return '\033[31m' + msge + '\033[0m'
def cl_green(msge): return '\033[32m' + msge + '\033[0m'


class KmpCommandNode(Node):
    def __init__(self,connection_type,robot):
        super().__init__('kmp_command_node2')
        self.name = 'kmp_command_node2'
        self.declare_parameter('port')
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
        sub_twist = self.create_subscription(Twist, 'cmd_vel', self.twist_callback, 10)
        sub_pose = self.create_subscription(Pose, 'pose', self.pose_callback, 10)
        sub_shutdown = self.create_subscription(String, 'shutdown', self.shutdown_callback, 10)

        # Publishers
        self.kmp_status_publisher = self.create_publisher(String, 'kmp_status', 10)


        while not self.soc.isconnected:
            pass
        self.get_logger().info('Node is ready')


    def shutdown_callback(self, data):
        print(data)
        msg = "shutdown"
        self.soc.send(msg)
        self.soc.close()

    def twist_callback(self, data):
        msg = 'setTwist ' + str(data.linear.x) + " " + str(data.linear.y) + " " + str(data.angular.z)
        self.soc.send(msg)

    def pose_callback(self, data):
        msg = 'setPose ' + str(data.position.x) + " " + str(data.position.y) + " " + str(data.orientation.z)
        self.soc.send(msg)

    def publish_status(self, status):
        """
            'status' is either 0 (offline) or 1 (online).
        """
        msg = String()
        msg.data = "kmp_online" if status else "kmp_offline"
        self.kmp_status_publisher.publish(msg)

    def tear_down(self):
        try:
            self.destroy_node()
            rclpy.shutdown()
            print(cl_green("Successfully tore down kmp node"))
        except:
            print(cl_red('Error: ') + "rclpy shutdown failed")


def main(argv=sys.argv[1:]):
    parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('-c', '--connection')
    parser.add_argument('-ro', '--robot')
    args = parser.parse_args(remove_ros_args(args=argv))

    while True:
        rclpy.init(args=argv)
        kmp_command_node = KmpCommandNode(args.connection,args.robot)

        rclpy.spin(kmp_command_node)


if __name__ == '__main__':
    main()