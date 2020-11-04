import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float64
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, TransformStamped
from rclpy.qos import qos_profile_sensor_data
from rclpy.utilities import remove_ros_args
import argparse
from rclpy.action import ActionServer, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor




class TestNode(Node):
    def __init__(self):
        super().__init__('test_node')
        print("Init node")


def main(argv=sys.argv[1:]):

    rclpy.init()
    test_node = TestNode()

    #executor = MultiThreadedExecutor()
    rclpy.spin(test_node)

    try:
        test_node.destroy_node()
        rclpy.shutdown()
    except:
        print(cl_red('Error: ') + "rclpy shutdown failed")


if __name__ == '__main__':
    main()
