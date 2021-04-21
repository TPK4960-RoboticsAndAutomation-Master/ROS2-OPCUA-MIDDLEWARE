#!/usr/bin/env python3

import sys
import rclpy
import argparse
from time import sleep
from std_msgs.msg import String
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, TransformStamped
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.utilities import remove_ros_args
from opcua import ua, Client

class PubSub(Node):
    def __init__(self, component, ua_obj):
        super().__init__(component + '_hybrid_node')
        self.component = component
        self.server_obj = ua_obj

        self.status_subscriber = self.create_subscription(String, self.component + '_status', self.status_callback, 10)
        
        self.status_checker_publisher = self.create_publisher(String, "status_check", 10)
        msg = String()
        msg.data = "status_please"
        self.status_checker_publisher.publish(msg)

    def event_notification(self, event):
        pass

    def status_callback(self, msg):
        rid = msg.data.split(":")[0]
        print("status update callback from " + self.component + " with RID: " + rid)
        method = "update_status"
        self.server_obj.call_method("2:" + method, str(msg.data))

class LBRPubSub(PubSub):
    def __init__(self, ua_obj):
        super().__init__('lbr', ua_obj)

    def event_notification(self, event):
        msg = String()
        msg.data, rid = event.Message.Text.split(",")
        
        publisher = self.create_publisher(String, "manipulator_vel_" + str(rid), 10)
        shutdown_publisher = self.create_publisher(String, self.component + '_shutdown_' + str(rid), 10)
        
        if msg.data == "shutdown":
            shutdown_publisher.publish(msg)
        else:
            publisher.publish(msg)
        

class KMPPubSub(PubSub):
    def __init__(self, ua_obj):
        super().__init__('kmp', ua_obj)

    def event_notification(self, event):
        """
            event.Message.text = "speed x y th", e.g. "0.5 0 1 0"
        """
        
        action, rid = event.Message.Text.split(",")
        publisher = self.create_publisher(Twist, "cmd_vel_" + str(rid), 10)
        shutdown_publisher = self.create_publisher(String, self.component + '_shutdown_' + str(rid), 10)

        e = action.split(" ")

        if e[0] == "shutdown":
            msg = String()
            msg.data = "shutdown"
            shutdown_publisher.publish(msg)
        else:
            speed = float(e[0])
            twist = Twist()
            twist.linear.x = float(e[1])*speed
            twist.linear.y = float(e[2])*speed
            twist.linear.z = 0.0
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = float(e[3])*speed #or turn
            publisher.publish(twist)

class CameraPubSub(PubSub):
    def __init__(self, ua_obj):
        super().__init__('camera', ua_obj)

    def event_notification(self, event):        
        action, rid, *kwargs = event.Message.Text.split(",")
        camera_publisher = self.create_publisher(String, 'handle_camera_' + str(rid), 10)

        msg = String()
        msg.data = action
        camera_publisher.publish(msg)


def main(argv=sys.argv[1:]):
    parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('-d', '--domain')
    args = parser.parse_args(remove_ros_args(args=argv))

    rclpy.init(args=None)
    
    """ 
        OPC UA CLIENT 
        For receiving commands from AAS
    """
    isConnected = False
    #opcua_client = Client("opc.tcp://andrcar-master.ivt.ntnu.no:4841/freeopcua/server/")
    opcua_client = Client("opc.tcp://" + args.domain + ":4841/freeopcua/server/")

    while not isConnected:
        try:
            opcua_client.connect()
            isConnected = True
        except:
            sleep(1)
    root = opcua_client.get_root_node()
    obj = root.get_child(["0:Objects", "2:MyObject"])

    lbr_event = root.get_child(["0:Types", "0:EventTypes", "0:BaseEventType", "2:LBREvent"])
    kmp_event = root.get_child(["0:Types", "0:EventTypes", "0:BaseEventType", "2:KMPEvent"])
    camera_event = root.get_child(["0:Types", "0:EventTypes", "0:BaseEventType", "2:CameraEvent"])

    lbr_publisher = LBRPubSub(obj)
    kmp_publisher = KMPPubSub(obj)
    camera_publisher = CameraPubSub(obj)

    lbr_sub = opcua_client.create_subscription(100, lbr_publisher)
    lbr_handle = lbr_sub.subscribe_events(obj, lbr_event)

    kmp_sub = opcua_client.create_subscription(100, kmp_publisher)
    kmp_handle = kmp_sub.subscribe_events(obj, kmp_event)

    camera_sub = opcua_client.create_subscription(100, camera_publisher)
    camera_handle = camera_sub.subscribe_events(obj, camera_event)
    """ OPC UA CLIENT END """


    try:
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(lbr_publisher)
        executor.add_node(kmp_publisher)
        executor.add_node(camera_publisher)
        executor.spin()
    finally:
        executor.shutdown()
        lbr_publisher.destroy_node()
        kmp_publisher.destroy_node()
        camera_publisher.destroy_node()
        
        lbr_sub.unsubscribe(lbr_handle)
        lbr_sub.delete()
        kmp_sub.unsubscribe(kmp_handle)
        kmp_sub.delete()
        camera_sub.unsubscribe(camera_handle)
        camera_sub.delete()

    rclpy.shutdown()


if __name__ == '__main__':
    main()