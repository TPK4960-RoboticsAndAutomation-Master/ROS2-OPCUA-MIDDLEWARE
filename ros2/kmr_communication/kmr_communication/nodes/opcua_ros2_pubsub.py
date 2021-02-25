#!/usr/bin/env python3

import rclpy
from time import sleep
from std_msgs.msg import String
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, TransformStamped
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from opcua import ua, Client


class LBRPubSub(Node):
    def __init__(self, ua_obj):
        super().__init__('lbr_hybrid_node')
        self.publisher_ = self.create_publisher(String, 'manipulator_vel', 10)
        self.shutdown_publisher_ = self.create_publisher(String, 'shutdown', 10)
        self.status_subscriber = self.create_subscription(String, 'lbr_status', self.status_callback, 10)
        self.server_obj = ua_obj


    def event_notification(self, event):
        msg = String()
        msg.data = event.Message.Text
        if msg.data == "shutdown":
            self.shutdown_publisher_.publish(msg)
        else:
            self.publisher_.publish(msg)

    def status_callback(self, msg):
        print("status update callback from lbr")
        method = "update_status"
        self.server_obj.call_method("2:" + method, str(msg.data))
        

class KMPPubSub(Node):
    def __init__(self, ua_obj):
        super().__init__('kmp_hybrid_node')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.status_subscriber = self.create_subscription(String, 'kmp_status', self.status_callback, 10)
        self.server_obj = ua_obj

    def event_notification(self, event):
        """
            event.Message,text = "speed x y th", e.g. "0.5 0 1 0"
        """
        e = event.Message.Text.split(" ")
        speed = float(e[0])
        turn = 1.0
        twist = Twist()
        twist.linear.x = float(e[1])*speed
        twist.linear.y = float(e[2])*speed
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = float(e[3])*speed #or turn
        self.publisher_.publish(twist)

    def status_callback(self, msg):
        print("status update callback from kmp")
        method = "update_status"
        self.server_obj.call_method("2:" + method, str(msg.data))


def main(args=None):
    rclpy.init(args=args)
    
    """ 
        OPC UA CLIENT 
        For receiving commands from AAS
    """
    isConnected = False
    #opcua_client = Client("opc.tcp://10.22.25.161:4840/freeopcua/server/")
    opcua_client = Client("opc.tcp://andrcar-master.ivt.ntnu.no:4841/freeopcua/server/")

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
    
    lbr_publisher = LBRPubSub(obj)
    kmp_publisher = KMPPubSub(obj)

    lbr_sub = opcua_client.create_subscription(100, lbr_publisher)
    lbr_handle = lbr_sub.subscribe_events(obj, lbr_event)

    kmp_sub = opcua_client.create_subscription(100, kmp_publisher)
    kmp_handle = kmp_sub.subscribe_events(obj, kmp_event)
    """ OPC UA CLIENT END """


    try:
        executor = MultiThreadedExecutor(num_threads=2)
        executor.add_node(lbr_publisher)
        executor.add_node(kmp_publisher)
        executor.spin()
    finally:
        executor.shutdown()
        lbr_publisher.destroy_node()
        kmp_publisher.destroy_node()
        
        lbr_sub.unsubscribe(lbr_handle)
        lbr_sub.delete()

        kmp_sub.unsubscribe(kmp_handle)
        kmp_sub.delete()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
