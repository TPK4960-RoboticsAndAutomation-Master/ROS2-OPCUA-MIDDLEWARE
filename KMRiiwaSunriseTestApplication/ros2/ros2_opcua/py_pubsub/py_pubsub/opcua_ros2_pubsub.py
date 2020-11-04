# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node

from std_msgs.msg import String

from opcua import ua, Client

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'manipulator_vel', 10)

    def publish_msg(self, msg):
        self.publisher_.publish(msg)

    def event_notification(self, event):
        print("New event recived: ", event.Message.Text)

        msg = String()
        msg.data = event.Message.Text
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()
    
    """ OPC UA CLIENT """
    opcua_client = Client("opc.tcp://0.0.0.0:4840/freeopcua/server/")
    opcua_client.connect()
    root = opcua_client.get_root_node()
    obj = root.get_child(["0:Objects", "2:MyObject"])
    myevent = root.get_child(["0:Types", "0:EventTypes", "0:BaseEventType", "2:MyFirstEvent"])
    sub = opcua_client.create_subscription(100, minimal_publisher)
    handle = sub.subscribe_events(obj, myevent)
    """ OPC UA CLIENT END """

    rclpy.spin(minimal_publisher)

    minimal_publisher.destroy_node()
    sub.unsubscribe(handle)
    sub.delete()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
