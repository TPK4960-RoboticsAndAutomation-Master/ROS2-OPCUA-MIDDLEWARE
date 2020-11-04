import rclpy
from rclpy.node import Node
from time import sleep
from std_msgs.msg import String

from opcua import ua, Client

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('hybrid_node')
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
    isConnected = False
    opcua_client = Client("opc.tcp://0.0.0.0:4840/freeopcua/server/")
    while not isConnected:
        try:
            opcua_client.connect()
            isConnected = True
        except:
            sleep(1)
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