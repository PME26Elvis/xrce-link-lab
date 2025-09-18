import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

class Relay(Node):
    def __init__(self):
        super().__init__('xrce_link_relay')
        # micro-ROS demo 的 topic 名稱
        self.sub = self.create_subscription(Int32, 'std_msgs_msg_Int32', self.cb, 10)
        self.pub = self.create_publisher(Int32, 'mcu/heartbeat', 10)

    def cb(self, msg):
        self.pub.publish(msg)

def main():
    rclpy.init()
    n = Relay()
    rclpy.spin(n)
    n.destroy_node()
    rclpy.shutdown()
