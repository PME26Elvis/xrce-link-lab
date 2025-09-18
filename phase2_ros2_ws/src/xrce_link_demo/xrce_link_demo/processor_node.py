import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from statistics import mean

class Processor(Node):
    def __init__(self):
        super().__init__('xrce_link_processor')
        self.sub = self.create_subscription(Int32, 'mcu/heartbeat', self.cb, 10)
        self.pub = self.create_publisher(Int32, 'mcu/heartbeat_avg5', 10)
        self.buf = []

    def cb(self, msg):
        self.buf.append(msg.data)
        if len(self.buf) > 5:
            self.buf.pop(0)
        out = Int32()
        out.data = int(mean(self.buf))
        self.pub.publish(out)

def main():
    rclpy.init()
    n = Processor()
    rclpy.spin(n)
    n.destroy_node()
    rclpy.shutdown()
