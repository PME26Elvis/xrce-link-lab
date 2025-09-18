import rclpy, time, statistics
from rclpy.node import Node
from std_msgs.msg import Int32

class Jitter(Node):
    def __init__(self):
        super().__init__('heartbeat_jitter')
        self.sub = self.create_subscription(Int32, 'mcu/heartbeat', self.cb, 10)
        self.last = None
        self.intervals = []

    def cb(self, _msg):
        now = time.perf_counter()
        if self.last is not None:
            dt = (now - self.last) * 1000.0  # ms
            self.intervals.append(dt)
            if len(self.intervals) >= 50:
                s = sorted(self.intervals)
                p90 = s[int(0.9*len(s))-1]
                avg = statistics.mean(self.intervals)
                std = statistics.pstdev(self.intervals)
                self.get_logger().info(
                    f'inter-arrival avg={avg:.2f} ms, std={std:.2f} ms, p90={p90:.2f} ms (n={len(self.intervals)})'
                )
                self.intervals.clear()
        self.last = now

def main():
    rclpy.init(); n = Jitter(); rclpy.spin(n); n.destroy_node(); rclpy.shutdown()

