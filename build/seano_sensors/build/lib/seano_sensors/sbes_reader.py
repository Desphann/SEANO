import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import math
import random
import time


class SBESSim(Node):
    def __init__(self):
        super().__init__('sbes_reader')

        self.declare_parameter('sample_rate', 1.0)
        self.sample_rate = float(self.get_parameter('sample_rate').value)

        self.publisher_ = self.create_publisher(Float64, '/sbes/depth', 10)
        self.timer = self.create_timer(1.0 / self.sample_rate, self.publish_sbes)

        self.start_time = time.time()
        self.base_depth = 12.0
        self.depth_amp = 1.5

        self.get_logger().info(
            f"SBES Dummy Started | topic=/sbes/depth | sample_rate={self.sample_rate} Hz"
        )

    def publish_sbes(self):
        t = time.time() - self.start_time

        depth = self.base_depth + self.depth_amp * math.sin(t * 0.1)
        depth += random.uniform(-0.05, 0.05)

        msg = Float64()
        msg.data = float(depth)

        self.publisher_.publish(msg)
        self.get_logger().info(f"Depth: {depth:.3f} m")


def main():
    rclpy.init()
    node = SBESSim()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()