import random

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState


class BatterySim(Node):
    def __init__(self):
        super().__init__('battery_reader')

        self.declare_parameter('sample_rate', 1.0)
        self.sample_rate = float(self.get_parameter('sample_rate').value)

        self.publisher_ = self.create_publisher(BatteryState, '/battery/state', 10)
        self.timer = self.create_timer(1.0 / self.sample_rate, self.publish_battery)

        self.percentage = 100.0
        self.voltage_max = 15.0
        self.voltage_min = 12.0

        self.publish_ok_reported = False
        self.publish_error_reported = False

        self.get_logger().info(
            f"Battery Reader started | topic=/battery/state | sample_rate={self.sample_rate} Hz"
        )

    def publish_battery(self):
        try:
            msg = BatteryState()
            msg.header.stamp = self.get_clock().now().to_msg()

            drain = random.uniform(0.05, 0.2)
            self.percentage = max(0.0, self.percentage - drain)

            voltage = self.voltage_min + (
                (self.percentage / 100.0) * (self.voltage_max - self.voltage_min)
            )
            current = random.uniform(-3.0, -0.5)
            voltage += random.uniform(-0.05, 0.05)

            msg.voltage = float(voltage)
            msg.current = float(current)
            msg.percentage = float(self.percentage / 100.0)

            self.publisher_.publish(msg)

            if not self.publish_ok_reported:
                self.get_logger().info("Battery dummy publish active")
                self.publish_ok_reported = True
                self.publish_error_reported = False

        except Exception as e:
            if not self.publish_error_reported:
                self.get_logger().error(f"Battery publish failed: {e}")
                self.publish_error_reported = True
                self.publish_ok_reported = False


def main():
    rclpy.init()
    node = BatterySim()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()