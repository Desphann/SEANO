import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
import random

class BatterySim(Node):
    def __init__(self):
        super().__init__('battery_reader')

        self.declare_parameter('sample_rate', 1.0)
        self.sample_rate = float(self.get_parameter('sample_rate').value)

        self.publisher_ = self.create_publisher(BatteryState, '/battery/state', 10)
        self.timer = self.create_timer(1.0 / self.sample_rate, self.publish_battery)

        # kondisi awal
        self.percentage = 100.0
        self.voltage_max = 15.0
        self.voltage_min = 12.0

        self.get_logger().info(f"Battery Sim Started | sample_rate={self.sample_rate} Hz")

    def publish_battery(self):
        msg = BatteryState()
        msg.header.stamp = self.get_clock().now().to_msg()

        # ====== SIMULASI BATTERY ======
        # battery turun pelan
        drain = random.uniform(0.05, 0.2)  # % per cycle
        self.percentage = max(0.0, self.percentage - drain)

        # voltage linear terhadap percentage
        voltage = self.voltage_min + (
            (self.percentage / 100.0) * (self.voltage_max - self.voltage_min)
        )

        # current: mostly discharge (negatif)
        current = random.uniform(-3.0, -0.5)

        # sedikit noise biar realistis
        voltage += random.uniform(-0.05, 0.05)

        # ====== ASSIGN ======
        msg.voltage = float(voltage)
        msg.current = float(current)
        msg.percentage = float(self.percentage / 100.0)  # ROS pakai 0.0–1.0

        self.publisher_.publish(msg)

        self.get_logger().info(
            f"V={msg.voltage:.2f}V | I={msg.current:.2f}A | SoC={self.percentage:.1f}%"
        )

def main():
    rclpy.init()
    node = BatterySim()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()