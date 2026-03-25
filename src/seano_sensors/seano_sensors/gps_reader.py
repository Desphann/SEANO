import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from datetime import datetime, timezone


class GPSReader(Node):
    def __init__(self):
        super().__init__('gps_reader')

        self.declare_parameter('sample_rate', 1.0)
        self.sample_rate = float(self.get_parameter('sample_rate').value)
        self.min_period = 1.0 / self.sample_rate
        self.last_time = None

        self.create_subscription(
            NavSatFix,
            '/mavros/global_position/raw/fix',
            self.gps_callback,
            10
        )

        self.get_logger().info(f"GPS Reader Started | sample_rate={self.sample_rate} Hz")

    def convert_time(self, stamp):
        sec = stamp.sec
        nanosec = stamp.nanosec
        dt = datetime.fromtimestamp(sec, tz=timezone.utc)
        return dt.strftime("%Y-%m-%d %H:%M:%S") + f".{int(nanosec/1e6):03d}"

    def gps_callback(self, msg):
        now = self.get_clock().now().nanoseconds / 1e9
        if self.last_time is not None and (now - self.last_time) < self.min_period:
            return
        self.last_time = now

        timestamp = self.convert_time(msg.header.stamp)
        self.get_logger().info(
            f"{timestamp} | Lat: {msg.latitude:.6f} | "
            f"Lon: {msg.longitude:.6f} | Alt: {msg.altitude:.2f}"
        )

def main():
    rclpy.init()
    node = GPSReader()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()