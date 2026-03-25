import time
from datetime import datetime, timezone

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix


class GPSReader(Node):
    def __init__(self):
        super().__init__('gps_reader')

        self.declare_parameter('sample_rate', 1.0)
        self.sample_rate = float(self.get_parameter('sample_rate').value)
        self.min_period = 1.0 / self.sample_rate

        self.timeout_sec = 5.0
        self.waiting_reminder_sec = 15.0

        self.last_process_time = None
        self.last_msg_time = None
        self.last_waiting_log_time = 0.0

        self.has_ever_received_data = False
        self.is_connected = False
        self.timeout_reported = False

        self.create_subscription(
            NavSatFix,
            '/mavros/global_position/raw/fix',
            self.gps_callback,
            10
        )

        self.create_timer(1.0, self.check_status)

        self.get_logger().info(
            f"GPS Reader started | topic=/mavros/global_position/raw/fix | sample_rate={self.sample_rate} Hz"
        )
        self.get_logger().info("GPS status: waiting for first data...")

    def convert_time(self, stamp):
        sec = stamp.sec
        nanosec = stamp.nanosec
        dt = datetime.fromtimestamp(sec, tz=timezone.utc)
        return dt.strftime("%Y-%m-%d %H:%M:%S") + f".{int(nanosec / 1e6):03d}"

    def gps_callback(self, msg):
        now = time.time()
        self.last_msg_time = now

        if self.last_process_time is not None and (now - self.last_process_time) < self.min_period:
            return

        self.last_process_time = now

        if not self.has_ever_received_data:
            timestamp = self.convert_time(msg.header.stamp)
            self.get_logger().info(f"GPS connected | first data received at {timestamp}")
            self.has_ever_received_data = True
            self.is_connected = True
            self.timeout_reported = False
            return

        if self.timeout_reported:
            timestamp = self.convert_time(msg.header.stamp)
            self.get_logger().info(f"GPS reconnected | data received again at {timestamp}")
            self.is_connected = True
            self.timeout_reported = False

    def check_status(self):
        now = time.time()

        if self.last_msg_time is None:
            if (now - self.last_waiting_log_time) >= self.waiting_reminder_sec:
                self.get_logger().warning("GPS still waiting for data...")
                self.last_waiting_log_time = now
            return

        if (now - self.last_msg_time) > self.timeout_sec:
            if not self.timeout_reported:
                self.get_logger().error("GPS timeout | no data received in the last 5 seconds")
                self.timeout_reported = True
                self.is_connected = False


def main():
    rclpy.init()
    node = GPSReader()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()