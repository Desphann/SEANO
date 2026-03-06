import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, Imu
from std_msgs.msg import Float64MultiArray
from rclpy.qos import qos_profile_sensor_data

import os
from datetime import datetime
import time


class SeanoLogger(Node):

    def __init__(self):
        super().__init__('logger_node')

        self.mount_point = "/media/raihan/FATA"
        self.flush_interval = 3.0

        # ===== Waktu Lokal Sistem =====
        self.start_time_obj = datetime.now()
        self.local_timezone = time.tzname[0]

        year = self.start_time_obj.strftime("%Y")
        month = self.start_time_obj.strftime("%m")
        day = self.start_time_obj.strftime("%d")

        # Folder terakhir hanya JAM mulai misi
        self.mission_id = self.start_time_obj.strftime(
            f"MISSION_START_%H-%M-%S_{self.local_timezone}"
        )

        self.base_path = os.path.join(
            self.mount_point,
            "SEANO_MISSIONS",
            year,
            month,
            day,
            self.mission_id
        )

        os.makedirs(self.base_path, exist_ok=True)

        # Metadata mission
        with open(os.path.join(self.base_path, "mission_info.txt"), "w") as f:
            f.write(f"Start Time: {self.start_time_obj}\n")
            f.write(f"Timezone: {self.local_timezone}\n")
            f.write("Platform: SEANO USV\n")

        self.files = {}
        self.csv_files = {}
        self.buffers = {}
        self.sample_count = {}
        self.detected_sensors = set()

        self.create_timer(2.0, self.detect_and_initialize_sensors)
        self.create_timer(self.flush_interval, self.flush_buffers)

        self.get_logger().info(f"📁 Mission folder: {self.base_path}")
        self.get_logger().info(f"🕒 Timezone: {self.local_timezone}")
        self.get_logger().info("✅ SEANO Logger Started")

    # ================= LOCAL TIMESTAMP =================

    def get_local_timestamp(self):
        now = datetime.now()
        return now.strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]

    # ================= SENSOR DETECTION =================

    def detect_and_initialize_sensors(self):

        topics = dict(self.get_topic_names_and_types())

        # ---------- GPS (MAVROS) ----------
        if '/mavros/global_position/raw/fix' in topics and 'gps' not in self.detected_sensors:
            self.init_sensor(
                "gps",
                "GPS",
                f"Timestamp({self.local_timezone})\tLatitude\tLongitude\tAltitude",
                "timestamp,latitude,longitude,altitude"
            )
            self.create_subscription(
                NavSatFix,
                '/mavros/global_position/raw/fix',
                self.gps_callback,
                qos_profile_sensor_data
            )
            self.detected_sensors.add('gps')

        # ---------- IMU (MAVROS) ----------
        if '/mavros/imu/data' in topics and 'imu' not in self.detected_sensors:
            self.init_sensor(
                "imu",
                "IMU",
                f"Timestamp({self.local_timezone})\tAccX\tAccY\tAccZ",
                "timestamp,acc_x,acc_y,acc_z"
            )
            self.create_subscription(
                Imu,
                '/mavros/imu/data',
                self.imu_callback,
                qos_profile_sensor_data
            )
            self.detected_sensors.add('imu')

        # ---------- CTD ----------
        if '/ctd/data' in topics and 'ctd' not in self.detected_sensors:
            self.init_sensor(
                "ctd",
                "CTD",
                f"Timestamp({self.local_timezone})\tDepth\tTemp\tCond\tSalinity\tDensity\tSoundVel",
                "timestamp,depth,temp,cond,salinity,density,soundvel"
            )
            self.create_subscription(
                Float64MultiArray,
                '/ctd/data',
                self.ctd_callback,
                50
            )
            self.detected_sensors.add('ctd')

        # ---------- ADCP ----------
        if '/adcp/data' in topics and 'adcp' not in self.detected_sensors:
            self.init_sensor(
                "adcp",
                "ADCP",
                f"Timestamp({self.local_timezone})\tCellCount\tBeamCount\tVelocityData",
                "timestamp,data"
            )
            self.create_subscription(
                Float64MultiArray,
                '/adcp/data',
                self.adcp_callback,
                10
            )
            self.detected_sensors.add('adcp')

    # ================= INIT FILE =================

    def init_sensor(self, key, name, log_columns, csv_columns):

        log_path = os.path.join(self.base_path, f"{key}.log")
        csv_path = os.path.join(self.base_path, f"{key}.csv")

        log_file = open(log_path, "w")
        csv_file = open(csv_path, "w")

        log_file.write(
            "[System]\n"
            "Platform=SEANO USV\n"
            f"Start_Time={self.start_time_obj.strftime('%Y-%m-%d %H:%M:%S')}\n"
            f"Timezone={self.local_timezone}\n\n"
            "[Sensor]\n"
            f"Name={name}\n\n"
            "[Columns]\n"
            f"{log_columns}\n\n"
            "[Data]\n"
        )

        csv_file.write(csv_columns + "\n")

        self.files[key] = log_file
        self.csv_files[key] = csv_file
        self.buffers[key] = []
        self.sample_count[key] = 0

        self.get_logger().info(f"🛰 {name} detected")

    # ================= CALLBACKS =================

    def gps_callback(self, msg):
        t = self.get_local_timestamp()
        log_line = f"{t}\t{msg.latitude}\t{msg.longitude}\t{msg.altitude}\n"
        csv_line = f"{t},{msg.latitude},{msg.longitude},{msg.altitude}\n"
        self.buffers['gps'].append((log_line, csv_line))
        self.sample_count['gps'] += 1

    def imu_callback(self, msg):
        t = self.get_local_timestamp()
        log_line = f"{t}\t{msg.linear_acceleration.x}\t{msg.linear_acceleration.y}\t{msg.linear_acceleration.z}\n"
        csv_line = f"{t},{msg.linear_acceleration.x},{msg.linear_acceleration.y},{msg.linear_acceleration.z}\n"
        self.buffers['imu'].append((log_line, csv_line))
        self.sample_count['imu'] += 1

    def ctd_callback(self, msg):
        t = self.get_local_timestamp()
        data_str = "\t".join(map(str, msg.data))
        csv_str = ",".join(map(str, msg.data))
        log_line = f"{t}\t{data_str}\n"
        csv_line = f"{t},{csv_str}\n"
        self.buffers['ctd'].append((log_line, csv_line))
        self.sample_count['ctd'] += 1

    def adcp_callback(self, msg):
        t = self.get_local_timestamp()
        data_str = "\t".join(map(str, msg.data))
        csv_str = ",".join(map(str, msg.data))
        log_line = f"{t}\t{data_str}\n"
        csv_line = f"{t},{csv_str}\n"
        self.buffers['adcp'].append((log_line, csv_line))
        self.sample_count['adcp'] += 1

    # ================= FLUSH =================

    def flush_buffers(self):

        for key in self.buffers:
            if self.buffers[key]:
                for log_line, csv_line in self.buffers[key]:
                    self.files[key].write(log_line)
                    self.csv_files[key].write(csv_line)

                self.files[key].flush()
                self.csv_files[key].flush()
                self.buffers[key].clear()

        self.get_logger().info("💾 Buffers flushed")

    # ================= SHUTDOWN =================

    def destroy_node(self):

        self.flush_buffers()

        for key in self.files:
            self.files[key].close()
            self.csv_files[key].close()

        os.sync()
        super().destroy_node()


def main():
    rclpy.init()
    node = SeanoLogger()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()