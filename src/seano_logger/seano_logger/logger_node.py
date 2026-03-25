import os
import time
from datetime import datetime

import psutil
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix, Imu, BatteryState
from std_msgs.msg import Float64MultiArray


class SeanoLogger(Node):
    def __init__(self):
        super().__init__('logger_node')

        # Konfigurasi penyimpanan
        self.external_mount_point = "/media/raihan/SEANO"
        self.local_mount_point = os.path.expanduser("~/Documents/SEANO_logs")

        self.enable_external_logging = True
        self.enable_local_logging = True

        # Interval logging metrics sistem
        self.metrics_interval = 1.0

        # State logger
        self.external_ready = False
        self.external_failed_runtime = False
        self.external_fail_reported = False
        self.sensor_log_status = {}

        # Ambil waktu mulai misi dan timezone lokal
        self.start_time_obj = datetime.now()
        self.local_timezone = time.tzname[0]

        # Struktur folder berdasarkan tanggal
        year = self.start_time_obj.strftime("%Y")
        month = self.start_time_obj.strftime("%m")
        day = self.start_time_obj.strftime("%d")

        # Nama folder misi
        self.mission_id = self.start_time_obj.strftime(
            f"MISSION_START_%H-%M-%S_{self.local_timezone}"
        )

        # Semua output path aktif
        self.base_paths = []

        # =========================================================
        # PRIORITAS: external SSD harus siap dulu
        # Kalau external tidak siap, logger tidak dimulai
        # =========================================================
        if self.enable_external_logging:
            if not self.is_path_writable(self.external_mount_point):
                self.get_logger().fatal(
                    f"SSD external belum terdeteksi / tidak writable: {self.external_mount_point}"
                )
                self.get_logger().fatal(
                    "Logger tidak akan dimulai. Pastikan SSD sudah terpasang terlebih dahulu."
                )
                raise RuntimeError("External SSD not ready")

            external_base_path = os.path.join(
                self.external_mount_point,
                "SEANO_MISSIONS",
                year,
                month,
                day,
                self.mission_id
            )

            try:
                os.makedirs(external_base_path, exist_ok=True)

                if not self.test_write_access(external_base_path):
                    self.get_logger().fatal(
                        f"SSD external terdeteksi tapi gagal ditulisi: {external_base_path}"
                    )
                    raise RuntimeError("External SSD not writable")

                self.base_paths.append(external_base_path)
                self.external_ready = True
                self.get_logger().info(f"External logging ready: {external_base_path}")

            except Exception as e:
                self.get_logger().fatal(
                    f"Gagal menyiapkan external logging: {external_base_path} | {e}"
                )
                raise RuntimeError("Failed to initialize external logging")

        # Local tetap dibuat juga, tapi external tetap prioritas utama
        if self.enable_local_logging:
            local_base_path = os.path.join(
                self.local_mount_point,
                year,
                month,
                day,
                self.mission_id
            )
            try:
                os.makedirs(local_base_path, exist_ok=True)
                self.base_paths.append(local_base_path)
                self.get_logger().info(f"Local logging ready: {local_base_path}")
            except Exception as e:
                self.get_logger().warning(
                    f"Gagal membuat folder local logging: {local_base_path} | {e}"
                )

        if not self.base_paths:
            raise RuntimeError("Tidak ada path logging yang valid.")

        # Tulis mission info di semua lokasi
        for base_path in self.base_paths:
            with open(os.path.join(base_path, "mission_info.txt"), "w") as f:
                f.write(f"Start Time: {self.start_time_obj}\n")
                f.write(f"Timezone: {self.local_timezone}\n")
                f.write("Platform: SEANO USV\n")

        # Struktur logger
        self.files = {}
        self.csv_files = {}
        self.sample_count = {}
        self.detected_sensors = set()
        self.subscriptions_map = {}

        # Metrics sistem
        self.metrics_log_files = []
        self.metrics_csv_files = []
        self.last_sample_count = {}
        self.last_metrics_time = time.time()
        self.bytes_written_since_last_metrics = 0

        self.init_metrics_logger()

        # Prime cpu_percent
        psutil.cpu_percent(interval=None)

        # Timer
        self.create_timer(2.0, self.detect_and_initialize_sensors)
        self.create_timer(self.metrics_interval, self.log_periodic_metrics)
        self.create_timer(1.0, self.monitor_external_storage)

        for path in self.base_paths:
            self.get_logger().info(f"Mission folder: {path}")
        self.get_logger().info(f"Timezone: {self.local_timezone}")
        self.get_logger().info("SEANO Logger Started")

    # =========================================================
    # Utility
    # =========================================================
    def get_local_timestamp(self):
        now = datetime.now()
        return now.strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]

    def is_path_writable(self, path):
        return os.path.exists(path) and os.access(path, os.W_OK)

    def test_write_access(self, path):
        test_file = os.path.join(path, ".seano_write_test")
        try:
            with open(test_file, "w") as f:
                f.write("ok")
            os.remove(test_file)
            return True
        except Exception:
            return False

    def monitor_external_storage(self):
        if not self.enable_external_logging or not self.external_ready:
            return

        if not self.is_path_writable(self.external_mount_point):
            if not self.external_fail_reported:
                self.get_logger().fatal(
                    f"SSD external terputus / tidak writable lagi: {self.external_mount_point}"
                )
                self.get_logger().fatal(
                    "Logging ke SSD bermasalah. Local mungkin masih aktif, tapi external gagal."
                )
                self.external_fail_reported = True
                self.external_failed_runtime = True
            return

    # =========================================================
    # Metrics logger
    # =========================================================
    def init_metrics_logger(self):
        for base_path in self.base_paths:
            metrics_log_path = os.path.join(base_path, "system_metrics.log")
            metrics_csv_path = os.path.join(base_path, "system_metrics.csv")

            metrics_log_file = open(metrics_log_path, "w")
            metrics_csv_file = open(metrics_csv_path, "w")

            metrics_log_file.write(
                "[System]\n"
                "Platform=SEANO USV\n"
                f"Start_Time={self.start_time_obj.strftime('%Y-%m-%d %H:%M:%S')}\n"
                f"Timezone={self.local_timezone}\n\n"
                "[Sensor]\n"
                "Name=System Metrics\n\n"
                "[Columns]\n"
                f"Timestamp({self.local_timezone})\tWriteSpeed(Bps)\tCPU(%)\tRAM(%)\tGPS_Hz\tIMU_Hz\tCTD_Hz\tADCP_Hz\tBATTERY_Hz\tSBES_Hz\n\n"
                "[Data]\n"
            )

            metrics_csv_file.write(
                "timestamp,write_speed_Bps,cpu_percent,ram_percent,gps_hz,imu_hz,ctd_hz,adcp_hz,battery_hz,sbes_hz\n"
            )

            self.metrics_log_files.append(metrics_log_file)
            self.metrics_csv_files.append(metrics_csv_file)

    # =========================================================
    # Sensor detection
    # =========================================================
    def detect_and_initialize_sensors(self):
        topics = dict(self.get_topic_names_and_types())

        if '/mavros/global_position/raw/fix' in topics and 'gps' not in self.detected_sensors:
            self.init_sensor(
                "gps",
                "GPS",
                f"Timestamp({self.local_timezone})\tLatitude\tLongitude\tAltitude",
                "timestamp,latitude,longitude,altitude"
            )
            self.subscriptions_map['gps'] = self.create_subscription(
                NavSatFix,
                '/mavros/global_position/raw/fix',
                self.gps_callback,
                qos_profile_sensor_data
            )
            self.detected_sensors.add('gps')
            self.get_logger().info("GPS detected")

        if '/mavros/imu/data' in topics and 'imu' not in self.detected_sensors:
            self.init_sensor(
                "imu",
                "IMU",
                f"Timestamp({self.local_timezone})\tAccX\tAccY\tAccZ",
                "timestamp,acc_x,acc_y,acc_z"
            )
            self.subscriptions_map['imu'] = self.create_subscription(
                Imu,
                '/mavros/imu/data',
                self.imu_callback,
                qos_profile_sensor_data
            )
            self.detected_sensors.add('imu')
            self.get_logger().info("IMU detected")

        if '/ctd/data' in topics and 'ctd' not in self.detected_sensors:
            self.init_sensor(
                "ctd",
                "CTD",
                f"Timestamp({self.local_timezone})\tDepth\tTemp\tCond\tSalinity\tDensity\tSoundVel",
                "timestamp,depth,temp,cond,salinity,density,soundvel"
            )
            self.subscriptions_map['ctd'] = self.create_subscription(
                Float64MultiArray,
                '/ctd/data',
                self.ctd_callback,
                50
            )
            self.detected_sensors.add('ctd')
            self.get_logger().info("CTD detected")

        if '/adcp/data' in topics and 'adcp' not in self.detected_sensors:
            self.init_sensor(
                "adcp",
                "ADCP",
                (
                    f"Timestamp({self.local_timezone})\tNumCells\tNumBeams\tCellSizeM\t"
                    "BlankingDistanceM\tHeadingDeg\tPitchDeg\tRollDeg\t"
                    "TemperatureC\tSalinityPSU\tPressureDbar\tVelocityProfile"
                ),
                (
                    "timestamp,num_cells,num_beams,cell_size_m,blanking_distance_m,"
                    "heading_deg,pitch_deg,roll_deg,temperature_c,salinity_psu,pressure_dbar,velocity_profile"
                )
            )
            self.subscriptions_map['adcp'] = self.create_subscription(
                Float64MultiArray,
                '/adcp/data',
                self.adcp_callback,
                10
            )
            self.detected_sensors.add('adcp')
            self.get_logger().info("ADCP detected")

        if '/battery/state' in topics and 'battery' not in self.detected_sensors:
            self.init_sensor(
                "battery",
                "Battery",
                f"Timestamp({self.local_timezone})\tVoltage\tCurrent\tPercentage",
                "timestamp,voltage,current,percentage"
            )
            self.subscriptions_map['battery'] = self.create_subscription(
                BatteryState,
                '/battery/state',
                self.battery_callback,
                10
            )
            self.detected_sensors.add('battery')
            self.get_logger().info("Battery detected")

        if '/sbes/data' in topics and 'sbes' not in self.detected_sensors:
            self.init_sensor(
                "sbes",
                "SBES",
                f"Timestamp({self.local_timezone})\tDepth\tWaterTemp\tQualityFlag",
                "timestamp,depth,water_temp,quality_flag"
            )
            self.subscriptions_map['sbes'] = self.create_subscription(
                Float64MultiArray,
                '/sbes/data',
                self.sbes_callback,
                10
            )
            self.detected_sensors.add('sbes')
            self.get_logger().info("SBES detected")

    def init_sensor(self, key, name, log_columns, csv_columns):
        self.files[key] = []
        self.csv_files[key] = []
        self.sample_count[key] = 0
        self.last_sample_count[key] = 0
        self.sensor_log_status[key] = False

        for base_path in self.base_paths:
            log_path = os.path.join(base_path, f"{key}.log")
            csv_path = os.path.join(base_path, f"{key}.csv")

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

            self.files[key].append(log_file)
            self.csv_files[key].append(csv_file)

    # =========================================================
    # Safe write
    # =========================================================
    def write_sensor_data(self, key, log_line, csv_line):
        if key not in self.files:
            return

        success_count = 0
        total_targets = len(self.files[key])

        for idx, log_file in enumerate(self.files[key]):
            try:
                log_file.write(log_line)
                log_file.flush()
                self.csv_files[key][idx].write(csv_line)
                self.csv_files[key][idx].flush()
                success_count += 1
            except Exception as e:
                self.get_logger().error(f"{key.upper()} logging failed on target {idx}: {e}")

        if success_count > 0:
            self.bytes_written_since_last_metrics += len(log_line.encode('utf-8'))
            self.bytes_written_since_last_metrics += len(csv_line.encode('utf-8'))
            self.sample_count[key] += 1

            if not self.sensor_log_status.get(key, False):
                self.get_logger().info(f"{key.upper()} logging active")
                self.sensor_log_status[key] = True
        else:
            if self.sensor_log_status.get(key, False):
                self.get_logger().error(f"{key.upper()} logging failed on all targets")
                self.sensor_log_status[key] = False

        # Tegaskan kondisi external jika target external gagal runtime
        if self.enable_external_logging and self.external_ready and total_targets > 0:
            external_ok = False
            external_path_prefix = os.path.join(self.external_mount_point, "SEANO_MISSIONS")

            for idx, log_file in enumerate(self.files[key]):
                file_name = getattr(log_file, "name", "")
                if file_name.startswith(external_path_prefix):
                    try:
                        # kalau penulisan sebelumnya sukses, file handle masih valid
                        external_ok = True
                    except Exception:
                        external_ok = False

            if not self.is_path_writable(self.external_mount_point):
                if not self.external_fail_reported:
                    self.get_logger().fatal(
                        f"SSD external terputus / tidak writable lagi: {self.external_mount_point}"
                    )
                    self.external_fail_reported = True
                    self.external_failed_runtime = True

    # =========================================================
    # Sensor callbacks
    # =========================================================
    def gps_callback(self, msg):
        t = self.get_local_timestamp()
        log_line = f"{t}\t{msg.latitude}\t{msg.longitude}\t{msg.altitude}\n"
        csv_line = f"{t},{msg.latitude},{msg.longitude},{msg.altitude}\n"
        self.write_sensor_data('gps', log_line, csv_line)

    def imu_callback(self, msg):
        t = self.get_local_timestamp()
        log_line = (
            f"{t}\t"
            f"{msg.linear_acceleration.x}\t"
            f"{msg.linear_acceleration.y}\t"
            f"{msg.linear_acceleration.z}\n"
        )
        csv_line = (
            f"{t},"
            f"{msg.linear_acceleration.x},"
            f"{msg.linear_acceleration.y},"
            f"{msg.linear_acceleration.z}\n"
        )
        self.write_sensor_data('imu', log_line, csv_line)

    def ctd_callback(self, msg):
        t = self.get_local_timestamp()

        if len(msg.data) >= 6:
            depth, temp, cond, salinity, density, soundvel = msg.data[:6]
            log_line = f"{t}\t{depth}\t{temp}\t{cond}\t{salinity}\t{density}\t{soundvel}\n"
            csv_line = f"{t},{depth},{temp},{cond},{salinity},{density},{soundvel}\n"
        else:
            data_str = "\t".join(map(str, msg.data))
            csv_str = ",".join(map(str, msg.data))
            log_line = f"{t}\t{data_str}\n"
            csv_line = f"{t},{csv_str}\n"

        self.write_sensor_data('ctd', log_line, csv_line)

    def adcp_callback(self, msg):
        t = self.get_local_timestamp()
        data = list(msg.data)

        if len(data) >= 10:
            num_cells = int(data[0])
            num_beams = int(data[1])
            cell_size_m = data[2]
            blanking_distance_m = data[3]
            heading_deg = data[4]
            pitch_deg = data[5]
            roll_deg = data[6]
            temperature_c = data[7]
            salinity_psu = data[8]
            pressure_dbar = data[9]
            velocity_profile = data[10:]

            velocity_profile_str = ";".join(map(str, velocity_profile))

            log_line = (
                f"{t}\t{num_cells}\t{num_beams}\t{cell_size_m}\t{blanking_distance_m}\t"
                f"{heading_deg}\t{pitch_deg}\t{roll_deg}\t"
                f"{temperature_c}\t{salinity_psu}\t{pressure_dbar}\t{velocity_profile_str}\n"
            )
            csv_line = (
                f"{t},{num_cells},{num_beams},{cell_size_m},{blanking_distance_m},"
                f"{heading_deg},{pitch_deg},{roll_deg},"
                f"{temperature_c},{salinity_psu},{pressure_dbar},"
                f"\"{velocity_profile_str}\"\n"
            )
        else:
            data_str = "\t".join(map(str, data))
            csv_str = ",".join(map(str, data))
            log_line = f"{t}\t{data_str}\n"
            csv_line = f"{t},{csv_str}\n"

        self.write_sensor_data('adcp', log_line, csv_line)

    def battery_callback(self, msg):
        t = self.get_local_timestamp()
        percentage_100 = msg.percentage * 100.0 if msg.percentage <= 1.0 else msg.percentage
        log_line = f"{t}\t{msg.voltage}\t{msg.current}\t{percentage_100}\n"
        csv_line = f"{t},{msg.voltage},{msg.current},{percentage_100}\n"
        self.write_sensor_data('battery', log_line, csv_line)

    def sbes_callback(self, msg):
        t = self.get_local_timestamp()

        if len(msg.data) >= 3:
            depth = msg.data[0]
            water_temp = msg.data[1]
            quality_flag = msg.data[2]
            log_line = f"{t}\t{depth}\t{water_temp}\t{quality_flag}\n"
            csv_line = f"{t},{depth},{water_temp},{quality_flag}\n"
        else:
            data_str = "\t".join(map(str, msg.data))
            csv_str = ",".join(map(str, msg.data))
            log_line = f"{t}\t{data_str}\n"
            csv_line = f"{t},{csv_str}\n"

        self.write_sensor_data('sbes', log_line, csv_line)

    # =========================================================
    # Metrics
    # =========================================================
    def get_sensor_rate(self, key, elapsed):
        current = self.sample_count.get(key, 0)
        previous = self.last_sample_count.get(key, 0)
        rate = (current - previous) / elapsed if elapsed > 0 else 0.0
        self.last_sample_count[key] = current
        return rate

    def log_periodic_metrics(self):
        now = time.time()
        elapsed = now - self.last_metrics_time if self.last_metrics_time else self.metrics_interval
        t = self.get_local_timestamp()

        write_speed = self.bytes_written_since_last_metrics / elapsed if elapsed > 0 else 0.0
        cpu_usage = psutil.cpu_percent(interval=None)
        ram_usage = psutil.virtual_memory().percent

        gps_hz = self.get_sensor_rate('gps', elapsed)
        imu_hz = self.get_sensor_rate('imu', elapsed)
        ctd_hz = self.get_sensor_rate('ctd', elapsed)
        adcp_hz = self.get_sensor_rate('adcp', elapsed)
        battery_hz = self.get_sensor_rate('battery', elapsed)
        sbes_hz = self.get_sensor_rate('sbes', elapsed)

        log_line = (
            f"{t}\t"
            f"{write_speed:.2f}\t"
            f"{cpu_usage:.2f}\t"
            f"{ram_usage:.2f}\t"
            f"{gps_hz:.2f}\t"
            f"{imu_hz:.2f}\t"
            f"{ctd_hz:.2f}\t"
            f"{adcp_hz:.2f}\t"
            f"{battery_hz:.2f}\t"
            f"{sbes_hz:.2f}\n"
        )

        csv_line = (
            f"{t},"
            f"{write_speed:.2f},"
            f"{cpu_usage:.2f},"
            f"{ram_usage:.2f},"
            f"{gps_hz:.2f},"
            f"{imu_hz:.2f},"
            f"{ctd_hz:.2f},"
            f"{adcp_hz:.2f},"
            f"{battery_hz:.2f},"
            f"{sbes_hz:.2f}\n"
        )

        for i, metrics_log_file in enumerate(self.metrics_log_files):
            try:
                metrics_log_file.write(log_line)
                metrics_log_file.flush()
                self.metrics_csv_files[i].write(csv_line)
                self.metrics_csv_files[i].flush()
            except Exception as e:
                self.get_logger().error(f"Metrics logging failed on target {i}: {e}")

        self.bytes_written_since_last_metrics = 0
        self.last_metrics_time = now

    # =========================================================
    # Shutdown
    # =========================================================
    def destroy_node(self):
        for key in self.files:
            for log_file in self.files[key]:
                log_file.close()
            for csv_file in self.csv_files[key]:
                csv_file.close()

        for metrics_log_file in self.metrics_log_files:
            metrics_log_file.close()
        for metrics_csv_file in self.metrics_csv_files:
            metrics_csv_file.close()

        try:
            os.sync()
        except AttributeError:
            pass

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