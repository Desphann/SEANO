import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import math
import time
import random


class ADCPSim(Node):
    def __init__(self):
        super().__init__('adcp_reader')

        self.declare_parameter('sample_rate', 1.0)
        self.sample_rate = float(self.get_parameter('sample_rate').value)

        # konfigurasi ADCP
        self.num_cells = 8
        self.num_beams = 4
        self.cell_size_m = 0.5
        self.blanking_distance_m = 0.7

        # kondisi awal dummy
        self.base_heading = 45.0
        self.base_pitch = 1.5
        self.base_roll = -0.8
        self.base_temp = 28.5
        self.base_salinity = 33.0
        self.base_pressure = 5.0

        self.publisher_ = self.create_publisher(Float64MultiArray, '/adcp/data', 10)
        self.start_time = time.time()
        self.timer = self.create_timer(1.0 / self.sample_rate, self.publish_adcp)

        self.get_logger().info(
            f"ADCP Reader Started | topic=/adcp/data | "
            f"sample_rate={self.sample_rate} Hz | "
            f"cells={self.num_cells} | beams={self.num_beams}"
        )

    def publish_adcp(self):
        t = time.time() - self.start_time

        # ===== metadata sensor yang berubah pelan =====
        heading = self.base_heading + 5.0 * math.sin(t * 0.03)
        pitch = self.base_pitch + 1.0 * math.sin(t * 0.07)
        roll = self.base_roll + 1.2 * math.cos(t * 0.06)
        temperature = self.base_temp + 0.3 * math.sin(t * 0.01)
        salinity = self.base_salinity + 0.1 * math.cos(t * 0.02)
        pressure = self.base_pressure + 0.5 * math.sin(t * 0.015)

        # ===== payload utama =====
        data = [
            float(self.num_cells),
            float(self.num_beams),
            float(self.cell_size_m),
            float(self.blanking_distance_m),
            round(heading, 3),
            round(pitch, 3),
            round(roll, 3),
            round(temperature, 3),
            round(salinity, 3),
            round(pressure, 3),
        ]

        # ===== velocity profile per cell dan beam =====
        # beam velocity dibuat bervariasi per depth cell + noise kecil
        for cell in range(self.num_cells):
            depth_factor = 1.0 - (cell * 0.08)

            # arus dasar berubah pelan terhadap waktu
            east_current = 0.8 * math.sin(t * 0.08 + cell * 0.15)
            north_current = 0.5 * math.cos(t * 0.06 + cell * 0.12)
            vertical_current = 0.05 * math.sin(t * 0.10 + cell * 0.20)

            # beam velocities dummy
            beam_1 = (east_current + north_current) * depth_factor + random.uniform(-0.03, 0.03)
            beam_2 = (-east_current + north_current) * depth_factor + random.uniform(-0.03, 0.03)
            beam_3 = (east_current - north_current) * depth_factor + random.uniform(-0.03, 0.03)
            beam_4 = vertical_current + random.uniform(-0.02, 0.02)

            data.extend([
                round(beam_1, 3),
                round(beam_2, 3),
                round(beam_3, 3),
                round(beam_4, 3),
            ])

        msg = Float64MultiArray()
        msg.data = data
        self.publisher_.publish(msg)

        self.log_summary(data)

    def log_summary(self, data):
        num_cells = int(data[0])
        num_beams = int(data[1])

        heading = data[4]
        pitch = data[5]
        roll = data[6]
        temperature = data[7]
        salinity = data[8]
        pressure = data[9]

        self.get_logger().info(
            f"ADCP | Heading={heading:.2f} deg | "
            f"Pitch={pitch:.2f} deg | Roll={roll:.2f} deg | "
            f"Temp={temperature:.2f} C | Sal={salinity:.2f} PSU | "
            f"Pressure={pressure:.2f} dbar"
        )

        values = data[10:]
        idx = 0
        for cell in range(num_cells):
            cell_values = values[idx:idx + num_beams]
            idx += num_beams
            self.get_logger().info(
                f"  Cell {cell + 1}: "
                f"B1={cell_values[0]:.3f}, "
                f"B2={cell_values[1]:.3f}, "
                f"B3={cell_values[2]:.3f}, "
                f"B4={cell_values[3]:.3f}"
            )


def main():
    rclpy.init()
    node = ADCPSim()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()