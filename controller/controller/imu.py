import time
import math
import yaml
from pathlib import Path

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField
from icm20948 import ICM20948

# Constants
G_TO_MS2 = 9.80665
DEG_TO_RAD = 0.0174533
uT_TO_T = 1e-6


class IMU(Node):
    def __init__(self):
        super().__init__("ICM20948_IMU_Node")

        # Publishers
        self.pub = self.create_publisher(Imu, '/imu/data_raw', 10)
        self.mag_pub = self.create_publisher(MagneticField, '/imu/mag', 10)

        # Device and calibration
        self.imu = ICM20948()
        self.offset_file = Path(__file__).parent / "mag_offsets.yaml"
        self.mag_offsets = [0.0, 0.0, 0.0]

        # ROS param
        self.declare_parameter("calibrate_mag", False)
        self.calibrate_mag = self.get_parameter("calibrate_mag").get_parameter_value().bool_value

        # Run calibration or load offsets
        if self.calibrate_mag:
            self.print_calib_instructions()
            self.calibrate_magnetometer()
        else:
            self.load_offsets()

        # Timers for publishing
        self.create_timer(0.02, self.timer_callback)        # IMU @ 50Hz
        self.create_timer(0.02, self.mag_timer_callback)    # MAG @ 50Hz

        self.get_logger().info("✅ IMU Node is running!")

    def print_calib_instructions(self):
        self.get_logger().info("\n🔧 Starting magnetometer calibration...")
        self.get_logger().info("👉 Move the IMU slowly in a figure-8 motion in all directions")
        self.get_logger().info("   Ensure full 360° rotation around X, Y, and Z axes")
        self.get_logger().info("   (e.g., tip, tilt, rotate like stirring air with a spoon!)")
        self.get_logger().info("⏳ Calibration will run for ~15 seconds...\n")

    def calibrate_magnetometer(self, duration=15):
        mag_min = [float('inf')] * 3
        mag_max = [-float('inf')] * 3
        start = time.time()

        while time.time() - start < duration:
            mag = list(self.imu.read_magnetometer_data())
            for i in range(3):
                mag_min[i] = min(mag_min[i], mag[i])
                mag_max[i] = max(mag_max[i], mag[i])

            # Log countdown every second
            remaining = duration - int(time.time() - start)
            self.get_logger().info( f"⏳ Calibrating... {remaining}s remaining")
            time.sleep(0.1)

        self.mag_offsets = [(mag_max[i] + mag_min[i]) / 2 for i in range(3)]
        self.get_logger().info(f"✅ Calibration complete. Offsets: {self.mag_offsets}")
        self.save_offsets()

    def save_offsets(self):
        data = {"offsets": self.mag_offsets}
        with open(self.offset_file, "w") as f:
            yaml.dump(data, f)
        self.get_logger().info(f"📁 Offsets saved to: {self.offset_file}")

    def load_offsets(self):
        if self.offset_file.exists():
            with open(self.offset_file, "r") as f:
                data = yaml.safe_load(f)
                self.mag_offsets = data.get("offsets", [0.0, 0.0, 0.0])
            self.get_logger().info(f"📂 Loaded magnetometer offsets: {self.mag_offsets}")
        else:
            self.get_logger().warn("⚠️ Offset file not found. Using [0, 0, 0].")

    def timer_callback(self):
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'imu_link'

        ax, ay, az, gx, gy, gz = self.imu.read_accelerometer_gyro_data()

        msg.linear_acceleration.x = ax * G_TO_MS2
        msg.linear_acceleration.y = ay * G_TO_MS2
        msg.linear_acceleration.z = az * G_TO_MS2

        msg.angular_velocity.x = gx * DEG_TO_RAD
        msg.angular_velocity.y = gy * DEG_TO_RAD
        msg.angular_velocity.z = gz * DEG_TO_RAD

        self.pub.publish(msg)

    def mag_timer_callback(self):
        mag_msg = MagneticField()
        mag_msg.header.stamp = self.get_clock().now().to_msg()
        mag_msg.header.frame_id = 'imu_link'

        x, y, z = self.imu.read_magnetometer_data()
        x -= self.mag_offsets[0]
        y -= self.mag_offsets[1]
        z -= self.mag_offsets[2]

        mag_msg.magnetic_field.x = x * uT_TO_T
        mag_msg.magnetic_field.y = y * uT_TO_T
        mag_msg.magnetic_field.z = z * uT_TO_T

        self.mag_pub.publish(mag_msg)


def main(args=None):
    rclpy.init(args=args)
    node = IMU()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

