
import time 
import rclpy 
from icm20948 import ICM20948
from rclpy.node import Node
from sensor_msgs.msg import Imu
from sensor_msgs.msg import MagneticField

G_TO_MS2 = 9.80665
DEG_TO_RAD = 0.0174533
uT_TO_T = 1e-6 

class IMU(Node):
    def __init__(self):

        super().__init__("ICM20948_IMU_Node")
        self.pub = self.create_publisher(Imu,'/imu/data_raw',10)
        self.mag_pub=self.create_publisher(MagneticField,'/imu/mag',10)
        self.frequency = self.create_timer(0.033,self.timer_callback)
        self.mag_frequency = self.create_timer(0.033,self.mag_timer_callback)
        self.imu = ICM20948()
        self.get_logger().info("IMU node is running")

    def timer_callback(self):
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'imu_link'
        
        # Read the IMU data
        ax,ay,az,gx,gy,gz = self.imu.read_accelerometer_gyro_data()
        
        # Fill in the message fields with the IMU data
        msg.linear_acceleration.x = ax * G_TO_MS2
        msg.linear_acceleration.y = ay * G_TO_MS2
        msg.linear_acceleration.z = az * G_TO_MS2
        
        msg.angular_velocity.x = gx * DEG_TO_RAD
        msg.angular_velocity.y = gy * DEG_TO_RAD
        msg.angular_velocity.z = gz * DEG_TO_RAD

        # Publish the message
        self.pub.publish(msg)

    def mag_timer_callback(self):
        mag_msg = MagneticField()
        mag_msg.header.stamp = self.get_clock().now().to_msg()
        mag_msg.header.frame_id = 'imu_link'
        # Read the magnetometer data
        x,y,z = self.imu.read_magnetometer_data()
        # Fill in the message fields with the magnetometer data
        mag_msg.magnetic_field.x = x * uT_TO_T
        mag_msg.magnetic_field.y = y * uT_TO_T
        mag_msg.magnetic_field.z = z * uT_TO_T
        # Publish the message
        self.mag_pub.publish(mag_msg)
        
def main(args=None):
    rclpy.init(args=args)
    Imu_node = IMU()
    rclpy.spin(Imu_node)
    Imu_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()