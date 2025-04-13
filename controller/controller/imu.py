
import time 
import rclpy 
from icm20948 import ICM20948
from rclpy.node import Node
from sensor_msgs.msg import Imu

class IMU(Node):
    def __init__(self):

        super().__init__("ICM20948_IMU_Node")
        self.pub = self.create_publisher(Imu,'/imu/data_raw',10)
        self.frequency = self.create_timer(0.1,self.timer_callback)
        self.imu = ICM20948()

    def timer_callback(self):
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'imu_link'
        
        # Read the IMU data
        x,y,z = self.imu.read_magnetometer_data()
        ax,ay,az,gx,gy,gz = self.imu.read_accelerometer_gyro_data()
        
        # Fill in the message fields with the IMU data
        msg.linear_acceleration.x = ax
        msg.linear_acceleration.y = ay
        msg.linear_acceleration.z = az
        
        msg.angular_velocity.x = gx
        msg.angular_velocity.y = gy
        msg.angular_velocity.z = gz

        # Publish the message
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    Imu_node = IMU()
    rclpy.spin(Imu_node)
    Imu_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()