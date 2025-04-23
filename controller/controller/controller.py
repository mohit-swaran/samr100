import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
import math
import time
from enum import Enum
from tf_transformations import euler_from_quaternion  # Import the conversion function
from rclpy.time import Time  # Import the Time class from rclpy.time
from rclpy.duration import Duration # Import the Duration class


# Fuzzy control rule table (from the paper)
fuzzy_table = {
    (1, 1): (5, 3),  # PB, PB -> PB
    (1, 2): (5, 3),  # PB, PM -> PB
    (1, 3): (5, 3),  # PB, PS -> PB
    (1, 4): (5, 3),  # PB, NS -> PB
    (1, 5): (5, 2),  # PB, NM -> PM
    (1, 6): (5, 1),  # PB, NB -> PS
    (2, 1): (5, 3),  # PM, PB -> PB
    (2, 2): (4, 2),  # PM, PM -> PM
    (2, 3): (4, 2),  # PM, PS -> PM
    (2, 4): (4, 2),  # PM, NS -> PM
    (2, 5): (2, 1),  # PM, NM -> PS
    (2, 6): (2, 1),  # PM, NB -> PS
    (3, 1): (5, 2),  # PS, PB -> PM
    (3, 2): (4, 1),  # PS, PM -> PS
    (3, 3): (3, 1),  # PS, PS -> PS
    (3, 4): (1, 0),  # PS, NS -> 0
    (3, 5): (1, 0),  # PS, NM -> 0
    (3, 6): (1, 0),  # PS, NB -> 0
    (4, 1): (1, 0),  # NS, PB -> 0
    (4, 2): (1, 0),  # NS, PM -> 0
    (4, 3): (1, 0),  # NS, PS -> 0
    (4, 4): (3, 1),  # NS, NS -> PS
    (4, 5): (4, 1),  # NS, NM -> PS
    (4, 6): (5, 2),  # NS, NB -> PM
    (5, 1): (2, 1),  # NM, PB -> PS
    (5, 2): (2, 1),  # NM, PM -> PS
    (5, 3): (4, 2),  # NM, PS -> NM
    (5, 4): (4, 2),  # NM, NS -> NM
    (5, 5): (4, 2),  # NM, NM -> NM
    (5, 6): (5, 3),  # NM, NB -> NB
    (6, 1): (1, 1),  # NB, PB -> PS
    (6, 2): (1, 2),  # NB, PM -> PM
    (6, 3): (5, 2),  # NB, PS -> NB
    (6, 4): (5, 3),  # NB, NS -> NB
    (6, 5): (5, 3),  # NB, NM -> NB
    (6, 6): (5, 3),  # NB, NB -> NB
}

class YawMode(Enum):
    HEAD_FREE = 0
    HEAD_HOLD = 1

class BallbotControlNode(Node):
    def __init__(self):
        super().__init__('ballbot_control_node')

        # Declare ROS 2 parameters (for tuning controllers)
        self.declare_parameter('kp_roll', 1.5)  # Reduced from 1.0
        self.declare_parameter('kd_roll', 0.1)  # Reduced from 0.1
        self.declare_parameter('kp_pitch', 1.5)  # Reduced from 1.0
        self.declare_parameter('kd_pitch', 0.1)  # Reduced from 0.1
        self.declare_parameter('kp_pos_x', 0.25)  # Reduced from 0.5
        self.declare_parameter('ki_pos_x', 0.05)  # Reduced from 0.1
        self.declare_parameter('kd_pos_x', 0.025)  # Reduced from 0.05
        self.declare_parameter('kp_pos_y', 0.25)  # Reduced from 0.5
        self.declare_parameter('ki_pos_y', 0.05)  # Reduced from 0.1
        self.declare_parameter('kd_pos_y', 0.025)  # Reduced from 0.05
        self.declare_parameter('kp_yaw', 0.5)    # Reduced from 1.0
        self.declare_parameter('kd_yaw', 0.05)    # Reduced from 0.1
        self.declare_parameter('alpha', 45.0)  # Angle alpha in degrees

        # Get parameters
        self.kp_roll = self.get_parameter('kp_roll').get_parameter_value().double_value
        self.kd_roll = self.get_parameter('kd_roll').get_parameter_value().double_value
        self.kp_pitch = self.get_parameter('kp_pitch').get_parameter_value().double_value
        self.kd_pitch = self.get_parameter('kd_pitch').get_parameter_value().double_value
        self.kp_pos_x = self.get_parameter('kp_pos_x').get_parameter_value().double_value
        self.ki_pos_x = self.get_parameter('ki_pos_x').get_parameter_value().double_value
        self.kd_pos_x = self.get_parameter('kd_pos_x').get_parameter_value().double_value
        self.kp_pos_y = self.get_parameter('kp_pos_y').get_parameter_value().double_value
        self.ki_pos_y = self.get_parameter('ki_pos_y').get_parameter_value().double_value
        self.kd_pos_y = self.get_parameter('kd_pos_y').get_parameter_value().double_value
        self.kp_yaw = self.get_parameter('kp_yaw').get_parameter_value().double_value
        self.kd_yaw = self.get_parameter('kd_yaw').get_parameter_value().double_value
        self.alpha = self.get_parameter('alpha').get_parameter_value().double_value
        self.alpha_rad = math.radians(self.alpha) # Store alpha in radians
        self.w1_prev = 0.0
        self.w2_prev = 0.0
        self.w3_prev = 0.0

        # Subscribe to IMU data
        self.imu_sub = self.create_subscription(
            Imu, 'imu/data', self.imu_callback, 10)
        self.imu_data = Imu()  # Store the latest IMU message

        # Publisher for wheel velocities
        self.cmd_vel_pub = self.create_publisher(
            Twist, 'cmd_vel', 10)
        self.cmd_vel_msg = Twist()

        # Initialize control states
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.roll_rate = 0.0
        self.pitch_rate = 0.0
        self.yaw_rate = 0.0
        self.x_position = 0.0  # Not directly from IMU, needs integration
        self.y_position = 0.0  # Not directly from IMU, needs integration
        self.x_position_error_sum = 0.0 # Integral term
        self.y_position_error_sum = 0.0 # Integral term
        self.prev_time = self.get_clock().now() # for calculating dt, changed to a Time object
        self.max_rpm = 50.0  # Maximum RPM for the wheels
        # Control mode
        self.yaw_mode = YawMode.HEAD_FREE  # Default yaw mode  # Change this line to switch modes

        # Timer for the control loop.  50Hz is suggested by the paper,
        # but you can adjust it as needed.
        self.control_timer = self.create_timer(0.02, self.control_loop)  # 50Hz

    def imu_callback(self, msg):
        """
        Callback function for IMU data.  Stores the roll, pitch,
        yaw, and their rates.  Handles the quaternion to Euler
        angle conversion using tf_transformations.
        """
        self.imu_data = msg # Store the IMU message
        # Extract quaternion components
        qx = msg.orientation.x
        qy = msg.orientation.y
        qz = msg.orientation.z
        qw = msg.orientation.w

        # Convert quaternion to Euler angles
        (self.roll, self.pitch, self.yaw) = euler_from_quaternion([qx, qy, qz, qw])

        # Angular velocities (roll, pitch, yaw rates)
        self.roll_rate = msg.angular_velocity.x
        self.pitch_rate = msg.angular_velocity.y
        self.yaw_rate = msg.angular_velocity.z

    def fuzzy_control(self, angle_error, angle_rate_error):
        """
        Fuzzy logic controller for adjusting PD gains.  Uses the
        table provided in the paper.

        :param angle_error: Angle error (e.g., roll error).
        :param angle_rate_error: Angle rate error (e.g., roll rate error).
        :return: (Kp, Kd) - Adjusted PD gains.
        """
        # Quantize errors to fit the table (adjust as needed for your application)
        angle_error_quantized = int(math.copysign(min(abs(angle_error) / 0.1, 6), angle_error)) # Example scaling
        angle_rate_error_quantized = int(math.copysign(min(abs(angle_rate_error) / 0.1, 6), angle_rate_error)) # Example scaling

        # Ensure the quantized values are within the table bounds
        angle_error_quantized = max(1, min(angle_error_quantized, 6))
        angle_rate_error_quantized = max(1, min(angle_rate_error_quantized, 6))

        kp_kd = fuzzy_table[(angle_error_quantized, angle_rate_error_quantized)]
        self.get_logger().info(
            f"Fuzzy Rule Fired -> Error ({angle_error:.3f}), Rate Error ({angle_rate_error:.3f}) | "
            f"Quantized: ({angle_error_quantized}, {angle_rate_error_quantized}) | "
            f"Fuzzy Gains: Kp={kp_kd[0]}, Kd={kp_kd[1]}"
        )
        return kp_kd    

    def balancing_control(self):
        """
        Calculates the control output for balancing (roll and pitch).
        Uses a cascade PD controller with fuzzy gain tuning for the
        outer loop.
        """
        # Roll control
        roll_error = -self.roll  # Target roll angle is 0
        roll_rate_error = -self.roll_rate
        kp_roll_fuzzy, kd_roll_fuzzy = self.fuzzy_control(roll_error, roll_rate_error)
        # Use gains.
        desired_roll_rate = kp_roll_fuzzy * roll_error # Outer loop
        roll_acceleration = self.kp_roll * (desired_roll_rate - self.roll_rate) + self.kd_roll * roll_rate_error # Inner loop

        # Pitch control (same structure as roll)
        pitch_error = -self.pitch
        pitch_rate_error = -self.pitch_rate
        kp_pitch_fuzzy, kd_pitch_fuzzy = self.fuzzy_control(pitch_error, pitch_rate_error)
        desired_pitch_rate = kp_pitch_fuzzy * pitch_error
        pitch_acceleration = self.kp_pitch * (desired_pitch_rate - self.pitch_rate) + self.kd_pitch * pitch_rate_error

        return roll_acceleration, pitch_acceleration

    def position_control(self):
        """
        Calculates the desired roll and pitch angles for position control.
        Uses a PI controller for the outer loop (position) and a PD
        controller for balancing. This function outputs desired roll and pitch angles
        based on x and y position errors.
        """
        # Simulate position error (you may want to replace this with real localization data)
        desired_x = 0.0  # Target position in x
        desired_y = 0.0  # Target position in y
        x_error = desired_x - self.x_position
        y_error = desired_y - self.y_position

        # Time step calculation
        now = self.get_clock().now()
        dt = (now - self.prev_time).nanoseconds / 1e9  # Convert to seconds
        self.prev_time = now

        if dt <= 0:
            dt = 1e-3  # Avoid division by zero

        # Integral error accumulation
        self.x_position_error_sum += x_error * dt
        self.y_position_error_sum += y_error * dt

        # Derivative terms can be added later if needed
        # Compute desired pitch and roll (forward/backward -> pitch, left/right -> roll)
        desired_pitch = self.kp_pos_x * x_error + self.ki_pos_x * self.x_position_error_sum
        desired_roll = self.kp_pos_y * y_error + self.ki_pos_y * self.y_position_error_sum

        # Limit angles (optional, for safety)
        max_angle = math.radians(5)  # Limit to 10 degrees
        desired_pitch = max(-max_angle, min(desired_pitch, max_angle))
        desired_roll = max(-max_angle, min(desired_roll, max_angle))

        return desired_roll, desired_pitch


    def yaw_control(self):
        """
        Calculates the desired yaw rate based on the selected mode
        (HEAD_FREE or HEAD_HOLD).
        """
        if self.yaw_mode == YawMode.HEAD_FREE:
            # In HEAD_FREE mode, we don't try to control the yaw,
            # so we target a zero yaw rate.
            desired_yaw_rate = 0.0
        elif self.yaw_mode == YawMode.HEAD_HOLD:
            # In HEAD_HOLD mode, we try to maintain the current yaw.
            target_yaw = 0.0 # set this to the heading you want to maintain
            yaw_error = target_yaw - self.yaw
            desired_yaw_rate = self.kp_yaw * yaw_error - self.kd_yaw * self.yaw_rate
        return desired_yaw_rate

    def calculate_wheel_velocities(self, roll_acceleration, pitch_acceleration, desired_yaw_rate):
        """
        Calculates the individual wheel velocities based on the
        desired accelerations and yaw rate.  This is where the
        kinematics of the ballbot are used.  The equations are
        derived from the paper.

        :param roll_acceleration:  Acceleration in the roll direction.
        :param pitch_acceleration: Acceleration in the pitch direction.
        :param desired_yaw_rate:  Desired yaw rate.
        :return: (w1, w2, w3) - Angular velocities of the three wheels.
        """

        # Deadzone to suppress jitter near zero
        if abs(roll_acceleration) < 0.05:
            roll_acceleration = 0.0
        if abs(pitch_acceleration) < 0.05:
            pitch_acceleration = 0.0
        if abs(desired_yaw_rate) < 0.02:
            desired_yaw_rate = 0.0

        #  The paper uses v_s1, v_s2, v_s3, which are tangential velocities.
        #  We need to convert them to angular velocities (w1, w2, w3)
        #  by dividing by the wheel radius (r).  Let's assume a wheel radius.
        wheel_radius = 0.1  #  adjust this value as needed (in meters)
        
        #  Equations from the paper (Equation 18), but converted to angular velocities
        w1_rad_per_sec = (pitch_acceleration * math.sin(self.alpha_rad) - desired_yaw_rate * math.cos(self.alpha_rad)) / wheel_radius
        w2_rad_per_sec = ( (math.sqrt(3)/2) * roll_acceleration * math.sin(self.alpha_rad) +
               (1/2) * pitch_acceleration * math.sin(self.alpha_rad) -
               desired_yaw_rate * math.cos(self.alpha_rad)) / wheel_radius
        w3_rad_per_sec = ( (-math.sqrt(3)/2) * roll_acceleration * math.sin(self.alpha_rad) +
               (1/2) * pitch_acceleration * math.sin(self.alpha_rad) -
               desired_yaw_rate * math.cos(self.alpha_rad)) / wheel_radius

        # Convert from rad/s to RPM
        w1_rpm = (w1_rad_per_sec * 60) / (2 * math.pi)
        w2_rpm = (w2_rad_per_sec * 60) / (2 * math.pi)
        w3_rpm = (w3_rad_per_sec * 60) / (2 * math.pi)

        # Low-pass filter the RPMs for smoother output
        # alpha = 0.1
        # self.w1_prev = alpha * w1_rpm + (1 - alpha) * self.w1_prev
        # self.w2_prev = alpha * w2_rpm + (1 - alpha) * self.w2_prev
        # self.w3_prev = alpha * w3_rpm + (1 - alpha) * self.w3_prev

        # Clamp RPMs
    
        w1_rpm = max(-self.max_rpm, min(self.w1_prev, self.max_rpm))
        w2_rpm = max(-self.max_rpm, min(self.w2_prev, self.max_rpm))
        w3_rpm = max(-self.max_rpm, min(self.w3_prev, self.max_rpm))

        return w1_rpm, w2_rpm, w3_rpm

    def control_loop(self):
        """
        Main control loop.  Executes the control algorithm and
        publishes the wheel velocities.
        """
        if self.imu_data.header.stamp.sec == 0:
            self.get_logger().warn("Waiting for IMU data...")

            # Publish stop command (zero RPM)
            stop_msg = Twist()
            stop_msg.linear.x = 0.0
            stop_msg.linear.y = 0.0
            stop_msg.linear.z = 0.0
            self.cmd_vel_pub.publish(stop_msg)
            
            return  # Wait for valid IMU data

        # Balancing control
        roll_acceleration, pitch_acceleration = self.balancing_control()

        # Position control
        # desired_roll, desired_pitch = self.position_control() # Comment out position control

        # Superpose the desired angles from position control onto the balancing control
        # roll_acceleration += desired_roll # Comment out position control
        # pitch_acceleration += desired_pitch # Comment out position control

        # Yaw control
        desired_yaw_rate = self.yaw_control()

        # Calculate wheel velocities (in RPM)
        w1, w2, w3 = self.calculate_wheel_velocities(roll_acceleration, pitch_acceleration, desired_yaw_rate)

        # Publish wheel velocities (using Twist message)
        self.cmd_vel_msg.linear.x = w1
        self.cmd_vel_msg.linear.y = w2
        self.cmd_vel_msg.linear.z = w3
        self.cmd_vel_pub.publish(self.cmd_vel_msg)

        #  Update position (very basic integration -  Good enough for testing)
        # current_time = self.get_clock().now()
        # dt = (current_time - self.prev_time).to_sec()
        # self.x_position += (self.roll_rate * dt) #  Use roll rate as a proxy for x velocity
        # self.y_position += (self.pitch_rate * dt) # Use pitch rate as proxy for y velocity
        # self.prev_time = current_time

        # Log some data for debugging and tuning
        self.get_logger().info(f"Roll: {self.roll:.2f}, Pitch: {self.pitch:.2f}, Yaw: {self.yaw:.2f}, W1: {w1:.2f} RPM, W2: {w2:.2f} RPM, W3: {w3:.2f} RPM")


    # def destroy_node(self):
    #     try:
    #         # Publish stop message before shutdown
    #         stop_msg = Twist()
    #         stop_msg.linear.x = 0.0
    #         stop_msg.linear.y = 0.0
    #         stop_msg.linear.z = 0.0
    #         self.cmd_vel_pub.publish(stop_msg)
    #         self.get_logger().info("Published zero velocity to stop the robot.")
    #     except rclpy._rclpy_pybind11.RCLError as e:
    #         self.get_logger().warn(f"Failed to publish stop message: {e}")
        
    #     # Proceed with node destruction after publishing
    #     super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    ballbot_node = BallbotControlNode()
    try:
        rclpy.spin(ballbot_node)
    except KeyboardInterrupt:
        pass
    finally:
        # ballbot_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

