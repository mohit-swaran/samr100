#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <micro_ros_utilities/type_utilities.h>
#include <micro_ros_utilities/string_utilities.h>

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <motorDriver.h>

#include <geometry_msgs/msg/twist.h>
#include <sensor_msgs/msg/imu.h>
#include <diagnostic_msgs/msg/diagnostic_status.h>
#include <diagnostic_msgs/msg/key_value.h>


#define SDA_0   0
#define SCL_0   1 
#define addr    0x68

#define M1PWM_L_PIN    20
#define M1PWM_R_PIN    21
#define M1ENC_A_PIN    16
#define M1ENC_B_PIN    17

#define LED_PIN 13

arduino::MbedI2C Wire0(SDA_0, SCL_0);

Adafruit_MPU6050 imu;



motor motor1(M1PWM_L_PIN, M1PWM_R_PIN, M1ENC_A_PIN, M1ENC_B_PIN);
// motor motor2(M2PWM_L_PIN, M2PWM_R_PIN, M2ENC_A_PIN, M2ENC_B_PIN);
// motor motor3(M3PWM_L_PIN, M3PWM_R_PIN, M3ENC_A_PIN, M3ENC_B_PIN);

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

rcl_subscription_t subscriber;
geometry_msgs__msg__Twist vel_msg;

rcl_publisher_t publisher;
sensor_msgs__msg__Imu imu_msg;

rcl_publisher_t debug_publisher;
diagnostic_msgs__msg__DiagnosticStatus debug_msg;
diagnostic_msgs__msg__KeyValue encoder_kv;
diagnostic_msgs__msg__KeyValue battery_kv;
diagnostic_msgs__msg__KeyValue velocity_kv;
diagnostic_msgs__msg__KeyValue eintegral_kv;
diagnostic_msgs__msg__KeyValue setpoint_kv;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;
rcl_timer_t debug_timer;


extern "C" int clock_gettime(clockid_t unused, struct timespec *tp);

unsigned long pidTimer = 0;
const unsigned long pidInterval = 10000;

// Error handle loop
void error_loop(){
  while(1){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

// Timer callback to publish IMU data periodically
void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{  
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    // Read IMU data
    sensors_event_t a, g, temp;
    imu.getEvent(&a, &g, &temp);

    // Print out the IMU sensor data
    Serial.print(">Acc X: ");
    Serial.println(a.acceleration.x);
    Serial.print(">Acc Y: ");
    Serial.println(a.acceleration.y);
    Serial.print(">Acc Z: ");
    Serial.println(a.acceleration.z);

    Serial.print(">Rotation X: ");
    Serial.println(g.gyro.x);
    Serial.print(">Rotation Y: ");
    Serial.println(g.gyro.y);
    Serial.print(">Rotation Z: ");
    Serial.println(g.gyro.z);

    Serial.print(">Temperature: ");
    Serial.println(temp.temperature);
    Serial.println("");

    // Populate the IMU message
    imu_msg.angular_velocity.x = g.gyro.x;
    imu_msg.angular_velocity.y = g.gyro.y;
    imu_msg.angular_velocity.z = g.gyro.z;

    imu_msg.linear_acceleration.x = a.acceleration.x;
    imu_msg.linear_acceleration.y = a.acceleration.y;
    imu_msg.linear_acceleration.z = a.acceleration.z;

    imu_msg.linear_acceleration_covariance[0] = 0;
    imu_msg.angular_velocity_covariance[0] = 0;
    imu_msg.orientation_covariance[0] = -1;  // Unavailable data

    imu_msg.orientation.x = 0;
    imu_msg.orientation.y = 0;
    imu_msg.orientation.z = 0;
    imu_msg.orientation.w = 0;

    // Timestamp IMU data with current system time
    struct timespec tv = {0};
    clock_gettime(0, &tv);
    imu_msg.header.stamp.nanosec = tv.tv_nsec;
    imu_msg.header.stamp.sec = tv.tv_sec;
    imu_msg.header.frame_id = micro_ros_string_utilities_set(imu_msg.header.frame_id, "/imu");

    // Publish IMU message
    RCSOFTCHECK(rcl_publish(&publisher, &imu_msg, NULL));
  }
}

// Subscription callback for received Twist messages
void subscription_callback(const void *msgin){
  const geometry_msgs__msg__Twist * vel_msg = (const geometry_msgs__msg__Twist *)msgin;

  float wheel1 = vel_msg->linear.x;
  // float wheel2 = vel_msg->linear.y;
  // float wheel3 = vel_msg->linear.z;

  motor1.setSpeed(wheel1);
  // motor2.setSpeed(wheel2);
  // motor3.setSpeed(wheel3);
}

// Timer callback to publish debug data periodically
void debug_timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  
  // Collect and populate the debug message
  int encoder = motor1.getEncoderCounts();
  float battery = 11.89;
  float velocity= motor1.getVelocityFiltered();
  float eintegral = motor1.getEintegral();
  float setpoint = motor1.getSetpoint();

  debug_msg.name.data = (char *)"DebugData";
  debug_msg.name.size = strlen(debug_msg.name.data);
  debug_msg.name.capacity = strlen(debug_msg.name.data) + 1;

  encoder_kv.key.data = (char *)"Encoder";
  encoder_kv.key.size = strlen(encoder_kv.key.data);
  encoder_kv.key.capacity = strlen(encoder_kv.key.data) + 1;

  String encoder_str = String(encoder);
  encoder_kv.value.data = (char *)encoder_str.c_str();
  encoder_kv.value.size = encoder_str.length();
  encoder_kv.value.capacity = encoder_str.length() + 1;

  eintegral_kv.key.data = (char *)"Eintegral";
  eintegral_kv.key.size = strlen(eintegral_kv.key.data);
  eintegral_kv.key.capacity = strlen(eintegral_kv.key.data) + 1;

  String eintegral_str = String(eintegral);
  eintegral_kv.value.data = (char *)eintegral_str.c_str();
  eintegral_kv.value.size = eintegral_str.length();
  eintegral_kv.value.capacity = eintegral_str.length() + 1;

  setpoint_kv.key.data = (char *)"Setpoint";
  setpoint_kv.key.size = strlen(setpoint_kv.key.data);
  setpoint_kv.key.capacity = strlen(setpoint_kv.key.data) + 1;

  String setpoint_str = String(setpoint);
  setpoint_kv.value.data = (char *)setpoint_str.c_str();
  setpoint_kv.value.size = setpoint_str.length();
  setpoint_kv.value.capacity = setpoint_str.length() + 1;

  battery_kv.key.data = (char *)"Battery";
  battery_kv.key.size = strlen(battery_kv.key.data);
  battery_kv.key.capacity = strlen(battery_kv.key.data) + 1;

  String battery_str = String(battery, 2);
  battery_kv.value.data = (char *)battery_str.c_str();
  battery_kv.value.size = battery_str.length();
  battery_kv.value.capacity = battery_str.length() + 1;

  velocity_kv.key.data = (char *)"Velocity";
  velocity_kv.key.size = strlen(velocity_kv.key.data);
  velocity_kv.key.capacity = strlen(velocity_kv.key.data) + 1;

  String velocity_str = String(velocity);
  velocity_kv.value.data = (char *)velocity_str.c_str();
  velocity_kv.value.size = velocity_str.length();
  velocity_kv.value.capacity = velocity_str.length() + 1;

  // Assign key-value pairs to the message
  diagnostic_msgs__msg__KeyValue value_array[5]= {encoder_kv, eintegral_kv, setpoint_kv, velocity_kv, battery_kv};
  debug_msg.values.data = value_array;
  debug_msg.values.size = 5;

  // Publish the message
  RCSOFTCHECK(rcl_publish(&debug_publisher, &debug_msg, NULL));
}

void setup() {
  
  Serial.begin(115200);
  Serial.println("Initialising");
  set_microros_serial_transports(Serial);
  delay(2000);

  // Initialize IMU sensor
  Wire0.begin();
  if (!imu.begin(addr, &Wire0)) {
    Serial.println("Failed to initialize MPU6050");
    while(1){
        delay(10);
    }
  }
  Serial.println("MPU6050 initialized");
  imu.setAccelerometerRange(MPU6050_RANGE_16_G);
  imu.setGyroRange(MPU6050_RANGE_2000_DEG);
  imu.setFilterBandwidth(MPU6050_BAND_44_HZ);

  // Initialize motors
  motor1.init();
  motor1.encoderTicksPerRevolution = 1170;
  motor1.setPI(3,35);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  allocator = rcl_get_default_allocator();

  // Create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // Create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support));

  // Debug publisher
  RCCHECK(rclc_publisher_init_default(
    &debug_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(diagnostic_msgs, msg, DiagnosticStatus),
    "/debug"));

  RCCHECK(rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
    "/imu/data_raw"));

  // Create subscription for Twist messages
  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "/cmd_vel"));

  // Create timer to periodically publish IMU data
  const unsigned int timer_timeout = 10;
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));

  RCCHECK(rclc_timer_init_default(
  &debug_timer,
  &support,
  RCL_MS_TO_NS(20),  // publish every 500 ms, or adjust as needed
  debug_timer_callback));

  // Create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &vel_msg, &subscription_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
  RCCHECK(rclc_executor_add_timer(&executor, &debug_timer));

}

void loop() { 
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));

  if (micros() - pidTimer >= pidInterval)
    {
        pidTimer = micros();
        motor1.updatePI(); 
    }
}
