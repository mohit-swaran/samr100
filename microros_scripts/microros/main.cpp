#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <micro_ros_utilities/type_utilities.h>
#include <micro_ros_utilities/string_utilities.h>

#include <microDrive.h>
#include <math.h>
#include <geometry_msgs/msg/twist.h>
#include <diagnostic_msgs/msg/diagnostic_status.h>
#include <diagnostic_msgs/msg/key_value.h>

#define M1PWM_L_PIN    21
#define M1PWM_R_PIN    20
#define M1ENC_A_PIN    9
#define M1ENC_B_PIN    8

#define M2PWM_L_PIN    19
#define M2PWM_R_PIN    18
#define M2ENC_A_PIN    12
#define M2ENC_B_PIN    11

#define M3PWM_L_PIN    16
#define M3PWM_R_PIN    17
#define M3ENC_A_PIN    15
#define M3ENC_B_PIN    14

#define LED_PIN 13


motor motor1(M1PWM_L_PIN, M1PWM_R_PIN, M1ENC_A_PIN, M1ENC_B_PIN);
motor motor2(M2PWM_L_PIN, M2PWM_R_PIN, M2ENC_A_PIN, M2ENC_B_PIN);
motor motor3(M3PWM_L_PIN, M3PWM_R_PIN, M3ENC_A_PIN, M3ENC_B_PIN);

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

rcl_subscription_t subscriber;
geometry_msgs__msg__Twist vel_msg;

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

// Subscription callback for received Twist messages
void subscription_callback(const void *msgin){
    const geometry_msgs__msg__Twist * vel_msg = (const geometry_msgs__msg__Twist *)msgin;
  
    float wheel1 = vel_msg->linear.x;
    float wheel2 = vel_msg->linear.y;
    float wheel3 = vel_msg->linear.z;
  
    motor1.setSpeed(wheel1);
    motor2.setSpeed(wheel2);
    motor3.setSpeed(wheel3);
  }

// Timer callback to publish debug data periodically
void debug_timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  
  // Collect and populate the debug message
  int encoder = motor3.getEncoderCounts();
  float battery = 11.89;
  float velocity= motor3.getVelocityFiltered();
  float eintegral = motor3.getEintegral();
  float setpoint = motor3.getSetpoint();

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

  // Initialize motors
  motor1.init(0);
  motor1.encoderTicksPerRevolution = 1170;
  motor1.setPI(3,35);

  motor2.init(1);
  motor2.encoderTicksPerRevolution = 1170;
  motor2.setPI(3,35);

  motor3.init(2);
  motor3.encoderTicksPerRevolution = 1170;
  motor3.setPI(3,35);

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

  // Create subscription for Twist messages
  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "/cmd_vel"));

  RCCHECK(rclc_timer_init_default(
  &debug_timer,
  &support,
  RCL_MS_TO_NS(20),  // publish every 500 ms, or adjust as needed
  debug_timer_callback));

  // Create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &vel_msg, &subscription_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_timer(&executor, &debug_timer));

}

void loop() { 
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));

  if (micros() - pidTimer >= pidInterval)
    {
        pidTimer = micros();
        motor1.updatePI(); 
        motor2.updatePI();
        motor3.updatePI();
    }
}