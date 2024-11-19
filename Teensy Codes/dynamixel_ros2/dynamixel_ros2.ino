/* Yihan Aug15 2024
   communicating with ros2 using micro-ros library
   receive command for ros2 and control dynamixel servos
*/
#include <micro_ros_arduino.h>
#include <Dynamixel2Arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/float32_multi_array.h>

// Please modify it to suit your hardware.
#define DXL_SERIAL   Serial1
#define DEBUG_SERIAL Serial
const int DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN

const uint8_t SERVO_ID[] = {101, 102, 103, 104};
const int SERVO_NUM = 4;
const int MAX_VEL = 30;
const float DXL_PROTOCOL_VERSION = 2.0;

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);
using namespace ControlTableItem;

rcl_subscription_t subscriber;
std_msgs__msg__Float32MultiArray msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

#define LED_PIN 13
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void error_loop() {
  while (1) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

void subscription_callback(const void * msgin)
{
  const std_msgs__msg__Float32MultiArray * msg = (const std_msgs__msg__Float32MultiArray *)msgin;
  //digitalWrite(LED_PIN,LOW);
  //delay(100);
  for (int i = 0; i < SERVO_NUM; i++) {
    dxl.setGoalPosition(SERVO_ID[i], msg->data.data[i], UNIT_DEGREE);
  }
}

void setup() {
  set_microros_transports();
  allocator = rcl_get_default_allocator();
  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  // create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support));
  // create subscriber
  RCCHECK(rclc_subscription_init_default(
            &subscriber,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
            "servo_command"));

  // Init the memory of your array in order to provide it to the executor.
  // If a message from ROS comes and it is bigger than this, it will be ignored, so ensure that capacities here are big enought.
  // this part solves the problem

  msg.data.capacity = 4;
  msg.data.size = 0;
  msg.data.data = (float_t*) malloc(msg.data.capacity * sizeof(float_t));

  msg.layout.dim.capacity = 4;
  msg.layout.dim.size = 0;
  msg.layout.dim.data = (std_msgs__msg__MultiArrayDimension*) malloc(msg.layout.dim.capacity * sizeof(std_msgs__msg__MultiArrayDimension));

  for (size_t i = 0; i < msg.layout.dim.capacity; i++) {
    msg.layout.dim.data[i].label.capacity = 4;
    msg.layout.dim.data[i].label.size = 0;
    msg.layout.dim.data[i].label.data = (char*) malloc(msg.layout.dim.data[i].label.capacity * sizeof(char));
  }

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));

  // Set Port baudrate to 57600bps. This has to match with DYNAMIXEL baudrate.
  dxl.begin(57600);


  // Set Port Protocol Version. This has to match with DYNAMIXEL protocol version.
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);

  // Turn off torque when configuring items in EEPROM area
  for (int i = 0; i < SERVO_NUM; i++) {
    dxl.torqueOff(SERVO_ID[i]);
    dxl.setOperatingMode(SERVO_ID[i], OP_POSITION);
    dxl.writeControlTableItem(PROFILE_VELOCITY, SERVO_ID[i], MAX_VEL);
    dxl.torqueOn(SERVO_ID[i]);
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}
