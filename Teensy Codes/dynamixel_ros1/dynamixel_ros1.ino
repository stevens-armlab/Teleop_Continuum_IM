/* Yihan Aug15 2024
   communicating with ros1 using rosserial library
   receive command for ros1 and control dynamixel servos
*/
#include <ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <Dynamixel2Arduino.h>

#define DXL_SERIAL   Serial1
#define DEBUG_SERIAL Serial

const int DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN
const uint8_t SERVO_ID[] = {101, 102, 103, 104};
const int SERVO_NUM = 4;
const int MAX_VEL = 30;
const float DXL_PROTOCOL_VERSION = 2.0;
float privious_target[] = {140,265,130,140};
Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);
using namespace ControlTableItem;
ros::NodeHandle  nh;

void messageCb( const std_msgs::Float32MultiArray& servo_msg) {
  for (int i = 0; i < SERVO_NUM; i++) {
    
    if (abs(1.0*servo_msg.data[i] - privious_target[i]) <= 5.0) {
      digitalWrite(13, HIGH - digitalRead(13)); // blink the led
      dxl.setGoalPosition(SERVO_ID[i], 1.0 * servo_msg.data[i], UNIT_DEGREE);
      privious_target[i] = servo_msg.data[i];

    }
  }
}

ros::Subscriber<std_msgs::Float32MultiArray> sub("servo_msg", messageCb );

void setup() {
  // put your setup code here, to run once:
  //rosserial init
  nh.initNode();
  nh.subscribe(sub);
  pinMode(13, OUTPUT);
  // Set Port baudrate to 57600bps. This has to match with DYNAMIXEL baudrate.
  dxl.begin(1000000);
  // Set Port Protocol Version. This has to match with DYNAMIXEL protocol version.
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);

  // Turn off torque when configuring items in EEPROM area
  // Limit the maximum velocity in Position Control Mode. Use 0 for Max speed
  for (int i = 0; i < SERVO_NUM; i++) {
    dxl.torqueOff(SERVO_ID[i]);
    dxl.setOperatingMode(SERVO_ID[i], OP_POSITION);
    dxl.writeControlTableItem(PROFILE_VELOCITY, SERVO_ID[i], MAX_VEL);
    dxl.torqueOn(SERVO_ID[i]);
  }
  // Limit the maximum velocity in Position Control Mode. Use 0 for Max speed
}

void loop() {
  // put your main code here, to run repeatedly:
  nh.spinOnce();
  delay(500);
}
