#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <math.h>
#include <SoftwareSerial.h>
#include <SabertoothSimplified.h>

// set power level and decide on duration

SoftwareSerial SWSerial(NOT_A_PIN, 11); // RX on no pin (unused), TX on pin 11 (to S1).
SabertoothSimplified ST(SWSerial);      // Use SWSerial as the serial port.

SoftwareSerial portROS(0, 1);

//ms to wait  before killing motors
const long CONTROL_TIMEOUT = 2000;

// Declare a ROS node handler
ros::NodeHandle nh;

unsigned long lastData = 0;
// double w_l, w_r = {0};
int cnt = 0;
char cb_msg[50];
char lin_x_str[10];
char ang_z_str[10];

// motor 1 = right
// motor 2 right
void motor_cmd_cb_left(const std_msgs::Int32 &cmd_velocity_left)
{
  int w_l = cmd_velocity_left.data;
  ST.motor(2, 10);
  // nh.loginfo("recieved cmd_velocity_left: %s", std::to_string(w_l));
}
ros::Subscriber<std_msgs::Int32> sub_left_vel("motor_cmd_left", &motor_cmd_cb_left);

void motor_cmd_cb_right(const std_msgs::Int32 &cmd_velocity_right)
{
  int w_r = cmd_velocity_right.data;
  ST.motor(1, 10);
  // nh.loginfo("recieved cmd_velocity_right: %i", w_r);
}
ros::Subscriber<std_msgs::Int32> sub_right_vel("motor_cmd_right", &motor_cmd_cb_right);

void setup()
{
  portROS.begin(57600);
  SabertoothTXPinSerial.begin(9600); // This is the baud rate you chose with the DIP switches.

  // So, we set both to zero initially.
  // delay(20);

  nh.initNode();
  nh.subscribe(sub_left_vel);
  nh.subscribe(sub_right_vel);

  // nh.advertise(chatter_pub);
  //  handle.advertise(cmd_vel_ard_pub);
  ST.motor(1, 1);
  ST.motor(2, 1);
}

void loop()
{
  nh.spinOnce();

  if (millis() - lastData >= CONTROL_TIMEOUT)
  {
    lastData = millis();
    ST.motor(1, 0);
    ST.motor(2, 0);
    // chatter_pub.publish(&msg);
  }
}