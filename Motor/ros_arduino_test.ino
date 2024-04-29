#include <ros.h>
#include <geometry_msgs/Twist.h>

ros::NodeHandle nh;

int motor1Pin = 5; // Arduino to first motor 
int motor2Pin = 6; // Arduino to second motor

void cmdVelCallback(const geometry_msgs::Twist& cmd_msg) {
  int motor1Speed = cmd_msg.linear.x * 255; // change the speed to pwm（0-255）
  int motor2Speed = cmd_msg.angular.z * 255; // change the speed to pwm（0-255）

  analogWrite(motor1Pin, abs(motor1Speed));
  analogWrite(motor2Pin, abs(motor2Speed));
}

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", cmdVelCallback); set the feedback topic to ros

void setup() {
  pinMode(motor1Pin, OUTPUT);
  pinMode(motor2Pin, OUTPUT);
  nh.initNode();
  nh.subscribe(sub);
}

void loop() {
  nh.spinOnce();
  delay(10);
}


