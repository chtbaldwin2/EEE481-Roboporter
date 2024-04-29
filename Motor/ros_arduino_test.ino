#include <Servo.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>

ros::NodeHandle nh;

Servo motor1;  // creat servo motor 1
Servo motor2;  // creat servo motor 2

// The servo motor receives angular values, but assume that it received linear velocity (-1.0 to 1.0) maps directly to the angle (0 to 180)
void cmdVelCallback(const geometry_msgs::Twist& cmd_msg) {
    int angle1 = map(cmd_msg.linear.x * 100, -100, 100, 0, 180);  // change line velocity to angle
    int angle2 = map(cmd_msg.angular.z * 100, -100, 100, 0, 180); // change angular velocity to angle

    motor1.write(angle1);  // set motor 1 angle 
    motor2.write(angle2);  // set motor 2 angle 
}

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", cmdVelCallback);


void setup() {
    nh.initNode();  // Initializing node
    nh.subscribe(sub);  // Subscribe to cmd_vel topic

    motor1.attach(5);  // set motor 1 to pin 5
    motor2.attach(6); // set motor 2 to pin 6
}

void loop() {
    nh.spinOnce();  // Processing ROS callback
    delay(10);
}


