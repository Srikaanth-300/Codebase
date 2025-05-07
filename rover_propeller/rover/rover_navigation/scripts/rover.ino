#include <ros.h>
#include <geometry_msgs/Twist.h>

// Motor pin definitions
#define left_wheel_forward_pwm 3
#define left_wheel_reverse_pwm 5
#define left_wheel_forward_enable 2
#define left_wheel_reverse_enable 4
#define right_wheel_forward_pwm 9
#define right_wheel_reverse_pwm 10
#define right_wheel_forward_enable 7
#define right_wheel_reverse_enable 8

// ROS Node
ros::NodeHandle nh;

// Function to set motor speeds
void setMotorSpeed(int leftSpeed, int rightSpeed) {
  if (leftSpeed > 0) {
    analogWrite(left_wheel_forward_pwm, leftSpeed);
    analogWrite(left_wheel_reverse_pwm, 0);
  } else {
    analogWrite(left_wheel_forward_pwm, 0);
    analogWrite(left_wheel_reverse_pwm, abs(leftSpeed));
  }

  if (rightSpeed > 0) {
    analogWrite(right_wheel_forward_pwm, rightSpeed);
    analogWrite(right_wheel_reverse_pwm, 0);
  } else {
    analogWrite(right_wheel_forward_pwm, 0);
    analogWrite(right_wheel_reverse_pwm, abs(rightSpeed));
  }
}

// Callback function for /cmd_vel messages
void cmdVelCallback(const geometry_msgs::Twist& msg) {
  float linear_x = msg.linear.x;  // Forward/Backward
  float angular_z = msg.angular.z; // Rotation

  // Convert to motor speeds
  int baseSpeed = linear_x * 255;    // Scale to PWM
  int turnSpeed = angular_z * 200;   // Adjust turning intensity

  int leftSpeed = baseSpeed - turnSpeed;
  int rightSpeed = baseSpeed + turnSpeed;

  // Constrain values to valid PWM range (0-255)
  leftSpeed = constrain(leftSpeed, -255, 255);
  rightSpeed = constrain(rightSpeed, -255, 255);

  // Set motor speed
  setMotorSpeed(leftSpeed, rightSpeed);
}

// Subscriber for cmd_vel
ros::Subscriber<geometry_msgs::Twist> sub("/cmd_vel", cmdVelCallback);

void setup() {
  nh.initNode();
  nh.subscribe(sub);

  pinMode(left_wheel_forward_pwm, OUTPUT);
  pinMode(left_wheel_reverse_pwm, OUTPUT);
  pinMode(left_wheel_forward_enable, OUTPUT);
  pinMode(left_wheel_reverse_enable, OUTPUT);
  pinMode(right_wheel_forward_pwm, OUTPUT);
  pinMode(right_wheel_reverse_pwm, OUTPUT);
  pinMode(right_wheel_forward_enable, OUTPUT);
  pinMode(right_wheel_reverse_enable, OUTPUT);

  digitalWrite(left_wheel_forward_enable, HIGH);
  digitalWrite(left_wheel_reverse_enable, HIGH);
  digitalWrite(right_wheel_forward_enable, HIGH);
  digitalWrite(right_wheel_reverse_enable, HIGH);
}

void loop() {
  nh.spinOnce();  // Process incoming ROS messages
  delay(10);
}