#include <PID_v1.h>

// Encoder pins
const int leftEncoderPinA = 5;
const int leftEncoderPinB = 6;
const int rightEncoderPinA = 7;
const int rightEncoderPinB = 8;

// Motor control pins
const int leftMotorPin = 9;
const int rightMotorPin = 10;

// Constants for speed calculation
const float wheelDiameter = 0.25; // in meters
const int encoderCountsPerRevolution = 1024;

// Encoder counts
volatile long leftEncoderCount = 0;
volatile long rightEncoderCount = 0;

// PID variables
double Setpoint, Input, Output;

// PID parameters
double Kp = 2.0, Ki = 5.0, Kd = 1.0;

// PID controller
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

void setup() {
  // Initialize serial communication
  Serial.begin(9600);

  // Setup encoder pins
  pinMode(leftEncoderPinA, INPUT);
  pinMode(leftEncoderPinB, INPUT);
  pinMode(rightEncoderPinA, INPUT);
  pinMode(rightEncoderPinB, INPUT);

  // Attach interrupt for encoders
  attachInterrupt(digitalPinToInterrupt(leftEncoderPinA), leftEncoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(rightEncoderPinA), rightEncoderISR, CHANGE);

  // Initialize PID controller
  Setpoint = 0; // Set desired speed
  myPID.SetMode(AUTOMATIC);

  // Setup motor control pins
  pinMode(leftMotorPin, OUTPUT);
  pinMode(rightMotorPin, OUTPUT);
}

void loop() {
  // Calculate speed and direction
  calculateDirection();

  // Update PID controller
  Input = calculateSpeed();
  myPID.Compute();

  // Adjust motor speed based on PID output
  analogWrite(leftMotorPin, Output);
  analogWrite(rightMotorPin, Output);

  // Print speed and direction to serial monitor
  Serial.print("Speed: ");
  Serial.print(Input);
  Serial.print(" m/s, Direction: ");
  Serial.println((Output > 0) ? "+" : (Output < 0) ? "-" : "=");

  // Delay for stability
  delay(100);
}

// Encoder ISR for left encoder
void leftEncoderISR() {
  // Increment or decrement encoder count based on direction
  if (digitalRead(leftEncoderPinB) == digitalRead(leftEncoderPinA)) {
    leftEncoderCount--;
  } else {
    leftEncoderCount++;
  }
}

// Encoder ISR for right encoder
void rightEncoderISR() {
  // Increment or decrement encoder count based on direction
  if (digitalRead(rightEncoderPinB) == digitalRead(rightEncoderPinA)) {
    rightEncoderCount++;
  } else {
    rightEncoderCount--;
  }
}

// Function to calculate direction
char calculateDirection() {
  if (leftEncoderCount > rightEncoderCount) {
    return '+';
  } else if (leftEncoderCount < rightEncoderCount) {
    return '-';
  }
  return '=';
}
// Function to calculate speed
float calculateSpeed() {
  // Calculate the distance each wheel has moved
  float leftDistance = (leftEncoderCount / (float)encoderCountsPerRevolution) * PI * wheelDiameter;
  float rightDistance = (rightEncoderCount / (float)encoderCountsPerRevolution) * PI * wheelDiameter;

  // Calculate the average distance
  float averageDistance = (leftDistance + rightDistance) / 2.0;

  // Calculate speed (distance/time)
  float speed = averageDistance / 0.1; // Assuming loop runs every 100ms

  return speed;
}
