#include <Arduino.h>

// Define pins
const float encoderPinA_Left = 2;
const float encoderPinA_Right = 3;
const float encoderPinB_Left = 4;
const float encoderPinB_Right = 5;
const int motorPin_Left = 9;
const int motorPin_Right = 10;

// Encoder count
volatile long encoderCount_Left = 0;
volatile long encoderCount_Right = 0;

// Last encoder count
long lastEncoderCount_Left = 0;
long lastEncoderCount_Right = 0;

// Time of last measurement
unsigned long lastTime = 0;

// Tyre circumference in metres
const float wheelCircumference = 0.25 * PI; // diameter converted to metres

// Encoder resolution
const float encoderResolution = 1024.0;

// Target distance and current distance
float targetDistance = 0;
float currentDistance = 0;

// Motor speed control
const int maxMotorSpeed = 255; // Maximum PWM value
const int motorSpeedStep = 10; // speed step

// PWM value for motor
const int motorSpeedForward = 128; // forward speed 
const int motorSpeedBackward = 128; // backward speed
const int motorStop = 0; // stop

void setup() {
  Serial.begin(9600);

  // set pin mode
  pinMode(encoderPinA_Left, INPUT);
  pinMode(encoderPinB_Left, INPUT);
  pinMode(encoderPinA_Right, INPUT);
  pinMode(encoderPinB_Right, INPUT);
  pinMode(motorPin_Left, OUTPUT);
  pinMode(motorPin_Right, OUTPUT);

  // setting interrupts for pins
  attachInterrupt(digitalPinToInterrupt(encoderPinA_Left), leftEncoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPinA_Right), rightEncoderISR, CHANGE);
}

void loop() {
  if (Serial.available() > 0) {
    targetDistance = Serial.parseFloat(); // Read the target distance
    moveRobotToDistance(targetDistance);  // Controlling robot movement
  }

  // Calculate current distance
  currentDistance = calculateDistance(encoderCount_Left, encoderCount_Right);

  // Displays speed and direction
  displaySpeedAndDirection();
}

void leftEncoderISR() {
  if (digitalRead(encoderPinB_Left) == digitalRead(encoderPinA_Left)) {
    encoderCount_Left--;
  } else {
    encoderCount_Left++;
  }
}

void rightEncoderISR() {
  if (digitalRead(encoderPinB_Right) == digitalRead(encoderPinA_Right)) {
    encoderCount_Right++;
  } else {
    encoderCount_Right--;
  }
}

float calculateDistance(long countLeft, long countRight) {
  // Calculate the average number of revolutions of the left and right wheels
  float avgRotations = ((float)countLeft + (float)countRight) / 2.0 / encoderResolution;

  // Calculated Distance
  return avgRotations * wheelCircumference;
}

void displaySpeedAndDirection() {
// current time
  unsigned long currentTime = millis();
  // measuring interval
  unsigned long timeInterval = currentTime - lastTime;

  if (timeInterval >= 100) { // Measured every 100 ms
    // Calculate the number of revolutions for each wheel
    float rotationsLeft = (encoderCount_Left - lastEncoderCount_Left) / encoderResolution;
    float rotationsRight = (encoderCount_Right - lastEncoderCount_Right) / encoderResolution;

    // Calculation of average rotation speed (revolutions/second)
    float avgRotationsPerSec = ((rotationsLeft + rotationsRight) / 2.0) / (timeInterval / 1000.0);

    // Conversion to speed (m/s)
    float speed = avgRotationsPerSec * wheelCircumference;

    // Direction
    char direction = (encoderCount_Left - lastEncoderCount_Left) >= 0 ? '+' : '-';

    // Display speed and direction
    Serial.print("Speed: ");
    Serial.print(speed);
    Serial.print(" m/s, Direction: ");
    Serial.println(direction);

    // Update the last value
    lastEncoderCount_Left = encoderCount_Left;
    lastEncoderCount_Right = encoderCount_Right;
    lastTime = currentTime;
  }
}

void moveRobotToDistance(float distance) {
  // Reset encoder count
  encoderCount_Left = 0;
  encoderCount_Right = 0;

  // Setting the motor direction
  int motorDirection = distance >= 0 ? 1 : -1;

  // Setting the target encoder count
  long targetCount = abs(distance) / wheelCircumference * encoderResolution;

  // Control motor movement
  while (abs(encoderCount_Left) < targetCount && abs(encoderCount_Right) < targetCount) {
    int speed = min(maxMotorSpeed, motorSpeedStep * (abs(encoderCount_Left) / targetCount));
    analogWrite(motorPin_Left, speed * motorDirection);
    analogWrite(motorPin_Right, speed * motorDirection);
    delay(10); // delay
  }

  // motor stop
  analogWrite(motorPin_Left, motorStop);
  analogWrite(motorPin_Right, motorStop);
}
