#include <Arduino.h>
#include <Servo.h>

Servo escLeft;  // Left motor
Servo escRight; // Right motor

// Define pins
const float encoderPinA_Left = 2;
const float encoderPinA_Right = 3;
const float encoderPinB_Left = 4;
const float encoderPinB_Right = 5;
const int escLeftPin = 9;  // Left motor control pin
const int escRightPin = 10; // Right motor control pin

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

int maxMotorSpeed = 2000;
int minMotorSpeed = 1000;
int motorSpeedStep = 100;

void setup() {
  Serial.begin(9600);

  // set pin mode
  pinMode(encoderPinA_Left, INPUT);
  pinMode(encoderPinB_Left, INPUT);
  pinMode(encoderPinA_Right, INPUT);
  pinMode(encoderPinB_Right, INPUT);

  escLeft.attach(escLeftPin);
  escRight.attach(escRightPin);

  escLeft.write(90);
  escRight.write(90);

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

    // Update the last value
    lastEncoderCount_Left = encoderCount_Left;
    lastEncoderCount_Right = encoderCount_Right;
    lastTime = currentTime;
  }
}

void moveRobotToDistance(float distance) {

  // Setting the motor direction
  int motorDirection = distance >= 0 ? 1 : -1;
  Serial.print("Direction = ");
  Serial.println(motorDirection);

  // Setting the target encoder count
  long targetCount = (abs(distance) / wheelCircumference) * encoderResolution;

  // Control motor movement
  while (abs(encoderCount_Left) < targetCount && abs(encoderCount_Right) < targetCount) {
    int speed = min(maxMotorSpeed, (motorSpeedStep * (abs(encoderCount_Left) / targetCount)));
    escLeft.writeMicroseconds(speed * motorDirection);
    escRight.writeMicroseconds(speed * motorDirection);
    delay(10); // delay
  }

  // motor stop
  escLeft.writeMicroseconds(1500);
  escRight.writeMicroseconds(1500);
}
