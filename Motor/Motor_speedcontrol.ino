#include <Servo.h>

Servo escLeft;  // Left motor
Servo escRight; // Right motor

volatile long encoderCountLeft = 0; // Left Encoder Count
volatile long encoderCountRight = 0; // Right Encoder Count

// Attached Pins
const int encoderPinLeft = 7; // Left encoder pin
const int encoderPinRight = 8; // Right encoder pin
const int escLeftPin = 9;  // Left motor control pin
const int escRightPin = 10; // Right motor control pin

int currentSpeedLeft = 1500; // Current left motor speed, initially set to neutral point
int currentSpeedRight = 1500; // Current right motor speed, initially set to neutral point
int targetSpeedLeft = 1500; // Motor speed on the left side of the target
int targetSpeedRight = 1500; // Target right motor speed

const int maxSpeed = 2500; // Maximum speed value
const int minSpeed = 500; // Minimum speed value
const int speedStep = 10; // Steps of speed change

void setup() {
  escLeft.attach(escLeftPin);
  escRight.attach(escRightPin);
  Serial.begin(9600);
  escLeft.writeMicroseconds(currentSpeedLeft);
  escRight.writeMicroseconds(currentSpeedRight);
  delay(2000);
  pinMode(encoderPinLeft, INPUT);
  pinMode(encoderPinRight, INPUT);
  attachInterrupt(digitalPinToInterrupt(encoderPinLeft), encoderLeftISR, RISING);
  attachInterrupt(digitalPinToInterrupt(encoderPinRight), encoderRightISR, RISING);
}

void updateSpeed() {
  // Update left motor speed
  if (currentSpeedLeft < targetSpeedLeft) {
    currentSpeedLeft += speedStep;
    if (currentSpeedLeft > targetSpeedLeft) {
      currentSpeedLeft = targetSpeedLeft;
    }
  } else if (currentSpeedLeft > targetSpeedLeft) {
    currentSpeedLeft -= speedStep;
    if (currentSpeedLeft < targetSpeedLeft) {
      currentSpeedLeft = targetSpeedLeft;
    }
  }
  escLeft.writeMicroseconds(currentSpeedLeft);

  // Update right motor speed
  if (currentSpeedRight < targetSpeedRight) {
    currentSpeedRight += speedStep;
    if (currentSpeedRight > targetSpeedRight) {
      currentSpeedRight = targetSpeedRight;
    }
  } else if (currentSpeedRight > targetSpeedRight) {
    currentSpeedRight -= speedStep;
    if (currentSpeedRight < targetSpeedRight) {
      currentSpeedRight = targetSpeedRight;
    }
  }
  escRight.writeMicroseconds(currentSpeedRight);
}

void loop() {
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');

    // Adjustment of target speed on command
    if (command == "speed L +") {
      targetSpeedLeft += 100;
    } else if (command == "speed L -") {
      targetSpeedLeft -= 100;
    } else if (command == "speed R +") {
      targetSpeedRight += 100;
    } else if (command == "speed R -") {
      targetSpeedRight -= 100;
    } else if (command == "speed max") {
      targetSpeedLeft = maxSpeed;
      targetSpeedRight = maxSpeed;
    } else if (command == "speed max L") {
      targetSpeedLeft = maxSpeed;
    } else if (command == "speed max R") {
      targetSpeedRight = maxSpeed;
    } else if (command == "speed min L") {
      targetSpeedLeft = minSpeed;
    } else if (command == "speed min R") {
      targetSpeedRight = minSpeed;
    } else if (command == "speed min") {
      targetSpeedLeft = minSpeed;
      targetSpeedRight = minSpeed;
    } else if (command == "stop") {
      targetSpeedLeft = 1500;
      targetSpeedRight = 1500;
    }

    targetSpeedLeft = constrain(targetSpeedLeft, minSpeed, maxSpeed);
    targetSpeedRight = constrain(targetSpeedRight, minSpeed, maxSpeed);
  }

  updateSpeed(); // Update speed
  delay(20); // Slight delay to smooth out speed changes
}

// Interrupt service routine for the left encoder
void encoderLeftISR() {
  encoderCountLeft++;
}

// Interrupt service routine for the right encoder
void encoderRightISR() {
  encoderCountRight++;
}

