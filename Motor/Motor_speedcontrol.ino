#include <Servo.h>

Servo escLeft;  // 左侧电机
Servo escRight; // 右侧电机
const int escLeftPin = 9;  // 左侧电机控制引脚
const int escRightPin = 10; // 右侧电机控制引脚

int currentSpeedLeft = 1500; // 当前左侧电机速度，初始设为中立点
int currentSpeedRight = 1500; // 当前右侧电机速度，初始设为中立点
int targetSpeedLeft = 1500; // 目标左侧电机速度
int targetSpeedRight = 1500; // 目标右侧电机速度

const int maxSpeed = 2000; // 最大速度值
const int minSpeed = 1000; // 最小速度值
const int speedStep = 10; // 速度改变的步长

void setup() {
  escLeft.attach(escLeftPin);
  escRight.attach(escRightPin);
  Serial.begin(9600);
  escLeft.writeMicroseconds(currentSpeedLeft);
  escRight.writeMicroseconds(currentSpeedRight);
  delay(2000);
}

void updateSpeed() {
  // 更新左侧电机速度
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

  // 更新右侧电机速度
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

    // 根据指令调整目标速度
    if (command == "speed L +") {
      targetSpeedLeft += 100;
    } else if (command == "speed L -") {
      targetSpeedLeft -= 100;
    } else if (command == "speed R +") {
      targetSpeedRight += 100;
    } else if (command == "speed R -") {
      targetSpeedRight -= 100;
    } else if (command == "speed max L") {
      targetSpeedLeft = maxSpeed;
    } else if (command == "speed max R") {
      targetSpeedRight = maxSpeed;
    } else if (command == "stop") {
      targetSpeedLeft = 1500;
      targetSpeedRight = 1500;
    }

    targetSpeedLeft = constrain(targetSpeedLeft, minSpeed, maxSpeed);
    targetSpeedRight = constrain(targetSpeedRight, minSpeed, maxSpeed);
  }

  updateSpeed(); // 更新速度
  delay(20); // 稍微延迟以平滑速度变化
}
