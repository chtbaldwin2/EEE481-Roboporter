#include <Arduino.h>

// 定义引脚
const int encoderPinA_Left = 5;
const int encoderPinB_Left = 6;
const int encoderPinA_Right = 7;
const int encoderPinB_Right = 8;

// 编码器计数
volatile long encoderCount_Left = 0;
volatile long encoderCount_Right = 0;

// 上次检查的编码器计数
long lastEncoderCount_Left = 0;
long lastEncoderCount_Right = 0;

// 轮胎周长（单位：米）
const float wheelCircumference = 0.785 * PI; // 直径转换为米

// 编码器分辨率
const float encoderResolution = 1024.0;

// 上次检查的时间
unsigned long lastCheckTime = 0;

// 检查间隔（毫秒）
const long checkInterval = 1000; // 1秒

void setup() {
  Serial.begin(9600);

  // 设置引脚模式
  pinMode(encoderPinA_Left, INPUT);
  pinMode(encoderPinB_Left, INPUT);
  pinMode(encoderPinA_Right, INPUT);
  pinMode(encoderPinB_Right, INPUT);

  // 为引脚设置中断
  attachInterrupt(digitalPinToInterrupt(encoderPinA_Left), leftEncoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPinA_Right), rightEncoderISR, CHANGE);

  lastCheckTime = millis();
}

void loop() {
  if (millis() - lastCheckTime >= checkInterval) {
    // 计算速度
    float speed_Left = calculateSpeed(encoderCount_Left - lastEncoderCount_Left);
    float speed_Right = calculateSpeed(encoderCount_Right - lastEncoderCount_Right);

    // 显示速度和方向
    Serial.print("Left Speed: ");
    Serial.print(speed_Left);
    Serial.print(" m/s, Direction: ");
    Serial.println((encoderCount_Left - lastEncoderCount_Left) >= 0 ? "+" : "-");

    Serial.print("Right Speed: ");
    Serial.print(speed_Right);
    Serial.print(" m/s, Direction: ");
    Serial.println((encoderCount_Right - lastEncoderCount_Right) >= 0 ? "+" : "-");

    // 更新上次检查的计数和时间
    lastEncoderCount_Left = encoderCount_Left;
    lastEncoderCount_Right = encoderCount_Right;
    lastCheckTime = millis();
  }
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

float calculateSpeed(long encoderDelta) {
  // 计算旋转的圈数
  float rotations = encoderDelta / encoderResolution;

  // 计算距离（米）
  float distance = rotations * wheelCircumference;

  // 计算速度（米/秒）
  return distance / (checkInterval / 1000.0);
}
