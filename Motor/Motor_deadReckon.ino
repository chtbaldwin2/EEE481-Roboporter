#include <Arduino.h>
#include <Servo.h>
#include <math.h>

// Servo Declaration
Servo esc_L; // left motor
Servo esc_R; // right motor

// Pin assignment for both Left and Right Encoders - pins 2 and 3 are capable of interrupts
const int encoderPinA_L = 4;
const int encoderPinA_R = 3;
const int encoderPinB_L = 2;
const int encoderPinB_R = 5;

//pin assignment for ESCs
const int escPin_L = 9;
const int escPin_R = 10;

const float wheelDiameter = 0.25;
const float wheelCircumference = (wheelDiameter * PI);
const float wheelSeparation = 0.45;
const float encoderRes = 1024;

volatile int encoderCount_L = 0;
volatile int encoderCount_R = 0;
volatile float encoderAngleCount_L = 0;
volatile float encoderAngleCount_R = 0;
volatile float encoderDistCount_L = 0;
volatile float encoderDistCount_R = 0;

float x;
float y;

float angle;
float dist;

String serialInput = "";
bool stringComplete = false;

void setup() {
 
  // Initialise Baud Rate for Serial
  Serial.begin(9600);
  serialInput.reserve(26);
  
  // Set up Interrupts used for PID control
  attachInterrupt(digitalPinToInterrupt(encoderPinA_L), encoderL_ISR, RISING);
  attachInterrupt(digitalPinToInterrupt(encoderPinA_R), encoderR_ISR, RISING);

  // Set up Outputs
  esc_L.attach(escPin_L);
  esc_R.attach(escPin_R);

  // Initialise Servos: 90 = stationary
  esc_L.write(90);
  esc_R.write(90);

  //Flush Serial Buffer
  Serial.flush();

}

// Left Encoder Interrupt
void encoderL_ISR () {
  if (digitalRead(encoderPinB_L) == digitalRead(encoderPinA_L)) {

    encoderCount_L --;
    encoderAngleCount_L --;
    encoderDistCount_L ++;

  } else {

    encoderCount_L ++;
    encoderAngleCount_L ++;
    encoderDistCount_L --;

  }
}

//Right Encoder Interrupt
void encoderR_ISR () {
  if (digitalRead(encoderPinB_R) == digitalRead(encoderPinA_R)) {

    encoderCount_R --;
    encoderAngleCount_R --;
    encoderDistCount_R --;

  } else {

    encoderCount_R ++;
    encoderAngleCount_R ++;
    encoderDistCount_R ++;

  }
}

void serialEvent() {
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    // add it to the inputString:
    serialInput += inChar;
    // if the incoming character is a newline, set a flag so the main loop can
    // do something about it:
    if (inChar == '\n') {
      stringComplete = true;
    }
  }
}

void moveAngle(float angle) {
  
  encoderAngleCount_L = 0;
  encoderAngleCount_R = 0;

  float rotations = ((angle * wheelSeparation) / (2*PI*wheelDiameter));
  float targetCount = (rotations * encoderRes);
  Serial.println(targetCount);

  while ((encoderAngleCount_L < targetCount) || (encoderAngleCount_R < targetCount)) {

    esc_L.write(70);
    esc_R.write(110);

    Serial.println(encoderAngleCount_L);
    Serial.println(encoderAngleCount_R);

  }

  esc_L.write(90);
  esc_R.write(90);

  encoderAngleCount_L = 0;
  encoderAngleCount_L = 0;

}

void moveDist(float dist) {

  encoderDistCount_L = 0;
  encoderDistCount_R = 0;

  float targetCount = ((dist/wheelCircumference) * encoderRes);

  while ((encoderDistCount_L < targetCount) || (encoderDistCount_R < targetCount)) {

    esc_L.write(110);
    esc_R.write(110);

    Serial.println(encoderDistCount_L);
    Serial.println(encoderDistCount_R);

  }

  esc_L.write(90);
  esc_R.write(90);

  encoderDistCount_L = 0;
  encoderDistCount_R = 0;

}

void loop() {
 
  if (stringComplete && serialInput.substring(0,1) == "$") {

    x = serialInput.substring(1,3).toFloat();
    y = serialInput.substring(3,5).toFloat();

    if ((x < 0) && (y < 0)) {
      angle = PI + (atan2(y,x));
    } else if ((x < 0) && (y >= 0)) {
      angle = PI - (atan2(y,x));
    } else if ((x >= 0) && (y < 0)) {
      angle = (2*PI) - (atan2(y,x));
    } else {
      angle = (atan2(y,x));
    }

    dist = sqrt(sq(x) + sq(y));

    Serial.println(angle);
    Serial.println(dist);

    moveAngle(angle);
    moveDist(dist);

    // clear the string:
    serialInput = "";
    stringComplete = false;

  }

}
