///////////////////////////////////
// ROBOPORTER MOTOR CONTROL CODE //
//  Version 6.0                  // 
///////////////////////////////////
// Input:   Speed value          //  
// Output:  Motor speed          // 
// Features:                     //
//  - L/R Encoders               //  
//  - L/R Speed Controllers (ESC)//
//  - Servo Library for control  //
//  - PID Loop for exact speed   //
///////////////////////////////////
// Written by:  Charlie Baldwin  //
//              Zhiyuan Liu      //
///////////////////////////////////

#include <Arduino.h>
#include <Servo.h>

// Servo Declaration
Servo esc_L; // left motor
Servo esc_R; // right motor

////////////////////
// PIN ASSIGNMENT //
////////////////////

// Pin assignment for both Left and Right Encoders - pins 2 and 3 are capable of interrupts
const int encoderPinA_L = 2;
const int encoderPinA_R = 3;
const int encoderPinB_L = 4;
const int encoderPinB_R = 5;

//pin assignment for ESCs
const int escPin_L = 9;
const int escPin_R = 10;

//pin assignement for US Sensors
#define trigPin1 6
#define echoPin1 7

///////////////
// CONSTANTS //
///////////////

const unsigned int maxSpeed = 150;
const unsigned int minSpeed = 30;
const float wheelCircumference = (0.25 * PI);
const unsigned int encoderRes = 1024;
const unsigned int pidPeriod = 5;

//////////////////////
// GLOBAL VARIABLES //
//////////////////////

// Timer - volatile so that data is read from RAM, not registers
volatile int timeout = 0;
volatile int loopCount = 0;

// PID Control
float kp_L = 1.0; //to be changed later
float ki_L = 0.5;
float kp_R = 1.0;
float ki_R = 0.5;

volatile int sensorPIDCount_L = 0;
volatile int sensorPIDCount_R = 0;
volatile float dPL = 0;
volatile float dPR = 0;
volatile float desiredPIDCount_L = 0;
volatile float desiredPIDCount_R = 0;

//used for testing
volatile int sensorDistCount_L = 0;
volatile int sensorDistCount_R = 0;

volatile int error_L = 0;
volatile int error_R = 0;
volatile int errorPrev_L = 0;
volatile int errorPrev_R = 0;

volatile int offset_L = 0;
volatile int offset_R = 0;

// 0 = forward, 1 = backward
boolean direction_L = 0;
boolean direction_R = 0;

String serialInput = "";
bool stringComplete = false;
int serialSpeed_L = 0;
int serialSpeed_R = 0;
int serialSpeedPrev_L = 0;
int serialSpeedPrev_R = 0;

int timercount = 0;

long duration;
long distance;
long TestSensor;

float motorDistance;

///////////
// SETUP //
///////////

void setup() {

  // Initialise Baud Rate for Serial
  Serial.begin(9600);
  serialInput.reserve(26);
  
  // Set up Interrupts used for PID control
  attachInterrupt(digitalPinToInterrupt(encoderPinA_L), encoderL_ISR, RISING);
  attachInterrupt(digitalPinToInterrupt(encoderPinA_R), encoderR_ISR, RISING);

  // Initialise Servos: 90 = stationary
  esc_L.write(90);
  esc_R.write(90);

  // Set up Outputs
  esc_L.attach(escPin_L);
  esc_R.attach(escPin_R);

  //set up pins for US sensors
  pinMode(trigPin1, OUTPUT);
  pinMode(echoPin1, INPUT);

  // Set up timer interrupt
  TCCR2A = 0; //set entire TCCR2A register to 0 
  TCCR2B = 0; //set entire TCCR2B register to 0
  TCNT2 = 0; //initialize counter value to 0

  //set compare match register for 2khz increments
  OCR2A = 124; // = (16*10^6) / (2000*64) - 1

  //turn on CTC mode
  TCCR2A |= (1 << WGM21);
  //set CS10 and CS12 bits for 1024 prescaler
  TCCR2B |= (1 << CS22) | (1 << CS21) | (0 << CS20); // 1024 prescaler
  //enable timer compare interrupt
  TIMSK2 |= (1 << OCIE2A);

  //Flush Serial Buffer
  Serial.flush();

}

/////////////////////////
// HARDWARE INTERRUPTS //
/////////////////////////

// Left Encoder Interrupt
void encoderL_ISR () {
  if (digitalRead(encoderPinB_L) == digitalRead(encoderPinA_L)) {

    sensorPIDCount_L ++;
    sensorDistCount_L ++;

  } else {

    sensorPIDCount_L --;
    sensorDistCount_L --;

  }
}

//Right Encoder Interrupt
void encoderR_ISR () {
  if (digitalRead(encoderPinB_R) == digitalRead(encoderPinA_R)) {

    sensorPIDCount_R --;
    sensorDistCount_R --;

  } else {

    sensorPIDCount_R ++;
    sensorDistCount_R ++;

  }
}

//PID Controller Interrupt
ISR (TIMER2_COMPA_vect) { //TIMER2_COMPA_vect is an interrupt handler which compares timer 2 and oc2ra and exectures the ISR when they are equal.

  sei();

  if (timercount >= 99) {

    timeout ++;

    error_L = desiredPIDCount_L - sensorPIDCount_L;
    error_R = desiredPIDCount_R - sensorPIDCount_R;
        
    sensorPIDCount_L = 0;
    sensorPIDCount_R = 0;

    offset_L = (kp_L * ((error_L - errorPrev_L)) + (ki_L * error_L));
    offset_R = (kp_R * ((error_R - errorPrev_R)) + (ki_R * error_R));

    errorPrev_L = error_L;
    errorPrev_R = error_R;

    offset_L = constrain(offset_L, -60, 90);
    offset_R = constrain(offset_R, -60, 90);

    esc_L.write(90 + offset_L);
    esc_R.write(90 + offset_R);

  } else {

    timercount ++;

  }

}

///////////////////////
// MOVEMENT FUNCTION //
///////////////////////

void moveRobot (int speed_L, int speed_R) {

  dPL = map(speed_L, 0, 180, -1024, 1024);
  dPR = map(speed_R, 0, 180, -1024, 1024);

  desiredPIDCount_L = dPL/pidPeriod;
  desiredPIDCount_R = dPR/pidPeriod;

}

void moveRobotToDistance(float motorDistance) {

  // Setting the motor direction
  int motorDirection = motorDistance >= 0 ? 1 : -1;
  Serial.print("Direction = ");
  Serial.println(motorDirection);

  // Setting the target encoder count
  long targetCount = (abs(motorDistance) / wheelCircumference) * encoderRes;

  // Control motor movement
  while (abs(sensorDistCount_L) < targetCount || abs(sensorDistCount_R) < targetCount) {
    moveRobot(serialSpeed_L, serialSpeed_R);

    SonarSensor(trigPin1, echoPin1);
    TestSensor = distance;
    Serial.println(TestSensor);

    if (TestSensor <= 10 && TestSensor > 5) {
      serialSpeed_L = 90*(20*motorDirection);
      serialSpeed_R = 90*(20*motorDirection);

   } else if (TestSensor <= 30 && TestSensor > 5) {
      serialSpeed_L = 110*(20*motorDirection);
      serialSpeed_R = 110*(20*motorDirection);

    } else if (TestSensor <= 60 && TestSensor > 5) {
      serialSpeed_L = 120*(20*motorDirection);
      serialSpeed_R = 120*(20*motorDirection);

    } else if (TestSensor > 60) {
      serialSpeed_L = 140*(20*motorDirection);
      serialSpeed_R = 140*(20*motorDirection);

    }

  }

  Serial.println("Complete");

  moveRobot(90, 90);

  serialSpeed_L = 0;
  serialSpeed_R = 0;
  sensorDistCount_L = 0;
  sensorDistCount_R = 0;
}

void SonarSensor(int trigPin,int echoPin)
{
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  distance = (duration/2) / 29.1;
}

// change the angle into encoder count

int angleToCounts(int angle) {
    const int countsPerRevolution = 1024; // encoder count per circle
    const float countsPerDegree = countsPerRevolution / 360.0; // count per degree
    return angle * countsPerDegree;
}

void turnRobot(int angle) {
    int encoderCounts = angleToCounts(angle);
    int direction = angle > 0 ? 1 : -1;

    desiredPIDCount_L = direction > 0 ? -encoderCounts : encoderCounts;
    desiredPIDCount_R = direction > 0 ? encoderCounts : -encoderCounts;

    sensorPIDCount_L = 0;
    sensorPIDCount_R = 0;

    while (abs(sensorPIDCount_L) < abs(encoderCounts) || abs(sensorPIDCount_R) < abs(encoderCounts)) {
        serialSpeed_L = 135; // turning speed
        serialSpeed_R = 135;
    }

    moveRobot(90, 90); // stop
}

//////////////////
// SERIAL EVENT //
//////////////////

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

///////////////
// MAIN LOOP //
///////////////

void loop() {

  if (stringComplete && serialInput.substring(0,1) == "$") {

    motorDistance = serialInput.substring(1,4).toInt();
    serialSpeed_L = map(serialInput.substring(4,6).toInt(), -99, 99, 0, 180);
    serialSpeed_R = map(serialInput.substring(6,8).toInt(), -99, 99, 0, 180);

    Serial.println(motorDistance);
    Serial.println(serialSpeed_L);
    Serial.println(serialSpeed_R);

    // clear the string:
    serialInput = "";
    stringComplete = false;

    moveRobotToDistance(motorDistance);

    /*if ((serialSpeed_L != serialSpeedPrev_L) || (serialSpeed_R != serialSpeedPrev_R)) {
      Serial.println("%VALID COMMAND%");
      moveRobot(serialSpeed_L, serialSpeed_R);
      serialSpeedPrev_L = serialSpeed_L;
      serialSpeedPrev_R = serialSpeed_R;
    }*/

  }  
  
  if (stringComplete) {
    if (serialInput.startsWith("+")) {
      int angle = serialInput.substring(1).toInt();
      turnRobot(angle); // turn right
    } else if (serialInput.startsWith("-")) {
      int angle = serialInput.substring(1).toInt();
      turnRobot(-angle); // turn left
    }

    serialInput = ""; // clean
    stringComplete = false;
  }

  //Serial.println(offset_L);
  //Serial.println(offset_R);

}
