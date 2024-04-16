#include <Arduino.h>
#include <Servo.h>

// Servo Declaration
Servo esc_L; // left motor
Servo esc_R; // right motor

//pin assignment for ESCs
const int escPin_L = 9;
const int escPin_R = 10;

int speed_L = 0;
int speed_R = 0;
int speed_L_prev = 90;
int speed_R_prev = 90;
int speed_L_new = 0;
int speed_R_new = 0;

String data;

void setup() {
  
  Serial.begin(9600);

  esc_L.attach(escPin_L);
  esc_R.attach(escPin_R);

}

void loop() {
  
  if (Serial.available() > 0) {
    data = Serial.readStringUntil('\n');

    if (data == "goForward") {

      speed_L = 5;
      speed_R = 5;

    } else if (data == "goLeft") {

      speed_L = -5;
      speed_R = 5;

    } else if (data == "goRight") {

      speed_L = 5;
      speed_R = -5;

    } else if (data == "goBackward") {
      
      speed_L = -5;
      speed_R = -5;

    } else if (data == "stop") {

      speed_L = 90;
      speed_R = 90;
      speed_L_prev = 0;
      speed_R_prev = 0;

    }

    speed_L_new = constrain((speed_L_prev + speed_L), 20, 160);
    speed_R_new = constrain((speed_R_prev + speed_R), 20, 160);

    Serial.print("New L Speed: ");
    Serial.print(speed_L_new);
    Serial.print(", New R Speed: ");
    Serial.println(speed_R_new);

    esc_L.write(speed_L_new);
    esc_R.write(speed_R_new);

    speed_L_prev = speed_L_new;
    speed_R_prev = speed_R_new;

    speed_L = 0;
    speed_R = 0;

  }

}

