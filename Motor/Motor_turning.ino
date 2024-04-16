// Calculated angles converted to encoder counts
int angleToCounts(int angle) {
    const int countsPerRevolution = 1024; // Encoder count per revolution
    const float countsPerDegree = countsPerRevolution / 360.0; // Counts per degree
    return angle * countsPerDegree;
}

void turnRobot(int angle) {
    int encoderCounts = angleToCounts(angle);
    int direction = angle > 0 ? 1 : -1;

    desiredPIDCount_L = direction > 0 ? -encoderCounts : encoderCounts;
    desiredPIDCount_R = direction > 0 ? encoderCounts : -encoderCounts;

    sensorPIDCount_L = 0;
    sensorPIDCount_R = 0;

        // stat turning
    for(int speed = 90; speed <= 135; speed++) {
        esc_L.write(speed);
        esc_R.write(speed);
        delay(10); // make the turning more smoothness
    }

    while (abs(sensorPIDCount_L) < abs(encoderCounts) || abs(sensorPIDCount_R) < abs(encoderCounts)) {
        // speed control
        esc_L.write(135);
        esc_R.write(135);
    }

    moveRobot(90, 90); // stop
}

// get the command

void serialEvent() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    serialInput += inChar;
    if (inChar == '\n') {
      stringComplete = true;
    }
  }

  if (stringComplete) {
    if (serialInput.startsWith("+")) {
      int angle = serialInput.substring(1).toInt();
      turnRobot(angle); // left
    } else if (serialInput.startsWith("-")) {
      int angle = serialInput.substring(1).toInt();
      turnRobot(-angle); // right
    }

    serialInput = ""; // for next command
    stringComplete = false;
  }
}



