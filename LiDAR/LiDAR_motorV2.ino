// Run a A4998 Stepstick from an Arduino UNO.
// Paul Hurley Aug 2015
int x; 
#define BAUD (9600)
#include <Wire.h>
#include <LIDARLite.h>

LIDARLite myLidarLite; 
unsigned long counter = 0; int samples = 25;
int angle; int output;

void setup() 
{
  Serial.begin(BAUD);
  pinMode(6,OUTPUT); // Enable
  pinMode(5,OUTPUT); // Step
  pinMode(4,OUTPUT); // Dir
  digitalWrite(6,LOW); // Set Enable low
  myLidarLite.begin(0, true);
  myLidarLite.configure(0);
}

void loop() 
{
  digitalWrite(6,LOW); // Set Enable low
  Serial.println("Loop 200 steps (1 rev)");

  // Every loop reverse direction
  if(counter % 2 == 0){
    Serial.println("Turning CCW");
    digitalWrite(4,HIGH);
  } else {
    Serial.println("Turning CW");
    digitalWrite(4,LOW);
  }

  for(x = 0; x < 200; x++) // Loop 200 times
  {
    angle = 1.8 * x;
    int reading = 0;
    digitalWrite(5,HIGH); // Output high
    delay(1); // Wait
    digitalWrite(5,LOW); // Output low

    // Measure Lidar reading 
    for(int i = 0; i < samples; i++)
    {
      reading += myLidarLite.distance();
      if(reading < 15){
        reading = 0;
      } else {
        reading = reading - 15;
      }
    }
    output = reading / samples;
    Serial.println(angle);
    Serial.println(",");
    Serial.println(output);
    Serial.println(",");
    delay(1);
  }
  counter += 1;
  Serial.println("Pause");
  delay(1000); // pause one second
}
