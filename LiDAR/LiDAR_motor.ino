// Run a A4998 Stepstick from an Arduino UNO.
// Paul Hurley Aug 2015
int x; 
#define BAUD (9600)
#include <Wire.h>
#include <LIDARLite.h>

LIDARLite myLidarLite; 

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
  digitalWrite(4,HIGH); // Set Dir high
  Serial.println("Loop 200 steps (1 rev)");

  for(x = 0; x < 200; x++) // Loop 200 times
  {
    digitalWrite(5,HIGH); // Output high
    delay(10); // Wait
    digitalWrite(5,LOW); // Output low
    delay(100); // Wait
  }

  Serial.println(myLidarLite.distance());
  for(int i = 0; i < 99; i++)
  {
    Serial.println(myLidarLite.distance(false));
  }
  Serial.println("Pause");
  delay(1000); // pause one second
}
