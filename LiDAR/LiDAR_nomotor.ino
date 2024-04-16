/*------------------------------------------------------------------------------

  LIDARLite Arduino Library
  v3/GetDistanceI2c

  This example shows how to initialize, configure, and read distance from a
  LIDAR-Lite connected over the I2C interface.

  Connections:
  LIDAR-Lite 5 Vdc (red) to Arduino 5v
  LIDAR-Lite I2C SCL (green) to Arduino SCL
  LIDAR-Lite I2C SDA (blue) to Arduino SDA
  LIDAR-Lite Ground (black) to Arduino GND

  (Capacitor recommended to mitigate inrush current when device is enabled)
  680uF capacitor (+) to Arduino 5v
  680uF capacitor (-) to Arduino GND

  See the Operation Manual for wiring diagrams and more information:
  http://static.garmin.com/pumac/LIDAR_Lite_v3_Operation_Manual_and_Technical_Specifications.pdf

------------------------------------------------------------------------------*/

#include <Wire.h>
#include <LIDARLite.h>

LIDARLite myLidarLite;

int Min;
int Max;
int Analog;
unsigned long Sum;
unsigned long Average;    //Should be equal to the bias.
  
void setup()
{ 
  Serial.begin(9600);
  myLidarLite.begin(0, true); // Set configuration to default and I2C to 400 kHz
  myLidarLite.configure(0);
  delay(100);       //"Stabilization time".   Probably not necessary
}
  
//Main Loop
void loop() 
{
Min = 1023;     //Initilize/reset to limit
Max = 0;        //Initilize/reset to limit
Sum = 0;        //Initialize/reset

//Take 1000 readings, find min, max, and average.  This loop takes about 100ms.
for (int i = 0; i < 100; i++)
{
  Analog = myLidarLite.distance(); 

  Sum = Sum + Analog;   //Sum for averaging
  
  if (Analog < Min)
    Min = Analog; 

  if (Analog > Max)
    Max = Analog;        
}

Average = (Sum/100);
          
// print results
Serial.print ( " Min = ");
Serial.print (Min);

Serial.print ( " Max = ");
Serial.print (Max);

Serial.print (" Average = ");
Serial.println (Average);
}
