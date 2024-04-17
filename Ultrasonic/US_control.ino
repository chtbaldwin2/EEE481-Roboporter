#include <Arduino.h>
#include <NewPing.h>
#include <stdarg.h>

#define SENSOR_NUM  4           
#define MAX_DISTANCE 300      
#define PING_INTERVAL 100            
unsigned long pingTimer[SENSOR_NUM]; 
unsigned int cm[SENSOR_NUM];        
uint8_t currentSensor = 0;

NewPing sonar[SENSOR_NUM] =         
{                                    
  NewPing(2, 3, MAX_DISTANCE),
  NewPing(4, 5, MAX_DISTANCE),
  NewPing(6, 7, MAX_DISTANCE),
  NewPing(8, 9, MAX_DISTANCE),
};

void echoCheck()                                
{                                    
  if (sonar[currentSensor].check_timer())
    cm[currentSensor] = sonar[currentSensor].ping_result / US_ROUNDTRIP_CM;
}

void oneSensorCycle()                           
{ 
  for (uint8_t i = 0; i < SENSOR_NUM; i++) {
    Serial.print("%");
    Serial.print(i);
    Serial.print(": ");
    Serial.println(cm[i], DEC);                    
  }
}

void setup() {
  
  Serial.begin(9600);

  pingTimer[0] = millis() + 75;                     
  for (uint8_t i = 1; i < SENSOR_NUM; i++)           
    pingTimer[i] = pingTimer[i - 1] + PING_INTERVAL;

}

void loop() {

  for (uint8_t i = 0; i < SENSOR_NUM; i++) {                            
    if (millis() >= pingTimer[i]) {                                     
      pingTimer[i] += PING_INTERVAL * SENSOR_NUM;                       
      if (i == 0 && currentSensor == SENSOR_NUM - 1) oneSensorCycle();  
      sonar[currentSensor].timer_stop();                             
      currentSensor = i;                                             
      cm[currentSensor] = 300;                                        
      sonar[currentSensor].ping_timer(echoCheck);                       
    }
  }

}
