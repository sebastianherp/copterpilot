#include <Wire.h>
#include <SPI.h>
#include <IMU3000.h>
#include <BMP085.h>
#include "vector.h"
#include "config.h"
#include "classes.h"

#define DEBUG 1

#define ON 1
#define OFF 0

#define ACRO 0
#define STABLE 1

#define BEEPER_ON false

char output[20];

Pilot pilot = Pilot();

byte armed = OFF;
byte safetyCheck = OFF;

static unsigned long timer_now = 0;
static unsigned long timer_old = 0;
static unsigned long cycleTime = 0;
static unsigned long cycleTimeMax = 0;
static unsigned long cycleTimeMin = 999999;
static unsigned int cycleCounter = 0;

ISR(TIMER1_COMPA_vect) {
    pilot._motors.handleInterrupt();
}

void receiver_update() {
    pilot.receiver_update(); 
}


/**
* Setup Quadrocopter
*/
void setup()
{
    SerialOpen(0,BAUD); 
    Wire.begin();

    // initialize pilot
    pilot.init();
    
    // receiver works with the pin change interrupt
    attachInterrupt(0, receiver_update, RISING);

    timer_old = timer_now = micros();

    SerialWrite(0,'s');
    SerialWrite(0,'t');
    SerialWrite(0,'a');
    SerialWrite(0,'r');
    SerialWrite(0,'t');
    SerialWrite(0,'\n');     
}

byte light_i = 0;
int buzzer_odd = 0;
byte light_values = 0;

unsigned long countme = 0;

void loop()
{
    static unsigned long scheduler50Hz = 0;
    static unsigned long scheduler10Hz = 0;    
    static unsigned long scheduler1Hz = 0;    
    static byte timer_counter=0;
    static unsigned int cycle_counter = 0;

    /* 50 Hz stuff */
    if(timer_now > scheduler50Hz) {
      scheduler50Hz = timer_now + 20000;
      pilot.processInput();
    }
    
    /* 10 Hz stuff */
    if(timer_now > scheduler10Hz) {
      scheduler10Hz = timer_now + 100000;
      timer_counter++;
      switch(timer_counter) {
        case 1: 
          pilot.power_update();
          break;
        case 2:
          pilot.mag_update();
          break;
        case 3:
          pilot.alt_update();
          break;        
        default:
          timer_counter = 0;
          break;
      }
//      telemetry();
    }
    
    /* 1 Hz stuff */
    if(timer_now > scheduler1Hz) {
      scheduler1Hz = timer_now + 1000000;
      cycleCounter = cycle_counter;
      cycle_counter = 0;
      /*
          SerialWrite(0,'+');
    SerialWriteInt(0, pilot._imu.imu.g[0]);
    SerialWrite(0,',');
    SerialWrite(0,' ');
    SerialWriteInt(0, pilot._imu.imu.g[1]);
    SerialWrite(0,',');
    SerialWrite(0,' ');
    SerialWriteInt(0, pilot._imu.imu.g[2]);
    SerialWrite(0,'\n'); */
    }
    

    // as fast as possible
    pilot.imu_update();
       
    
    timer_now = micros();
    cycleTime = timer_now - timer_old;
    timer_old = timer_now;
    cycle_counter++;
    
    if(cycleTimeMax<cycleTime) cycleTimeMax = cycleTime;
    if(cycleTimeMin>cycleTime) cycleTimeMin = cycleTime;

    pilot.adjustMotorsPlus();

    serialCom();
}

/*
void telemetry() {
    Serial.print(timer_now / 1000);
    Serial.print(", ");        
    Serial.print(cycleTime);
    Serial.print(", ");        
    Serial.print(cycleTimeMax);
    Serial.print(", ");        
    Serial.print(cycleTimeMin);
    Serial.print(", ");        
    Serial.print(cycleCounter);
    Serial.print(", ");        
    Serial.print(pilot.voltage);
    Serial.print("V, ");        
    Serial.print(pilot.ampere);
    Serial.print("A, "); 
    Serial.print(pilot.ampere_hours, 0);
    Serial.print("mAh, "); 

    Serial.print(myIMU.imu.a.x);
    Serial.print(", ");
    Serial.print(myIMU.imu.a.y);
    Serial.print(", ");
    Serial.print(myIMU.imu.a.z);
    Serial.print("| ");
    Serial.print(myIMU.imu.g.x);
    Serial.print(", ");
    Serial.print(myIMU.imu.g.y);
    Serial.print(", ");
    Serial.print(myIMU.imu.g.z);
    Serial.print("| ");
    
    for(int i=0;i<8;i++) {
      Serial.print(pilot._receiver.get(i));
      Serial.print(", ");
    }
    
    
    Serial.print(pilot._motors.getCalculated(FRONT));
    Serial.print(", ");
    Serial.print(pilot._motors.getCalculated(RIGHT));
    Serial.print(", ");
    Serial.print(pilot._motors.getCalculated(REAR));
    Serial.print(", ");
    Serial.print(pilot._motors.getCalculated(LEFT));
    Serial.print("| ");
    Serial.print(pilot._motors.get(FRONT));
    Serial.print(", ");
    Serial.print(pilot._motors.get(RIGHT));
    Serial.print(", ");
    Serial.print(pilot._motors.get(REAR));
    Serial.print(", ");
    Serial.print(pilot._motors.get(LEFT));
    
    Serial.println(""); 
    
}
*/


