#include <Wire.h>
#include <SPI.h>
#include <IMU3000.h>
#include <BMP085.h>
#include <LSM303DLH.h>
#include <vector.h>
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

float time_diff;

unsigned long timer_now = 0;
unsigned long timer_old;
unsigned long timer_diff;
int timer_counter = 0;
unsigned long timer_start = 0;
unsigned long timer_start2 = 0;
unsigned long timer_counter100 = 0, timer_counter50 = 0, timer_counter10 = 0, timer_counter5 = 0;
unsigned long ltimer_counter100 = 0, ltimer_counter50 = 0, ltimer_counter10 = 0, ltimer_counter5 = 0;

unsigned long maxTime1 = 0, maxTime2 = 0;

unsigned long cycleTime = 0;
unsigned long lastCycle = 0;
unsigned long lastTime = 0;
unsigned long lastMax = 0;

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
    Serial.begin(BAUD); 
    Wire.begin();

#ifdef DEBUG
    Serial.println("Init pilot");
    Serial.println("...");
#endif

    // initialize pilot
    pilot.init();
    
    // receiver works with the pin change interrupt
    attachInterrupt(0, receiver_update, RISING);

    timer_old = micros();
    timer_counter = 0;
    cycleTime = 0;
    
#ifdef DEBUG
    Serial.println("Setup done");
#endif
    
}

byte light_i = 0;
int buzzer_odd = 0;
byte light_values = 0;

unsigned long countme = 0;

void loop()
{
    
    timer_now = micros();
    timer_diff = timer_now - timer_old;

    /* 200 Hz base timing */
    if(timer_diff >= 5000) {
      timer_counter++;
      timer_old += 5000;
      
      if(timer_counter % 2 == 0) { // 100Hz
        timer_start = micros();
        pilot.imu_update();
        pilot.adjustMotorsPlus();
        timer_counter100 += micros() - timer_start;
      }
      
      if(timer_counter % 4 == 1) { // 50Hz
        timer_counter50++;
        pilot.processInput();
      }
      
      if(timer_counter % 20 == 3) { // 10Hz
        timer_counter10++;
        pilot.power_update();
      }

      if(timer_counter % 40 == 7) { // 5Hz
        timer_counter5++;
        telemetry();
      }
      maxTime1 = micros() - timer_now;
      if(maxTime1 > maxTime2) maxTime2 = maxTime1;
      cycleTime += maxTime1;
    }
    if(timer_counter >= 200) {
      lastMax = maxTime2;
      maxTime1 = maxTime2 = 0;
      timer_counter = 0;
      ltimer_counter100 = timer_counter100;
      timer_counter = timer_counter100 = timer_counter50 = timer_counter10 = timer_counter5 = 0;
      countme = 0;
      lastTime = micros() - timer_start2;
      timer_start2 = micros();
      lastCycle = cycleTime;
      cycleTime = 0;
    }
}

void telemetry() {
    Serial.print(timer_now / 1000);
    Serial.print(", ");        
    Serial.print(lastCycle);
    Serial.print(", ");        
    Serial.print(ltimer_counter100);
    Serial.print(", ");        
    Serial.print(lastTime);
    Serial.print(", ");        
    Serial.print(lastMax);
    Serial.print(", ");        
    Serial.print(pilot.voltage);
    Serial.print("V, ");        
    Serial.print(pilot.ampere);
    Serial.print("A, "); 
    Serial.print(pilot.ampere_hours, 0);
    Serial.print("mAh, "); 
    /*
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
    */
    
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



