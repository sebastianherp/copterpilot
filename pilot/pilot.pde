#include <Wire.h>
#include <SPI.h>
#include <IMU3000.h>
#include <BMP085.h>
#include <LSM303DLH.h>
#include <FreeIMU.h>
#include <Battery.h>
#include <Receiver.h>
#include <LED.h>
#include <Motors.h>



#define BEEPER_ON false

char output[20];

FreeIMU myIMU = FreeIMU();
Battery bat = Battery();
Motors motors = Motors(4, 5, 6, 7);
Receiver receiver = Receiver(2); // pin 2
//LED led = LED();


float quat[4];
float angles[3];
float angles2[3] = {0.0, 0.0, 0.0};
float time_diff;
float scale_factor;








unsigned long timer_now;
unsigned long timer_old;
unsigned long timer_diff;
int timer_counter;


ISR(TIMER1_COMPA_vect) {
    //motors.handleInterrupt();
}

void receiver_update() {
  receiver.update(); 
}

void setup()
{
   
    bat.init();
    
    motors.init();
    
    // Receiver stuff
    receiver.init();
    attachInterrupt(0, receiver_update, CHANGE);
  
    motors.set(FRONT, 2000);
    motors.set(REAR, 2200);    
    motors.set(LEFT, 2300);
    motors.set(RIGHT, 2400);
    
    Serial.begin(28800); 
    
    Wire.begin();

    delay(5);
    myIMU.init();
    delay(5);
    
    scale_factor = 180000000.0 / (myIMU.g_scale.x * PI);
    
    timer_old = micros();
    timer_counter = 0;
    
}

byte light_i = 0;
int buzzer_odd = 0;
byte light_values = 0;

unsigned long countme = 0;

void loop()
{
    timer_now = micros();
    timer_diff = timer_now - timer_old;

    //iUpdateBAM3();
  
    /* 200 Hz base timing */
    if(timer_diff >= 5000) {
      timer_counter++;
      timer_old += 5000;
        
      if(timer_counter % 2 == 0) { // 200Hz
        myIMU.update();
        myIMU.getYawPitchRoll(angles);
        //myIMU.getQ(quat);
      }
      
      if(timer_counter % 50 == 0) { // 1Hz
     
        //TODO: flight commands
        Serial.print(timer_now / 1000);
        Serial.print(", ");        
        Serial.print(digitalRead(2));
        Serial.print(", ");        
        Serial.print(bat.voltage);
        Serial.print("V, ");        
        Serial.print(bat.ampere);
        Serial.print("A, "); 
        Serial.print(bat.ampere_hours, 0);
        Serial.print("mAh, "); 
        Serial.print(receiver.get(0));
        Serial.print(", ");
        Serial.print(receiver.get(1));
        Serial.print(", ");
        Serial.print(receiver.get(2));
        Serial.print(", ");
        Serial.print(receiver.get(3));
        Serial.print("| ");
        Serial.print(motors.get(FRONT));
        Serial.print(", ");
        Serial.print(motors.get(RIGHT));
        Serial.print(", ");
        Serial.print(motors.get(BACK));
        Serial.print(", ");
        Serial.print(motors.get(LEFT));
        Serial.println("");          

      }
      
      if(timer_counter % 20 == 0) { // 10Hz
        bat.update();

        if(BEEPER_ON && bat.voltage < 10.0)
          tone(9, 400);
        else
          noTone(9);
        
        //PrintSensors();
  
      }

      
    }
    if(timer_counter >= 200) {
      timer_counter = 0;
      countme = 0;
    }
}




