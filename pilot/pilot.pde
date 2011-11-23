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


#define BAUD 115200
#define BUZZER_PIN 9
#define RECEIVER_PIN 2
#define MOTOR_PINS 4,5,6,7

#define ON 1
#define OFF 0

#define BEEPER_ON false

char output[20];

FreeIMU myIMU = FreeIMU();
Battery bat = Battery();
Motors motors = Motors(MOTOR_PINS);
Receiver receiver = Receiver(RECEIVER_PIN); // pin 2
//LED led = LED();


byte armed = OFF;
byte safetyCheck = OFF;


float quat[4];
float angles[3];
float angles2[3] = {0.0, 0.0, 0.0};
float time_diff;

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


/**
* Setup Quadrocopter
*/
void setup()
{
    Serial.begin(BAUD); 
    Wire.begin();

    // initialize motors
    motors.init();
    
    // initialize receiver
    receiver.init();
    attachInterrupt(0, receiver_update, CHANGE);
    
    // initialize sensors
    myIMU.init();

    // battery measurements
    bat.init();
  
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
        
      if(timer_counter % 2 == 0) { // 100Hz
        myIMU.update();
        myIMU.getYawPitchRoll(angles);
        //myIMU.getQ(quat);
        
        if(armed == ON) {
          motors.set(FRONT, receiver.get(THROTTLE));
          motors.set(REAR, receiver.get(THROTTLE));    
          motors.set(LEFT, receiver.get(THROTTLE));
          motors.set(RIGHT, receiver.get(THROTTLE));
        }
      }
      
      if(timer_counter % 4 == 0) { // 50Hz
        processInput();
      
      }
      
      if(timer_counter % 20 == 0) { // 10Hz
        bat.update();

        if(BEEPER_ON && bat.voltage < 10.0)
          tone(9, 400);
        else
          noTone(9);
        
        telemetry();
      }

      
    }
    if(timer_counter >= 200) {
      timer_counter = 0;
      countme = 0;
    }
}

void telemetry() {
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
    Serial.print(motors.get(REAR));
    Serial.print(", ");
    Serial.print(motors.get(LEFT));
    Serial.println(""); 
}

void processInput() {
  if(receiver.get(THROTTLE) < RX_LOW) {
    
    // disarm motors if left stick in lower left corner
    if(receiver.get(YAW) < RX_LOW && armed == ON) {
      armed = OFF;
      motors.setAll(MOTOR_OFF);
    }
    
    // TODO: calibration
    
    // arm motors if left stick in lower right corner
    if(receiver.get(YAW) > RX_HIGH && armed == OFF && safetyCheck == ON) {
      armed = ON;
      motors.setAll(MOTOR_ON);
    }
    
    if(receiver.get(YAW) > RX_LOW) safetyCheck = ON;
  }
  
}

