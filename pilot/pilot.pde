#include <Wire.h>
#include <IMU3000.h>
#include <BMP085.h>
#include <LSM303DLH.h>
#include <FreeIMU.h>

char output[20];

unsigned long timer;
unsigned long timer_diff;
int timer_counter;

float timer_diffcounter;

FreeIMU myIMU = FreeIMU();

float angles[3];

void setup()
{
    Serial.begin(57600); 
    
    Wire.begin();

    delay(5);
    myIMU.init();
    delay(5);
    
    timer = micros();
    timer_counter = 0;
}

void loop()
{
    timer_diff = micros() - timer;
    timer_counter++;

    //myIMU.update();
    myIMU.getEuler(angles);
 
    if(timer_diff > 100000UL) {
      timer = micros();
      PrintSensors();
      timer_counter = 0;
    }

}

void PrintSensors() {
    
    timer_diffcounter = timer_diff / (float)timer_counter;
  
/*    float t = p_avg/101325.0;
    t = 1 - pow(t, 0.190295);
    float altitude = 44330 * t;
*/

    // Print out what we have
    /*
    sprintf(output, "%+09i, ", timer_diff);
    Serial.print(output);
    sprintf(output, "%+06i, ", timer_counter);
    Serial.print(output);
    Serial.print(timer_diffcounter, 2);
    Serial.print(", ");
    */
    
    Serial.print(timer_counter);
    Serial.print("; ");
    Serial.print(angles[0]);
    Serial.print(", ");
    Serial.print(angles[1]);
    Serial.print(", ");
    Serial.print(angles[2]);
    Serial.print("; ");
    
    
    Serial.print((myIMU.imu.g.x + myIMU.g_bias.x) * myIMU.g_scale.x);
    Serial.print(", ");
    Serial.print((myIMU.imu.g.y + myIMU.g_bias.y) * myIMU.g_scale.y);
    Serial.print(", ");
    Serial.print((myIMU.imu.g.z + myIMU.g_bias.z) * myIMU.g_scale.z);
    Serial.print("; ");

/*    
    Serial.print(timer_counter);
    Serial.print("; ");
    Serial.print(myIMU.m_max.x);
    Serial.print(", ");
    Serial.print(myIMU.m_max.y);
    Serial.print(", ");
    Serial.print(myIMU.m_max.z);
    Serial.print("; ");
    Serial.print(myIMU.m_min.x);
    Serial.print(", ");
    Serial.print(myIMU.m_min.y);
    Serial.print(", ");
    Serial.print(myIMU.m_min.z);
    Serial.print("; ");
    
    Serial.print(myIMU.magn.m.x);
    Serial.print(", ");
    Serial.print(myIMU.magn.m.y);
    Serial.print(", ");
    Serial.print(myIMU.magn.m.z);
    Serial.print("; ");

    Serial.print(myIMU.g_bias.x);
    Serial.print(", ");
    Serial.print(myIMU.g_bias.y);
    Serial.print(", ");
    Serial.print(myIMU.g_bias.z);
    Serial.print("; ");

    Serial.print(myIMU.imu.g.x);
    Serial.print(", ");
    Serial.print(myIMU.imu.g.y);
    Serial.print(", ");
    Serial.print(myIMU.imu.g.z);
    Serial.print(", ");
*/

/*    Serial.print(imu.a.x);
    Serial.print(", ");
    Serial.print(imu.a.y);
    Serial.print(", ");
    Serial.print(imu.a.z);
    Serial.print("; ");

    Serial.print(compass.a.x);
    Serial.print(", ");
    Serial.print(compass.a.y);
    Serial.print(", ");
    Serial.print(compass.a.z);
    Serial.print("; ");
    

*/    

    Serial.println("");     // prints carriage return
    
}

