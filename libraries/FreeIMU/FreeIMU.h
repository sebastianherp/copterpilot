/*
FreeIMU.h - A libre and easy to use orientation sensing library for Arduino
Copyright (C) 2011 Fabio Varesano <fabio at varesano dot net>


This program is free software: you can redistribute it and/or modify
it under the terms of the version 3 GNU General Public License as
published by the Free Software Foundation.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <Wire.h>
#include "WProgram.h"

#include <IMU3000.h>
#include <BMP085.h>
#include <LSM303DLH.h>
#include <vector.h>


#ifndef FreeIMU_h
#define FreeIMU_h

#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif

class FreeIMU
{
  public:
    FreeIMU();
    void init();
    void init(bool fastmode);
    void getRawValues(int * raw_values);
    void update();
    void getQ(float * q);
    void getEuler(float * angles);
    void getYawPitchRoll(float * ypr);
	void zeroCalibrateGyro(unsigned int totSamples, unsigned int sampleDelayMS);
    
	BMP085 baro;
    LSM303DLH magn;
    IMU3000 imu;
	
  float vx, vy, vz, wx, wy, wz;
    vector g_bias;
	vector g_scale;
	float gxAdj, gyAdj, gzAdj;
    float q0, q1, q2, q3; // quaternion elements representing the estimated orientation
	
  private:
    int* raw_acc, raw_gyro, raw_magn;
    void AHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
    float iq0, iq1, iq2, iq3;
    float exInt, eyInt, ezInt;  // scaled integral error
    long lastUpdate, now; // sample period expressed in milliseconds
    float halfT; // half the sample period expressed in seconds
    int startLoopTime;
	
	float maxOfVector(vector v);
	
};

float invSqrt(float number);

#endif // FreeIMU_h

