/*
FreeIMU.cpp - A libre and easy to use orientation sensing library for Arduino
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

#include <inttypes.h>
#include "WProgram.h"
#include "FreeIMU.h"
#include <IMU3000.h>
#include <BMP085.h>
#include <LSM303DLH.h>

//----------------------------------------------------------------------------------------------------
// Definitions

#define KpAcc 0.0 //2.0f   // proportional gain governs rate of convergence to accelerometer/magnetometer
#define KiAcc 0.0 //0.0005f   // integral gain governs rate of convergence of gyroscope biases

#define KpMag 0.0 //2.0f   // proportional gain governs rate of convergence to accelerometer/magnetometer
#define KiMag 0.0 //0.005f   // integral gain governs rate of convergence of gyroscope biases

//#define halfT 0.02f   // half the sample period


FreeIMU::FreeIMU() {
	baro = BMP085();
	imu = IMU3000();
	magn = LSM303DLH();

	// initialize quaternion
	q0 = 1.0;
	q1 = 0.0;
	q2 = 0.0;
	q3 = 0.0;
	exInt = 0.0;
	eyInt = 0.0;
	ezInt = 0.0;
	lastUpdate = 0;
	now = 0;
	
}

void FreeIMU::init() {
  init(false);
}

void FreeIMU::init(bool fastmode) {
  delay(5);
  
  /*
  if(fastmode) { // switch to 400KHz I2C - eheheh
    TWBR = ((16000000L / 400000L) - 16) / 2; // see twi_init in Wire/utility/twi.c
    // TODO: make the above usable also for 8MHz arduinos..
  } */

  // init LSM303DLH
  magn.init();
  
  // init BMP085
  baro.init();
  
  // init IMU3000
  imu.init();
  // calibrate the IMU3000
  // TODO: imu.zeroCalibrate(64,5);
  zeroCalibrateGyro(164,5);
  
}


void FreeIMU::getRawValues(int * raw_values) {
  /*acc.readAccel(&raw_values[0], &raw_values[1], &raw_values[2]);
  gyro.readGyroRaw(&raw_values[3], &raw_values[4], &raw_values[5]);
  magn.getValues(&raw_values[6], &raw_values[7], &raw_values[8]);*/
}

void FreeIMU::zeroCalibrateGyro(unsigned int totSamples, unsigned int sampleDelayMS) {
	int xyz[3]; 
  float tmpOffsets[] = {0,0,0};

  for (int i = 0;i < totSamples;i++){
    delay(sampleDelayMS);
	imu.read();
    tmpOffsets[0] += imu.g.x;
    tmpOffsets[1] += imu.g.y;
    tmpOffsets[2] += imu.g.z;  
  }
  g_bias.x = -tmpOffsets[0] / totSamples;
  g_bias.y = -tmpOffsets[1] / totSamples;
  g_bias.z = -tmpOffsets[2] / totSamples;
  
  g_scale.x = 2000.0 / 32768.0 * PI / 180.0;
  g_scale.y = 2000.0 / 32768.0 * PI / 180.0;
  g_scale.z = 2000.0 / 32768.0 * PI / 180.0;
}

void FreeIMU::update() {  
  imu.read();
  //baro.read();
  //magn.read();
  
 
}

float FreeIMU::maxOfVector(vector v) {
	if(v.y > v.x) {
		if(v.z > v.x)
			return v.z;
		return v.y;
	}
	
	if(v.z > v.x)
		return v.z;
	
	return v.x;
}


//=====================================================================================================
// AHRS.c
// S.O.H. Madgwick
// 25th August 2010
//=====================================================================================================
// Description:
//
// Quaternion implementation of the 'DCM filter' [Mayhony et al].  Incorporates the magnetic distortion
// compensation algorithms from my filter [Madgwick] which eliminates the need for a reference
// direction of flux (bx bz) to be predefined and limits the effect of magnetic distortions to yaw
// axis only.
//
// User must define 'halfT' as the (sample period / 2), and the filter gains 'Kp' and 'Ki'.
//
// Global variables 'q0', 'q1', 'q2', 'q3' are the quaternion elements representing the estimated
// orientation.  See my report for an overview of the use of quaternions in this application.
//
// User must call 'AHRSupdate()' every sample period and parse calibrated gyroscope ('gx', 'gy', 'gz'),
// accelerometer ('ax', 'ay', 'ay') and magnetometer ('mx', 'my', 'mz') data.  Gyroscope units are
// radians/second, accelerometer and magnetometer units are irrelevant as the vector is normalised.
//
//=====================================================================================================
void FreeIMU::AHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz) {
  float norm;
  float hx, hy, hz, bx, bz;
  float exAcc, eyAcc, ezAcc;
  float exMag, eyMag, ezMag;

  // auxiliary variables to reduce number of repeated operations
  float q0q0 = q0*q0;
  float q0q1 = q0*q1;
  float q0q2 = q0*q2;
  float q0q3 = q0*q3;
  float q1q1 = q1*q1;
  float q1q2 = q1*q2;
  float q1q3 = q1*q3;
  float q2q2 = q2*q2;   
  float q2q3 = q2*q3;
  float q3q3 = q3*q3;          
  
  // normalise the measurements
  
  now = micros();
  halfT = (now - lastUpdate) / 2000000.0;
  lastUpdate = now;
  
  norm = sqrt(ax*ax + ay*ay + az*az);       
  ax = ax / norm;
  ay = ay / norm;
  az = az / norm;
  
  /*
  norm = invSqrt(ax*ax + ay*ay + az*az);
  ax = ax * norm;
  ay = ay * norm;
  az = az * norm;
  */
  
  norm = sqrt(mx*mx + my*my + mz*mz);          
  mx = mx / norm;
  my = my / norm;
  mz = mz / norm;
  
  /*
  norm = invSqrt(mx*mx + my*my + mz*mz);
  mx = mx * norm;
  my = my * norm;
  mz = mz * norm;
  */
  
  // compute reference direction of flux
  hx = 2*mx*(0.5 - q2q2 - q3q3) + 2*my*(q1q2 - q0q3) + 2*mz*(q1q3 + q0q2);
  hy = 2*mx*(q1q2 + q0q3) + 2*my*(0.5 - q1q1 - q3q3) + 2*mz*(q2q3 - q0q1);
  hz = 2*mx*(q1q3 - q0q2) + 2*my*(q2q3 + q0q1) + 2*mz*(0.5 - q1q1 - q2q2);         
  bx = sqrt((hx*hx) + (hy*hy));
  bz = hz;     
  
  // estimated direction of gravity and flux (v and w)
  vx = 2*(q1q3 - q0q2);
  vy = 2*(q0q1 + q2q3);
  vz = q0q0 - q1q1 - q2q2 + q3q3;
  wx = 2*bx*(0.5 - q2q2 - q3q3) + 2*bz*(q1q3 - q0q2);
  wy = 2*bx*(q1q2 - q0q3) + 2*bz*(q0q1 + q2q3);
  wz = 2*bx*(q0q2 + q1q3) + 2*bz*(0.5 - q1q1 - q2q2);  
  
  // error is sum of cross product between reference direction of fields and direction measured by sensors
  exAcc = (ay*vz - az*vy);
  eyAcc = (az*vx - ax*vz);
  ezAcc = (ax*vy - ay*vx); 
  
  exMag = (my*wz - mz*wy);
  eyMag = (mz*wx - mx*wz);
  ezMag = (mx*wy - my*wx);
  
  // integral error scaled integral gain
  exInt = exInt + exAcc*KiAcc + exMag*KiMag;
  eyInt = eyInt + eyAcc*KiAcc + eyMag*KiMag;
  ezInt = ezInt + ezAcc*KiAcc + ezMag*KiMag;
  
  // adjusted gyroscope measurements
  gxAdj = gx + KpAcc*exAcc + KpMag*exMag + exInt;
  gyAdj = gy + KpAcc*eyAcc + KpMag*eyMag + eyInt;
  gzAdj = gz + KpAcc*ezAcc + KpMag*ezMag + ezInt;
  
  // integrate quaternion rate and normalise
  iq0 = (-q1*gxAdj - q2*gyAdj - q3*gzAdj)*halfT;
  iq1 = (q0*gxAdj + q2*gzAdj - q3*gyAdj)*halfT;
  iq2 = (q0*gyAdj - q1*gzAdj + q3*gxAdj)*halfT;
  iq3 = (q0*gzAdj + q1*gyAdj - q2*gxAdj)*halfT;  
  
  q0 += iq0;
  q1 += iq1;
  q2 += iq2;
  q3 += iq3;
  
  // normalise quaternion
  
  norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
  q0 = q0 / norm;
  q1 = q1 / norm;
  q2 = q2 / norm;
  q3 = q3 / norm;
  
/*
  norm = invSqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
  q0 = q0 * norm;
  q1 = q1 * norm;
  q2 = q2 * norm;
  q3 = q3 * norm;
  */
}

void FreeIMU::getQ(float * q) {
  update();
  
  //input TODO: fix it!
  //float max = maxOfVector(m_max);
  //float min = -maxOfVector((vector){m_min.x, m_min.y, m_min.z});
  
  float gx = (imu.g.x + g_bias.x) * g_scale.x;
  float gy = (imu.g.y + g_bias.y) * g_scale.y;
  float gz = (imu.g.z + g_bias.z) * g_scale.z;
  float ax = imu.a.x;
  float ay = imu.a.y;
  float az = imu.a.z;
  float mx = (magn.m.x - magn.m_min.x);
  mx = mx / (magn.m_max.x - magn.m_min.x) * 2 - 1.0;
  float my = (magn.m.y - magn.m_min.y);
  my = my / (magn.m_max.y - magn.m_min.y) * 2 - 1.0;
  float mz = (magn.m.z - magn.m_min.z);
  mz = mz / (magn.m_max.z - magn.m_min.z) * 2 - 1.0;
  
  AHRSupdate(gx, gy, gz, ax, ay, az, magn.m.x, magn.m.y, magn.m.z);

  q[0] = q0;
  q[1] = q1;
  q[2] = q2;
  q[3] = q3;
}

// Returns the Euler angles in radians defined with the Aerospace sequence.
// See Sebastian O.H. Madwick report 
// "An efficient orientation filter for inertial and intertial/magnetic sensor arrays" Chapter 2 Quaternion representation
void FreeIMU::getEuler(float * angles) {
  float q[4]; // quaternion
  getQ(q);
  angles[0] = atan2(2 * q[1] * q[2] - 2 * q[0] * q[3], 2 * q[0]*q[0] + 2 * q[1] * q[1] - 1) * 180/M_PI; // psi
  angles[1] = -asin(2 * q[1] * q[3] + 2 * q[0] * q[2]) * 180/M_PI; // theta
  angles[2] = atan2(2 * q[2] * q[3] - 2 * q[0] * q[1], 2 * q[0] * q[0] + 2 * q[3] * q[3] - 1) * 180/M_PI; // phi
}



void FreeIMU::getYawPitchRoll(float * ypr) {
  float q[4]; // quaternion
  float gx, gy, gz; // estimated gravity direction
  getQ(q);
  
  gx = 2 * (q[1]*q[3] - q[0]*q[2]);
  gy = 2 * (q[0]*q[1] + q[2]*q[3]);
  gz = q[0]*q[0] - q[1]*q[1] - q[2]*q[2] + q[3]*q[3];
  
  ypr[0] = atan2(2 * q[1] * q[2] - 2 * q[0] * q[3], 2 * q[0]*q[0] + 2 * q[1] * q[1] - 1) * 180/M_PI;
  ypr[1] = atan(gx / sqrt(gy*gy + gz*gz))  * 180/M_PI;
  ypr[2] = atan(gy / sqrt(gx*gx + gz*gz))  * 180/M_PI;
}


float invSqrt(float number) {
  volatile long i;
  volatile float x, y;
  volatile const float f = 1.5F;

  x = number * 0.5F;
  y = number;
  i = * ( long * ) &y;
  i = 0x5f375a86 - ( i >> 1 );
  y = * ( float * ) &i;
  y = y * ( f - ( x * y * y ) );
  return y;
}
