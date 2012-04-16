#include <vector.h>
#include "Arduino.h"

#ifndef IMU3000_h
#define IMU3000_h

#define IMUGYRO 0x68         // gyro I2C address
#define REG_GYRO_TEMP 0x1B   // IMU-3000 Register address for GYRO_TEMP_H
#define IMUACCEL 0x53        // Accel I2c Address
#define ADXL345_POWER_CTL 0x2D

class IMU3000
{
public:
	vector a;
	vector g;
	float temp;
	
	IMU3000();
	void init();
	void read();
private:
	void readImu(int* tgxgygzaxayaz);
	void readImu(int* t, int* gx, int* gy, int* gz, int* ax, int* ay, int* az);
	void writeTo(byte device, byte address, byte val);
	void readFrom(byte address, int num, byte buff[]);
	byte _buff[14] ;    //14 bytes buffer for saving data read from the device
	int imuval[7];
	

};

#endif