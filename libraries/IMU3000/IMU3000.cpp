#include <Wire.h>
#include "IMU3000.h"
#include "WProgram.h"

IMU3000::IMU3000() {

/*
	a_gains[0] = 0.00376;
	a_gains[1] = 0.00376;
	a_gains[2] = 0.00346;

	a_offset[0] = 0;
	a_offset[1] = 0;
	a_offset[2] = 20;
*/
}

void IMU3000::init() {
	// Set Gyro settings
	// Sample Rate 100 Hz (the real one) 1kHz / (x + 1) ... 100 Hz with x = 9
	writeTo(IMUGYRO, 0x15, 0x09);  
	// Sample Rate 1kHz, Filter Bandwidth 42Hz, Gyro Range 500 d/s
	// 0x0B = 500 d/s, 0x03 = 250 d/s, 0x13 = 1000 d/s, 0x1B = 2000 d/s
	writeTo(IMUGYRO, 0x16, 0x1B);       
	//set accel register data address
	writeTo(IMUGYRO, 0x18, 0x32);     
	// set accel i2c slave address
	writeTo(IMUGYRO, 0x14, IMUACCEL);     
    
	// Set passthrough mode to Accel so we can turn it on
	writeTo(IMUGYRO, 0x3D, 0x08);     
	// set accel power control to 'measure'
	writeTo(IMUACCEL, ADXL345_POWER_CTL, 8);     
	writeTo(IMUACCEL, 0x31, 0x0B); // Full Resolution and 16 g scale
	//cancel pass through to accel, gyro will now read accel for us   
	writeTo(IMUGYRO, 0x3D, 0x28);
}

void IMU3000::read() {
	readImu(imuval);
	temp = (imuval[0] + 13200) / 280.0 + 35.0;	
	g.x = imuval[1];
	g.y = imuval[2];
	g.z = imuval[3];
	a.x = imuval[4];
	a.y = imuval[5];
	a.z = imuval[6];
}

void IMU3000::readImu(int *tgxgygzaxayaz) {
	readImu(tgxgygzaxayaz, tgxgygzaxayaz +1, tgxgygzaxayaz +2, tgxgygzaxayaz +3, tgxgygzaxayaz +4, tgxgygzaxayaz +5, tgxgygzaxayaz +6);
}

void IMU3000::readImu(int* t, int* gx, int* gy, int* gz, int* ax, int* ay, int* az) {
	readFrom(REG_GYRO_TEMP, 14, _buff);

	*t = _buff[0] << 8 | _buff[1];
	//Combine bytes into integers
	// Gyro format is MSB first
	*gx = -(_buff[2] << 8 | _buff[3]);
	*gy = -(_buff[4] << 8 | _buff[5]);
	*gz = _buff[6] << 8 | _buff[7];
	// Accel is LSB first. Also because of orientation of chips
	// accel y output is in same orientation as gyro x
	// and accel x is gyro -y
	*ax = _buff[9] << 8 | _buff[8];
	*ay = _buff[11] << 8 | _buff[10];
	*az = _buff[13] << 8 | _buff[12];  
}


// Writes val to address register on device
void IMU3000::writeTo(byte device, byte address, byte val) {
	Wire.beginTransmission(device); // start transmission to device
	Wire.send(address);             // send register address
	Wire.send(val);                 // send value to write
	Wire.endTransmission();         // end transmission
}

// Reads num bytes starting from address register on device in to _buff array
void IMU3000::readFrom(byte address, int num, byte _buff[]) {
	Wire.beginTransmission(IMUGYRO); // start transmission to device
	Wire.send(address);             // sends address to read from
	Wire.endTransmission();         // end transmission

	Wire.beginTransmission(IMUGYRO); // start transmission to device
	Wire.requestFrom(IMUGYRO, num);    // request 6 bytes from device

	int i = 0;
	while(Wire.available())         // device may send less than requested (abnormal)
	{
		_buff[i] = Wire.receive();    // receive a byte
		i++;
	}
	Wire.endTransmission();         // end transmission
}