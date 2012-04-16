#include <Wire.h>
#include "BMP085.h"


BMP085::BMP085() {
	temperatureWait = 4500;
	pressureWait = 25500;
	temperatureSamplingStartMicros = -1;
	pressureSamplingStartMicros = -1;
	tempCounter = 0;
	
	pressure = -1;
	temp = -1;
}

void BMP085::init() {
	init(MODE_STANDARD);
}

void BMP085::init(byte mode) {
	long t;

	readCaldata();
	oversampling = mode;

	if(mode == MODE_ULTRA_LOW_POWER)
		pressureWait = 4500;
	else if(mode == MODE_STANDARD)
		pressureWait = 7500;
	else if(mode == MODE_HIGHRES)
		pressureWait = 13500;
	else 
		pressureWait = 25500;

	temperatureSamplingStartMicros = -1;
	pressureSamplingStartMicros = -1;

	// init b5 (temperature calibration)
	sampleTemperature();
	temperatureSamplingStartMicros = micros() + temperatureWait;
	while(temperatureSamplingStartMicros != -1 && micros() > temperatureSamplingStartMicros);
	readTemperature();
	temperatureSamplingStartMicros = -1;
}

void BMP085::read() {
	long t;

	if(tempCounter == TEMP_MEASURE_CYCLES) {
		if(temperatureSamplingStartMicros != -1) {
			if(micros() > temperatureSamplingStartMicros) {
				readTemperature();
				temperatureSamplingStartMicros = -1;
				tempCounter = 0;
			}
		} else {
			sampleTemperature();
			temperatureSamplingStartMicros = micros() + temperatureWait;
		}
	} else {
		if(pressureSamplingStartMicros != -1) {
			if(micros() > pressureSamplingStartMicros) {
				readPressure();
				pressureSamplingStartMicros = -1;
				tempCounter++;
			}
		} else {
			samplePressure();
			pressureSamplingStartMicros = micros() + pressureWait;
		}
	}
}

void BMP085::sampleTemperature() {
	writeTo(BMP085_ADDR, CONTROL, READ_TEMPERATURE);
}

void BMP085::readTemperature() {
	long x1,x2;

	readFrom(CONTROL_OUTPUT, 2, _buff);
	ut = ((long)_buff[0] << 8 | ((long)_buff[1]));

	x1 = ((long)ut - ac6) * ac5 >> 15;
	x2 = ((long)mc << 11) / (x1 + md);
	b5 = x1 + x2;
	temp = (( b5 + 8) >> 4) / 10.0;
}

void BMP085::samplePressure() {
	writeTo(BMP085_ADDR, CONTROL, READ_PRESSURE + (oversampling << 6));
}

void BMP085::readPressure() {
	long x1,x2,x3,b3,b6,p;
	unsigned long b4,b7;
	int32_t tmp;
	
	readFrom(CONTROL_OUTPUT, 3, _buff);
	up = ((((long)_buff[0] <<16) | ((long)_buff[1] <<8) | ((long)_buff[2])) >> (8-oversampling));
	
	b6 = b5 - 4000;  // b5 is updated by readTemperature().
	x1 = (b2* (b6 * b6 >> 12)) >> 11;
	x2 = ac2 * b6 >> 11;
	x3 = x1 + x2;
	tmp = ac1;
	tmp = (tmp * 4 + x3) << oversampling;
	b3 = (tmp + 2) >> 2;
	x1 = ac3 * b6 >> 13;
	x2 = (b1 * (b6 * b6 >> 12)) >> 16;
	x3 = ((x1 + x2) + 2) >> 2;
	b4 = (ac4 * (uint32_t) (x3 + 32768)) >> 15;
	b7 = ((uint32_t)up - b3) * (50000 >> oversampling);
	p = b7 < 0x80000000 ? (b7 << 1) / b4 : (b7 / b4) << 1;
	x1 = (p >> 8) * (p >> 8);
	x1 = (x1 * 3038) >> 16;
	x2 = (-7357 * p) >> 16;
	
	pressure = p + ((x1 + x2 + 3791) >> 4);	
}

void BMP085::readCaldata() {
	readFrom(CAL_AC1, 2, _buff);
	ac1 = ((int)_buff[0] <<8 | ((int)_buff[1]));
	readFrom(CAL_AC2, 2, _buff);
	ac2 = ((int)_buff[0] <<8 | ((int)_buff[1]));
	readFrom(CAL_AC3, 2, _buff);
	ac3 = ((int)_buff[0] <<8 | ((int)_buff[1]));
	readFrom(CAL_AC4, 2, _buff);
	ac4 = ((unsigned int)_buff[0] <<8 | ((unsigned int)_buff[1]));
	readFrom(CAL_AC5, 2, _buff);
	ac5 = ((unsigned int)_buff[0] <<8 | ((unsigned int)_buff[1]));
	readFrom(CAL_AC6, 2, _buff);
	ac6 = ((unsigned int)_buff[0] <<8 | ((unsigned int)_buff[1])); 
	readFrom(CAL_B1, 2, _buff);
	b1 = ((int)_buff[0] <<8 | ((int)_buff[1])); 
	readFrom(CAL_B2, 2, _buff);
	b2 = ((int)_buff[0] <<8 | ((int)_buff[1])); 
	readFrom(CAL_MB, 2, _buff);
	mb = ((int)_buff[0] <<8 | ((int)_buff[1]));
	readFrom(CAL_MC, 2, _buff);
	mc = ((int)_buff[0] <<8 | ((int)_buff[1]));
	readFrom(CAL_MD, 2, _buff);
	md = ((int)_buff[0] <<8 | ((int)_buff[1])); 
}


// Writes val to address register on device
void BMP085::writeTo(byte device, byte address, byte val) {
	Wire.beginTransmission(device); // start transmission to device
	Wire.write(address);             // send register address
	Wire.write(val);                 // send value to write
	Wire.endTransmission();         // end transmission
}

// Reads num bytes starting from address register on device in to _buff array
void BMP085::readFrom(byte address, int num, byte _buff[]) {
	Wire.beginTransmission(BMP085_ADDR); // start transmission to device
	Wire.write(address);             // sends address to read from
	Wire.endTransmission();         // end transmission

	Wire.beginTransmission(BMP085_ADDR); // start transmission to device
	Wire.requestFrom(BMP085_ADDR, num);    // request 6 bytes from device

	int i = 0;
	while(Wire.available())         // device may send less than requested (abnormal)
	{
		_buff[i] = Wire.read();    // receive a byte
		i++;
	}
	Wire.endTransmission();         // end transmission
}