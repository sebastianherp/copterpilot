#include "Arduino.h"

#ifndef BMP085_h
#define BMP085_h

#define BMP085_ADDR 0x77 	// barometer I2C address


/* from http://code.google.com/p/bmp085driver/source/browse/trunk/BMP085.h */
#define CAL_AC1           0xAA  // R   Calibration data (16 bits)
#define CAL_AC2           0xAC  // R   Calibration data (16 bits)
#define CAL_AC3           0xAE  // R   Calibration data (16 bits)    
#define CAL_AC4           0xB0  // R   Calibration data (16 bits)
#define CAL_AC5           0xB2  // R   Calibration data (16 bits)
#define CAL_AC6           0xB4  // R   Calibration data (16 bits)
#define CAL_B1            0xB6  // R   Calibration data (16 bits)
#define CAL_B2            0xB8  // R   Calibration data (16 bits)
#define CAL_MB            0xBA  // R   Calibration data (16 bits)
#define CAL_MC            0xBC  // R   Calibration data (16 bits)
#define CAL_MD            0xBE  // R   Calibration data (16 bits)
#define CONTROL           0xF4  // W   Control register 
#define CONTROL_OUTPUT    0xF6  // R   Output registers 0xF6=MSB, 0xF7=LSB, 0xF8=XLSB

// BMP085 Modes
#define MODE_ULTRA_LOW_POWER    0 //oversampling=0, internalsamples=1, maxconvtimepressure=4.5ms, avgcurrent=3uA, RMSnoise_hPA=0.06, RMSnoise_m=0.5
#define MODE_STANDARD           1 //oversampling=1, internalsamples=2, maxconvtimepressure=7.5ms, avgcurrent=5uA, RMSnoise_hPA=0.05, RMSnoise_m=0.4
#define MODE_HIGHRES            2 //oversampling=2, internalsamples=4, maxconvtimepressure=13.5ms, avgcurrent=7uA, RMSnoise_hPA=0.04, RMSnoise_m=0.3
#define MODE_ULTRA_HIGHRES      3 //oversampling=3, internalsamples=8, maxconvtimepressure=25.5ms, avgcurrent=12uA, RMSnoise_hPA=0.03, RMSnoise_m=0.25
                  // "Sampling rate can be increased to 128 samples per second (standard mode) for
                  // dynamic measurement.In this case it is sufficient to measure temperature only 
                  // once per second and to use this value for all pressure measurements during period."
                  // (from BMP085 datasheet Rev1.2 page 10).
                  // To use dynamic measurement set AUTO_UPDATE_TEMPERATURE to false and
                  // call calcTrueTemperature() from your code. 
// Control register
#define READ_TEMPERATURE        0x2E 
#define READ_PRESSURE           0x34 

#define TEMP_MEASURE_CYCLES	10

class BMP085
{
public:
	float temp;
	long pressure;
	long ut;
	long up;
	
	BMP085();
	void init();
	void init(byte mode);
	void read();

private:
	void readCaldata();
	void sampleTemperature();
	void readTemperature();
	void samplePressure();
	void readPressure();

	void writeTo(byte device, byte address, byte val);
	void readFrom(byte address, int num, byte buff[]);

	int ac1, ac2, ac3, b1, b2, mb, mc, md;
	unsigned int ac4, ac5, ac6;
	long b5;
	
	
	long temperatureWait;
	long pressureWait;
	long temperatureSamplingStartMicros;
	long pressureSamplingStartMicros;

	byte tempCounter;
	
	byte oversampling;
	byte _buff[3] ;    //3 bytes buffer for saving data read from the device
};

#endif