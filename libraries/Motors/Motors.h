/*
Motors.h - Library to measure A and V of LiPo
Copyright (C) 2011 Sebastian Herp
*/

#include "WProgram.h"

#ifndef Motors_h
#define Motors_h

#define FRONT 0
#define RIGHT 1
#define LEFT 2
#define REAR 3
#define LASTMOTOR 4

#define MOTOR_LOW 2000
#define MOTOR_HIGH 4000
#define MOTOR_OFF 200
#define MOTOR_ON MOTOR_LOW + 200

class Motors
{
public:
    Motors(int pin1, int pin2, int pin3, int pin4);
	void init();
	void handleInterrupt();
	void setYaw(int value) { _yaw = value; };
	void setRoll(int value) { _roll = value; };
	void setPitch(int value) { _pitch = value; };
	void setThrottle(float value) { _throttle = value; };
	
	void process(float * ypr);
	void write();
	
	unsigned int getCalculated(char motor) { return motor_calculated_values[motor]; };
	unsigned int get(char motor) { return motor_values[motor]; };
	void setAll(unsigned int value);
	
private:
	int _pin1, _pin2, _pin3, _pin4;
	int _yaw, _pitch, _roll;
	float _throttle;
	

	unsigned int motor_calculated_values[LASTMOTOR];  
	unsigned int motor_min_values[LASTMOTOR];  
 	unsigned int motor_max_values[LASTMOTOR];   
 	volatile unsigned int motor_values[LASTMOTOR];   

	void set(char motor, unsigned int value) { motor_values[motor] = value; };


};

#endif