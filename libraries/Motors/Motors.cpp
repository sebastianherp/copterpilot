/*
Motors.cpp - Library to measure A and V of LiPo
Copyright (C) 2011 Sebastian Herp
*/

#include "WProgram.h"
#include "Motors.h"

Motors::Motors(int pin1, int pin2, int pin3, int pin4) {
	_pin1 = pin1;
	_pin2 = pin2;
	_pin3 = pin3;
	_pin4 = pin4;
	
    pinMode(_pin1, OUTPUT);
    pinMode(_pin2, OUTPUT);
    pinMode(_pin3, OUTPUT);
    pinMode(_pin4, OUTPUT);
	
	setAll(MOTOR_OFF);
}

void Motors::init() {
	TCCR1A = 0; // normal counting mode
    TCCR1B = 0 | (1<<CS11); // | (1<<CS10); // prescaler :8
    TIMSK1 |= (1<<OCIE1A); // Enable CTC interrupt  

}

unsigned int Motors::get(char motor) {
	return motor_values[motor];
}

void Motors::set(char motor, unsigned int value) {
	motor_values[motor] = value;
}

void Motors::setAll(unsigned int value) {
	for(byte motor = 0; motor < LASTMOTOR; motor++) {
		motor_values[motor] = value;
	}
}

void Motors::handleInterrupt() {
	state = 0;
	if (state == 0) {
		//http://billgrundmann.wordpress.com/2009/03/03/to-use-or-not-use-writedigital/
		bitWrite(PORTD, 4, 1);
		OCR1A+= motor_values[0]; // 0.5 us increments
		state++ ;
	} else if (state == 1) {
		bitWrite(PORTD, 4, 0);
		bitWrite(PORTD, 5, 1);
		OCR1A+= motor_values[1]; // 1000 us
		state++;
	} else if (state == 2) {
		bitWrite(PORTD, 5, 0);
		bitWrite(PORTD, 6, 1);
		OCR1A+= motor_values[2]; // 1000 us
		state++;
	} else if (state == 3) {
		bitWrite(PORTD, 6, 0);
		bitWrite(PORTD, 7, 1);
		OCR1A+= motor_values[3]; // 1000 us
		state++;
	} else if (state == 4) {
		bitWrite(PORTD, 7, 0);
		count = 10; // 12 x 1000 us
		state++;
	} else if (state == 5) {
		if (count > 0) count--;
		else state = 0;
		OCR1A+= 2000; // 1000 us
	}
}


