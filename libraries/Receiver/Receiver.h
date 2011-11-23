/*
Receiver.h - Library to measure A and V of LiPo
Copyright (C) 2011 Sebastian Herp
*/

#include "WProgram.h"

#ifndef Receiver_h
#define Receiver_h

#define THROTTLE 2
#define YAW 3
#define ROLL 1
#define PITCH 0
#define LASTCHANNEL 8

#define RX_LOW 2200
#define RX_HIGH 3800

class Receiver
{
public:
    Receiver(int pin);
	void init();
    void update();  
	unsigned int get(char channel);
	volatile unsigned int rx_values[LASTCHANNEL];
	
private:
	int _pin;
	char rx_channel;
	unsigned int rx_start;
	unsigned int rx_duration;

};

#endif