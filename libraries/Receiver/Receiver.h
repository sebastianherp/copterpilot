/*
Receiver.h - Library to measure A and V of LiPo
Copyright (C) 2011 Sebastian Herp
*/

#include "WProgram.h"

#ifndef Receiver_h
#define Receiver_h

class Receiver
{
public:
    Receiver();
    void init();
    void update();  
 	volatile unsigned int rx_values[8];   
	
private:
	char rx_channel;
	unsigned int rx_start;
	unsigned int rx_duration;

};

#endif