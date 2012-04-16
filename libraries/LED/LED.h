/*
LED.h - Library to measure A and V of LiPo
Copyright (C) 2011 Sebastian Herp
*/

#include "Arduino.h"

#ifndef LED_h
#define LED_h

//
// constants
//
#define PWM_FREQ 100 // hz
#define PWM_STEPS 128 // PWM resolution - set to 256 when using bam
#define BAM_STEPS 8 // how many times BAM updates are done each cycle
#define NUM_SHIFT_REGS 1  
#define NUM_CHANNELS NUM_SHIFT_REGS*8 // eigentlich 8
#define NUM_COLORS 21

class LED
{
public:
    LED();
	void setAllChannelsTo(int duty);
	void setChannel(int ch,int duty);
	void iUpdateBAM3();
	
private:
	char chalf;
	//Pin connected to latch pin (ST_CP) of 74HC595
	int latchPin;
	//Pin connected to clock pin (SH_CP) of 74HC595
	int clockPin;
	////Pin connected to Data in (DS) of 74HC595
	int dataPin;
	////Pin connected to Output Enable (GND = enabled)
	int outputEnablePin;
	// interrupt counter
	volatile int ticker;
	volatile byte odd;	

	volatile byte pwmValues[NUM_CHANNELS];
	volatile byte bamLookup[BAM_STEPS*NUM_SHIFT_REGS]; // precalculate the bytes to send every time the PWM values change
	volatile byte colors[NUM_COLORS][3];

    void precalcBamBytes();
	void prepareColors();
	void setup_lights();
	void sendSPI(volatile byte *valuesToSend, int from, int to);

};

#endif