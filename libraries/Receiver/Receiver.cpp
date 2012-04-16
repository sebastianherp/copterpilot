/*
Receiver.cpp - Receive commandos
Copyright (C) 2011 Sebastian Herp
*/

#include "Receiver.h"

Receiver::Receiver(int pin) {
	_pin = pin;
	rx_channel = 0;
	rx_duration = 0;
}

void Receiver::init() {
	TCCR1A = 0; // normal counting mode
    TCCR1B = 0 | (1<<CS11);// | (1<<CS10); // prescaler :8

    pinMode(_pin, INPUT);
	for(byte channel = 0; channel < LASTCHANNEL; channel++) {
		rx_values[channel] = 1000;
	}
}

unsigned int Receiver::get(char channel) {
	return rx_values[channel];
}

/***
* normal values are between 2000 and 4000 (1000 us - 2000 us)
*/
void Receiver::update() {
   if(!digitalRead(_pin)) {
     rx_start = TCNT1;
   } else {
     rx_duration = TCNT1 - rx_start;
     if(rx_duration < 0)
       rx_duration += 65536;
       
     if(rx_duration > 8000)
       rx_channel = 0;
     else {
	   rx_values[rx_channel] = rx_duration;
       //rx_values[rx_channel]  = (rx_values[rx_channel] * 3 + rx_duration) / 4;
       rx_channel++;
       if(rx_channel >= 8)
         rx_channel = 0;
     }
   }
}