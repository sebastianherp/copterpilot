/*
Receiver.cpp - Library to measure A and V of LiPo
Copyright (C) 2011 Sebastian Herp
*/

#include "WProgram.h"
#include "Receiver.h"

Receiver::Receiver() {

}

void Receiver::init() {
	rx_channel = 0;
	rx_duration = 0;
	
    pinMode(2, INPUT);
	
	rx_values[0] = 1234;
}

void Receiver::update() {
   if(digitalRead(2)) {
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