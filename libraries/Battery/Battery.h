/*
Battery.h - Library to measure A and V of LiPo
Copyright (C) 2011 Sebastian Herp
*/

#include "Arduino.h"

#ifndef Battery_h
#define Battery_h

class Battery
{
public:
    Battery();
    void init();
    void update();  
    float ampere;
	float ampere_hours;
    float voltage;
    
private:
    int ampere_ad, voltage_ad;
	long last_update;
};

#endif