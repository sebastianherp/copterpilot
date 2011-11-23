/*
Battery.cpp - Library to measure A and V of LiPo
Copyright (C) 2011 Sebastian Herp
*/

#include "WProgram.h"
#include "Battery.h"

Battery::Battery() {

}

void Battery::init() {
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  
  ampere = 0;
  ampere_hours = 0;
  voltage = 0;
  
  last_update = micros();
}

void Battery::update() {
  long tmp = micros();
  ampere_ad = analogRead(0);
  // 0.174386 == 5 * / 1024 / -0.028
  ampere = (ampere_ad - 512) * -0.174386; 

  voltage_ad = analogRead(1);
  voltage = voltage_ad / 1024.0 * 5.0 * 5.5;
  
  // 0.0277778 == A * 100 [=mAs] / 3600 [=mAh]
  //ampere_hours += ampere * 0.0277778;
  ampere_hours += ampere * (tmp-last_update) / 3600000;

  last_update = tmp;
}