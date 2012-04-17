Motors::Motors() {
    _throttle = _yaw = _pitch = _roll = 0;
	
    for(byte motor = 0; motor < LASTMOTOR; motor++) {
      motor_values[motor] = MOTOR_OFF;
      motor_calculated_values[motor] = MOTOR_OFF;
      motor_min_values[motor] = MOTOR_LOW;
      motor_max_values[motor] = MOTOR_HIGH;
    }
}

void Motors::init(int pin1, int pin2, int pin3, int pin4) {
    _pin1 = pin1;
    _pin2 = pin2;
    _pin3 = pin3;
    _pin4 = pin4;
	
    pinMode(_pin1, OUTPUT);
    pinMode(_pin2, OUTPUT);
    pinMode(_pin3, OUTPUT);
    pinMode(_pin4, OUTPUT);
  
    TCCR1A = 0; // normal counting mode
    TCCR1B = 0 | (1<<CS11); // | (1<<CS10); // prescaler :8
    TIMSK1 |= (1<<OCIE1A); // Enable CTC interrupt  
}

// calculate motor output
void Motors::process(float * ypr) {
	// roll & pitch
	// calc motor change motor_calculated_values[ROLL] = _roll - ypr[0];

	_yaw = _yaw - 3000;
	_roll = _roll - 3000;
	_pitch = _pitch - 3000;
	
	motor_calculated_values[FRONT] = _throttle + _pitch + _yaw;
	motor_calculated_values[REAR] = _throttle - _pitch + _yaw;
	motor_calculated_values[LEFT] = _throttle + _roll - _yaw;
	motor_calculated_values[RIGHT] = _throttle - _roll - _yaw;
}

// write motor output
void Motors::write() {
	for(byte motor = 0; motor < LASTMOTOR; motor++) {
		motor_values[motor] = motor_calculated_values[motor];
	}
}

void Motors::setAll(unsigned int value) {
	for(byte motor = 0; motor < LASTMOTOR; motor++) {
		motor_values[motor] = value;
	}
}

void Motors::handleInterrupt() {
	static uint8_t state = 0;
	static uint8_t count = 0;
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
