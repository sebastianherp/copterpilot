
Pilot::Pilot() {
	 
}

void Pilot::init() {
    _receiver.init(RECEIVER_PIN);
    _motors.init(MOTOR_PINS);
    _imu.init();
    
    pinMode(A0, INPUT); // amps
    pinMode(A1, INPUT); // volts
    
    ampere = 0;
    ampere_hours = 0;
    voltage = 0;
    
    last_update = micros();
}

void Pilot::imu_update() {
  _imu.imu.read();
  //_imu.getQ(quat);
  _imu.getYawPitchRoll(ypr);
}

void Pilot::mag_update() {
  _imu.magn.read();
}

void Pilot::alt_update() {
  _imu.baro.read();
}

void Pilot::power_update() {
  long tmp = micros();
  ampere_ad = 512; //analogRead(0);
  // 0.174386 == 5 * / 1024 / -0.028
  ampere = (ampere_ad - 512) * -0.174386; 

  voltage_ad = analogRead(1);
  voltage = voltage_ad / 1024.0 * 5.0 * 5.5;
  
  // 0.0277778 == A * 100 [=mAs] / 3600 [=mAh]
  //ampere_hours += ampere * 0.0277778;
  ampere_hours += ampere * (tmp-last_update) / 3600000;

  last_update = tmp;
  
  if(BEEPER_ON && voltage < 10.0)
    tone(9, 400);
  else
    noTone(9);
  
}

void Pilot::receiver_update() {
    _receiver.update(); 
}

void Pilot::adjustMotorsPlus() {
  
  _motors.setYaw(_receiver.get(YAW));
  _motors.setThrottle(_receiver.get(THROTTLE));
  _motors.setRoll(_receiver.get(ROLL));
  _motors.setPitch(_receiver.get(PITCH));
  
  _motors.process(ypr);
  if(armed == ON && safetyCheck == ON) {
    _motors.write();
  }
  
}

void Pilot::processInput() {
  if(_receiver.get(THROTTLE) < RX_LOW) {
    
    // disarm motors if left stick in lower left corner
    if(_receiver.get(YAW) < RX_LOW && armed == ON) {
      armed = OFF;
      _motors.setAll(MOTOR_OFF);
    }
    
    // TODO: calibration
    
    // arm motors if left stick in lower right corner
    if(_receiver.get(YAW) > RX_HIGH && armed == OFF && safetyCheck == ON) {
      armed = ON;
      _motors.setAll(MOTOR_ON);
    }
    
    if(_receiver.get(YAW) > RX_LOW) safetyCheck = ON;
  }
  
}
