static uint8_t point;
static uint8_t s[128];
void serialize16(int16_t a) {s[point++]  = a; s[point++]  = a>>8&0xff;}
void serialize8(uint8_t a)  {s[point++]  = a;}

// ***********************************
// Interrupt driven UART transmitter for MIS_OSD
// ***********************************
static uint8_t tx_ptr;
static uint8_t tx_busy = 0;


//ISR(USART_UDRE_vect) {
//  UDR0 = s[tx_ptr++];           /* Transmit next byte */
//  if ( tx_ptr == point ) {      /* Check if all data is transmitted */
//    UCSR0B &= ~(1<<UDRIE0);     /* Disable transmitter UDRE interrupt */
//    tx_busy = 0;
//  }
//}

//void UartSendData() {          // start of the data block transmission
//  cli();
//  tx_ptr = 0;
//  UCSR0A |= (1<<UDRE0);        /* Clear UDRE interrupt flag */
//  UCSR0B |= (1<<UDRIE0);       /* Enable transmitter UDRE interrupt */
//  UDR0 = s[tx_ptr++];          /* Start transmission */
//  tx_busy = 1;
//  sei();
//}

void serialCom() {
  int16_t a;
  uint8_t i;

  uint16_t intPowerMeterSum, intPowerTrigger1;   

  if ((!tx_busy) && Serial.available()) {
    switch (Serial.read()) {
    #ifdef BTSERIAL
    case 'K': //receive RC data from Bluetooth Serial adapter as a remote
      rcData[THROTTLE] = (Serial.read() * 4) + 1000;
      rcData[ROLL]     = (Serial.read() * 4) + 1000;
      rcData[PITCH]    = (Serial.read() * 4) + 1000;
      rcData[YAW]      = (Serial.read() * 4) + 1000;
      rcData[AUX1]     = (Serial.read() * 4) + 1000;
      break;
    #endif
    case 'M': // Multiwii @ arduino to GUI all data
      point=0;
      serialize8('M');
      serialize8(VERSION);  // MultiWii Firmware version
      serialize16(pilot._imu.imu.a.x);
      serialize16(pilot._imu.imu.a.y);
      serialize16(pilot._imu.imu.a.z);
      serialize16(1234);
      serialize16(5678);
      serialize16(55555);
      //serialize16(myIMU.imu.g.x);
      //serialize16(myIMU.imu.g.y);
      //serialize16(myIMU.imu.g.z);
      serialize16(pilot._imu.magn.m.x);
      serialize16(pilot._imu.magn.m.y);
      serialize16(pilot._imu.magn.m.z);

      serialize16(100); //EstAlt/10
      serialize16(pilot._imu.magn.heading()); // compass
      for(i=0;i<4;i++) serialize16(0); //servo[i]
      for(i=0;i<4;i++) serialize16(pilot._motors.get(i));
      for(i=4;i<8;i++) serialize16(pilot._motors.get(i-4)); // 8 motors?!
      for(i=0;i<8;i++) serialize16(pilot._receiver.get(i));
      serialize8(1|1<<1|1<<2|1<<3|0<<4); //nunchuk|ACC<<1|BARO<<2|MAG<<3|GPSPRESENT<<4
      serialize8(0); //accMode|baroMode<<1|magMode<<2|(GPSModeHome|GPSModeHold)<<3
      serialize16(cycleTime);
      for(i=0;i<2;i++) serialize16(0); //angle[i]/10
      serialize8(2); // 2 is + config, 3 is x config
      /*
      for(i=0;i<5;i++) {serialize8(P8[i]);serialize8(I8[i]);serialize8(D8[i]);}
      serialize8(P8[PIDLEVEL]);
      serialize8(I8[PIDLEVEL]);
      serialize8(P8[PIDMAG]);
      serialize8(rcRate8);
      serialize8(rcExpo8);
      serialize8(rollPitchRate);
      serialize8(yawRate);
      serialize8(dynThrPID);
      for(i=0;i<8;i++) serialize8(activate[i]); */
      for(i=0;i<21;i++) serialize8(0); // cover commented region      
      serialize16(0); //GPS_distanceToHome
      serialize16(0); //GPS_directionToHome
      serialize8(6); //GPS_numSat
      serialize8(0); //GPS_fix
      serialize8(0); //GPS_update
      serialize16(pilot.ampere);
      serialize16(pilot.ampere_hours);
      serialize16(pilot.voltage);
      serialize16(0);        // 4 variables are here for general monitoring purpose
      serialize16(0);  // debug2
      serialize16(0);                 // debug3
      serialize16(0);                 // debug4
      serialize8('M');
      //UartSendData();
      Serial.write(s,point);
      break;
    case 'W': //GUI write params to eeprom @ arduino
      while (Serial.available()<33) {}
      /*
      for(i=0;i<5;i++) {P8[i]= Serial.read(); I8[i]= Serial.read(); D8[i]= Serial.read();} //15
      P8[PIDLEVEL] = Serial.read(); I8[PIDLEVEL] = Serial.read(); //17
      P8[PIDMAG] = Serial.read(); //18
      rcRate8 = Serial.read(); rcExpo8 = Serial.read(); //20
      rollPitchRate = Serial.read(); yawRate = Serial.read(); //22
      dynThrPID = Serial.read(); //23
      for(i=0;i<8;i++) activate[i] = Serial.read(); //31
     #if defined(POWERMETER)
      powerTrigger1 = (Serial.read() + 256* Serial.read() ) / PLEVELSCALE; // we rely on writeParams() to compute corresponding pAlarm value
     #else
      Serial.read();Serial.read(); // so we unload the two bytes
     #endif
      writeParams();
      */
      for(i=0;i<33;i++) Serial.read();
      break;
    case 'S': //GUI to arduino ACC calibration request
      //calibratingA=400;
      break;
    case 'E': //GUI to arduino MAG calibration request
      //calibratingM=1;
      break;
    }
  }
}

