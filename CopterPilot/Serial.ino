uint8_t serialBufferRX[256][1];
volatile uint8_t serialHeadRX[1],serialTailRX[1];

#define NULLDATA ((int16_t)'X')<<8|'Y'
#define NULLDATA2 ((int16_t)'1')<<8|'2'
#define NULLDATA8 'Z'
#define NULLDATA82 '9'

void serialCom() {
  uint8_t i, sr;

  uint16_t intPowerMeterSum, intPowerTrigger1;   

  if (SerialAvailable(0)) {
    switch (sr = SerialRead(0)) {
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
      serialize8('M');
      serialize8(VERSION);  // MultiWii Firmware version
      for(i=0;i<3;i++) serialize16(pilot._imu.imu.a[i]); //pilot._imu.imu.a[i]);
      for(i=0;i<3;i++) serialize16(pilot._imu.imu.g[i]); //518);
      for(i=0;i<3;i++) serialize16(pilot._imu.magn.m[i]); //519);

      serialize16(100); //100); //EstAlt/10
      serialize16(pilot._imu.magn.heading()); // compass
      for(i=0;i<8;i++) serialize16(NULLDATA2); //0); //servo[i]
      for(i=0;i<4;i++) serialize16(NULLDATA2); //pilot._motors.get(i));
      for(i=4;i<8;i++) serialize16(NULLDATA2); //pilot._motors.get(i-4)); // 8 motors?!
      for(i=0;i<8;i++) serialize16(pilot._receiver.get(i)/2);
      serialize8(1|1<<1|1<<2|1<<3|0<<4); //nunchuk|ACC<<1|BARO<<2|MAG<<3|GPSPRESENT<<4
      serialize8(0); //accMode|baroMode<<1|magMode<<2|(GPSModeHome|GPSModeHold)<<3
      serialize16(cycleTime);
      serialize16(0); // i2c_errors_count
      for(i=0;i<2;i++) serialize16(0); //0); //angle[i]/10
      serialize8(2); // 2 is + config, 3 is x config
      /*
      for(i=0;i<PIDITEMS;i++) {serialize8(P8[i]);serialize8(I8[i]);serialize8(D8[i]);}
      serialize8(rcRate8);
      serialize8(rcExpo8);
      serialize8(rollPitchRate);
      serialize8(yawRate);
      serialize8(dynThrPID);
      */
      for(i=0;i<24;i++) serialize8(NULLDATA82); //0); // cover pids
      for(i=0;i<5;i++) serialize8(NULLDATA82); //0); // cover rest
      for(i=0;i<11;i++) { serialize8(NULLDATA82); serialize8(NULLDATA82); } // checkboxitems
      
      serialize16(NULLDATA); //0); //GPS_distanceToHome
      serialize16(NULLDATA); //0); //GPS_directionToHome
      serialize8(NULLDATA8); //6); //GPS_numSat
      serialize8(NULLDATA8); //0); //GPS_fix
      serialize8(NULLDATA8); //0); //GPS_update
      serialize16(NULLDATA); //pilot.ampere);
      serialize16(NULLDATA); //pilot.ampere_hours);
      serialize8(pilot.voltage);
      serialize16(123); //0);        // 4 variables are here for general monitoring purpose
      serialize16(SerialAvailable(0)); //0);  // debug2
      serialize16(cycleTimeMax); //0);                 // debug3
      serialize16(cycleCounter); //0);                 // debug4
      serialize8('M');
      UartSendData();
      //Serial.write(s,point);
      break;
    case 'W': //GUI write params to eeprom @ arduino
      while (SerialAvailable(0)<33) {}
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
      for(i=0;i<33;i++) SerialRead(0);
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


// *******************************************************
// Interrupt driven UART transmitter - using a ring buffer
// *******************************************************
static uint8_t headTX,tailTX;
static uint8_t bufTX[256];      // 256 is choosen to avoid modulo operations on 8 bits pointers
void serialize16(int16_t a) {
  bufTX[headTX++]  = a;
  bufTX[headTX++]  = a>>8&0xff;
  UCSR0B |= (1<<UDRIE0);      // in case ISR_UART desactivates the interrupt, we force its reactivation anyway
}
void serialize8(uint8_t a)  {
  bufTX[headTX++]  = a;
  UCSR0B |= (1<<UDRIE0);

}

ISR(USART_UDRE_vect) {
  if( headTX != tailTX )
    UDR0 = bufTX[tailTX++];       // Transmit next byte in the ring
  if ( tailTX == headTX )         // Check if all data is transmitted
    UCSR0B &= ~(1<<UDRIE0);       // Disable transmitter UDRE interrupt
}


void UartSendData() {               // Data transmission acivated when the ring is not empty
  UCSR0B |= (1<<UDRIE0);          // Enable transmitter UDRE interrupt
}


void SerialOpen(uint8_t port, uint32_t baud) {
  uint8_t h = ((F_CPU  / 4 / baud -1) / 2) >> 8;
  uint8_t l = ((F_CPU  / 4 / baud -1) / 2);
  switch (port) {
    case 0: UCSR0A  = (1<<U2X0); UBRR0H = h; UBRR0L = l; UCSR0B |= (1<<RXEN0)|(1<<TXEN0)|(1<<RXCIE0); break;
  }
}

void SerialEnd(uint8_t port) {
  switch (port) {
    case 0: UCSR0B &= ~((1<<RXEN0)|(1<<TXEN0)|(1<<RXCIE0)|(1<<UDRIE0)); break;
  }
}

ISR(USART_RX_vect){
  uint8_t d = UDR0;
  uint8_t i = serialHeadRX[0] + 1;
  if (i != serialTailRX[0]) {serialBufferRX[serialHeadRX[0]][0] = d; serialHeadRX[0] = i;}
}

uint8_t SerialRead(uint8_t port) {
  uint8_t c = serialBufferRX[serialTailRX[port]][port];
  if ((serialHeadRX[port] != serialTailRX[port])) serialTailRX[port] = serialTailRX[port] + 1;
  
  return c;
}

uint8_t SerialAvailable(uint8_t port) {
  return serialHeadRX[port] - serialTailRX[port];
}

void SerialWrite(uint8_t port,uint8_t c){
  switch (port) {
    case 0: serialize8(c);UartSendData(); break;                 // Serial0 TX is driven via a buffer and a background intterupt
  }
}

void SerialWriteInt(uint8_t port, int number){
  uint8_t t, i;
  unsigned long limit = 100000;
  while(limit > 1) {
    limit = limit / 10;
    t = number / limit;
    SerialWrite(port, '0' + t);
    number = number % limit;
  }
}
