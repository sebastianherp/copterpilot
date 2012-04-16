/*
LED.cpp - Library to measure A and V of LiPo
Copyright (C) 2011 Sebastian Herp
*/

#include "LED.h"
#include <SPI.h>



LED::LED() {
	chalf = 127;
	latchPin = 10;
	clockPin = 13;
	dataPin = 11;
	outputEnablePin = 8;
	ticker = 0;
	odd = 0;
	
	setup_lights();
}

/*
  Precalculate the bytes to send for each time slice. Call everytime you update or change
  pwmvalues.

*/
void LED::precalcBamBytes() {
  byte bytesToSend[NUM_SHIFT_REGS];
  
  for(int slice=0;slice<BAM_STEPS;slice++) {
    unsigned int sliceMask = (1 << slice);  
    // generate one byte per register
    for(unsigned char regNo = 0 ;regNo<NUM_SHIFT_REGS;regNo++) {
      bytesToSend[regNo]  = 0;
      // loop bits of each register
      for(unsigned char ch = 0;ch<8;ch++){
        // test if the pwm value has the current slicebit 1
        if( (pwmValues[regNo*8+ch] & sliceMask) == sliceMask) {
          // turn on channel 0-7
          bytesToSend[regNo] |= (1 << ch);
        }
        
      }
      bamLookup[slice*NUM_SHIFT_REGS + regNo] = bytesToSend[regNo];
    }
    
  }

}

void LED::prepareColors() {

 int i=0;
 colors[i][0] = 0; colors[i][1] = 0; colors[i][2] = 0; i++; //black
 colors[i][0] = 255; colors[i][1] = 0; colors[i][2] = 0; i++; //red
 colors[i][0] = 255; colors[i][1] = chalf; colors[i][2] = 0; i++; //orange
 colors[i][0] = 255; colors[i][1] = 0; colors[i][2] = chalf; i++; //pink
 colors[i][0] = 255; colors[i][1] = chalf; colors[i][2] = chalf; i++; //skin
 
 colors[i][0] = 0; colors[i][1] = 255; colors[i][2] = 0; i++; //green
 colors[i][0] = chalf; colors[i][1] = 255; colors[i][2] = 0; i++; //lime
 colors[i][0] = 0; colors[i][1] = 255; colors[i][2] = chalf; i++; //grass
 colors[i][0] = chalf; colors[i][1] = 255; colors[i][2] = chalf; i++; //grassier

 colors[i][0] = 0; colors[i][1] = 0; colors[i][2] = 255; i++; //blue
 colors[i][0] = chalf; colors[i][1] = 0; colors[i][2] = 255; i++; //violet
 colors[i][0] = 0; colors[i][1] = chalf; colors[i][2] = 255; i++; //sky
 colors[i][0] = chalf; colors[i][1] = chalf; colors[i][2] = 255; i++; //lavendel
 
 colors[i][0] = 255; colors[i][1] = 255; colors[i][2] = 0; i++; //yellow
 colors[i][0] = chalf; colors[i][1] = chalf; colors[i][2] = 0; i++; //ockar
 colors[i][0] = 0; colors[i][1] = 255; colors[i][2] = 255; i++; //cyan
 colors[i][0] = 0; colors[i][1] = chalf; colors[i][2] = chalf; i++; //greenlish
 colors[i][0] = 255; colors[i][1] = 0; colors[i][2] = 255; i++; //magenta
 colors[i][0] = chalf; colors[i][1] = 0; colors[i][2] = chalf; i++; //violetish

 colors[i][0] = 255; colors[i][1] = 255; colors[i][2] = 255; i++; //white
 colors[i][0] = chalf; colors[i][1] = chalf; colors[i][2] = chalf; i++; //gray
 
}



void LED::setup_lights() {                
  
  //set pins to output because they are addressed in the main loop
  pinMode(outputEnablePin, OUTPUT);
  digitalWrite(outputEnablePin, HIGH);
  pinMode(latchPin, OUTPUT);
  pinMode(dataPin, OUTPUT);  
  pinMode(clockPin, OUTPUT);
  
  // serial communication to 595s
  SPI.begin();
  SPI.setClockDivider(SPI_CLOCK_DIV2);

  //enable output
  setAllChannelsTo(0);
  iUpdateBAM3();
  
  digitalWrite(outputEnablePin, LOW);
  prepareColors();
}

/*
  Sends a byte array using SPI to 595 shift registers
*/
void LED::sendSPI(volatile byte *valuesToSend, int from, int to) {
  // latch off
  digitalWrite(latchPin, LOW);  
  // send bytes the last register first
  for(int i = to-1; i>=from;i--) {
    SPI.transfer(valuesToSend[i]);
  }
   // latch on
  digitalWrite(latchPin, HIGH);
}

void LED::setAllChannelsTo(int duty) {
  for(int i = 0;i<NUM_CHANNELS;i++) {
    pwmValues[i] = duty;
  }
  precalcBamBytes();
}

void LED::setChannel(int ch,int duty) {
  pwmValues[ch] = duty;
  precalcBamBytes();
}


/*

  Bit Angle modulation. Faster version that uses precalculated data for each slice.
  Mighty fast compared to the other version.

  This fersion also divideds
  the bits 6 and 7 to 32 tick slices to smoothen the transitions over 126->127


*/
void LED::iUpdateBAM3() {
  // update ticker and reset if rolls over
  ticker++;
  if(ticker > PWM_STEPS) {
    ticker = 0;
  }
  
  if(ticker == 0)
    odd = (odd + 1) % 2;

  // find out timeslice startpoint we are at
  unsigned char slice;
  if(odd == 0) {
    if(ticker == 0)
      slice = 0;
    else if(ticker == 1)
      slice = 1;
    else if(ticker == 3)
      slice = 2;
    else if(ticker == 7)
      slice = 3;
    else if(ticker == 15)
      slice = 4;
    else if(ticker == 31)
      slice = 5;
    else if(ticker == 63)
      slice = 6;
    else if(ticker == 127)
      slice = 7;
    else if(ticker == 255)
      slice = 8;
    else if(ticker == 511)
      slice = 9;
    else // no actions required
      return;
  }
  else
  {
    if(ticker == 0)
      slice = 7;    
    else if(ticker == 129)
      slice = 6;    
    else if(ticker == 193) // 128+64
      slice = 5;    
    else if(ticker == 225) // 128+64+32
      slice = 4;    
    else if(ticker == 241)
      slice = 3;    
    else if(ticker == 249)
      slice = 2;    
    else if(ticker == 253)
      slice = 1;    
    else if(ticker == 255)
      slice = 0;
    else // no actions required
      return;
  }

  // update registers. The lookup table is recalculated always when setting pwm values.
  sendSPI(bamLookup,slice*NUM_SHIFT_REGS,slice*NUM_SHIFT_REGS + NUM_SHIFT_REGS);
}