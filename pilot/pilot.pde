#include <Wire.h>
#include <IMU3000.h>
#include <BMP085.h>
#include <LSM303DLH.h>
#include <FreeIMU.h>
#include <Battery.h>
#include <Receiver.h>
#include <SPI.h>


#define BEEPER_ON false

char output[20];

FreeIMU myIMU = FreeIMU();
Battery bat = Battery();
Receiver receiver = Receiver();

float quat[4];
float angles[3];
float angles2[3] = {0.0, 0.0, 0.0};
float time_diff;
float scale_factor;




//Pin connected to latch pin (ST_CP) of 74HC595
const int latchPin = 10;
//Pin connected to clock pin (SH_CP) of 74HC595
const int clockPin = 13;
////Pin connected to Data in (DS) of 74HC595
const int dataPin = 11;
////Pin connected to Output Enable (GND = enabled)
const int outputEnablePin = 8;

//
// constants
//
#define PWM_FREQ 100 // hz
#define PWM_STEPS 128 // PWM resolution - set to 256 when using bam
#define BAM_STEPS 8 // how many times BAM updates are done each cycle
#define NUM_SHIFT_REGS 1  
#define NUM_CHANNELS NUM_SHIFT_REGS*8 // eigentlich 8
#define NUM_COLORS 21

// holds the current pwm values of pins to be sent by interrupt
volatile byte pwmValues[NUM_CHANNELS];
volatile byte bamLookup[BAM_STEPS*NUM_SHIFT_REGS]; // precalculate the bytes to send every time the PWM values change
volatile byte colors[NUM_COLORS][3];

unsigned long timer_now;
unsigned long timer_old;
unsigned long timer_diff;
int timer_counter;

volatile uint8_t atomicServo[4] = {125,125,125,125};

volatile unsigned int rx_values[8];

void init_ppm_out() {
    pinMode(4, OUTPUT);
    pinMode(5, OUTPUT);
    pinMode(6, OUTPUT);
    pinMode(7, OUTPUT);
  
    
    TCCR1A = 0; // normal counting mode
    TCCR1B = 0 | (1<<CS11); // | (1<<CS10); // prescaler :8
    TIMSK1 |= (1<<OCIE1A); // Enable CTC interrupt  
}

ISR(TIMER1_COMPA_vect) {
  static uint8_t state = 0;
  static uint8_t count;
  if (state == 0) {
    //http://billgrundmann.wordpress.com/2009/03/03/to-use-or-not-use-writedigital/
    bitWrite(PORTD, 4, 1);
    OCR1A+= rx_values[2]; // 0.5 us increments
    state++ ;
  } else if (state == 1) {
    bitWrite(PORTD, 4, 0);
    bitWrite(PORTD, 5, 1);
    OCR1A+= rx_values[2]; // 1000 us
    state++;
  } else if (state == 2) {
    bitWrite(PORTD, 5, 0);
    bitWrite(PORTD, 6, 1);
    OCR1A+= rx_values[2]; // 1000 us
    state++;
  } else if (state == 3) {
    bitWrite(PORTD, 6, 0);
    bitWrite(PORTD, 7, 1);
    OCR1A+= rx_values[2]; // 1000 us
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

void receiver_update() {
  receiver.update(); 
}

void setup()
{
    setup_lights();
    
    bat.init();
    receiver.init();
    attachInterrupt(0, receiver_update, CHANGE);
  
    Serial.begin(28800); 
    
    Wire.begin();

    delay(5);
    myIMU.init();
    delay(5);
    
    scale_factor = 180000000.0 / (myIMU.g_scale.x * PI);
    
    timer_old = micros();
    timer_counter = 0;
    
        setChannel(7, 0);
        setChannel(6, 255);
        setChannel(5, 0);

        setChannel(4, 127);
        setChannel(3, 255);
        setChannel(2, 0);

  init_ppm_out();

}

byte light_i = 0;
int buzzer_odd = 0;
byte light_values = 0;

unsigned long countme = 0;

void loop()
{
    timer_now = micros();
    timer_diff = timer_now - timer_old;

    //iUpdateBAM3();
/*
                // latch off
        digitalWrite(latchPin, LOW);
        // send bytes the last register first
        light_values = SPI.transfer(light_i++);
         // latch on
        digitalWrite(latchPin, HIGH);   
  */
  
    /* 200 Hz base timing */
    if(timer_diff >= 5000) {
      timer_counter++;
      timer_old += 5000;
        
      if(timer_counter % 2 == 0) { // 200Hz
        myIMU.update();
        myIMU.getYawPitchRoll(angles);
        //myIMU.getQ(quat);
      }
      
      if(timer_counter % 100 == 0) { // 1Hz
     
        //TODO: flight commands
        Serial.print(timer_now / 1000);
        Serial.print(", ");        
        Serial.print(bat.voltage);
        Serial.print("V, ");        
        Serial.print(bat.ampere);
        Serial.print("A, "); 
        Serial.print(bat.ampere_hours, 0);
        Serial.print("mAh, "); 
        Serial.print(receiver.rx_values[0]);
        Serial.print(", ");
        Serial.print(receiver.rx_values[1]);
        Serial.print(", ");
        Serial.print(receiver.rx_values[2]);
        Serial.print(", ");
        Serial.print(receiver.rx_values[3]);
        Serial.print(", ");
        Serial.print(receiver.rx_values[4]);
        Serial.print(", ");
        Serial.print(receiver.rx_values[5]);
        Serial.print(", ");
        Serial.print(receiver.rx_values[6]);
        Serial.print(", ");
        Serial.print(receiver.rx_values[7]);
        Serial.println("");          

      }
      
      if(timer_counter % 20 == 0) { // 10Hz
        bat.update();

        if(BEEPER_ON && bat.voltage < 10.0)
          tone(9, 400);
        else
          noTone(9);
        
        //PrintSensors();
  
      }

      
    }
    if(timer_counter >= 200) {
      timer_counter = 0;
      countme = 0;
    }
}

int myvar = 0;
int myvar2 = 0;



void PrintSensors() {
    
  
/*    float t = p_avg/101325.0;
    t = 1 - pow(t, 0.190295);
    float altitude = 44330 * t;
*/

    // Print out what we have
    /*
    sprintf(output, "%+09i, ", timer_diff);
    Serial.print(output);
    sprintf(output, "%+06i, ", timer_counter);
    Serial.print(output);
    Serial.print(timer_diffcounter, 2);
    Serial.print(", ");
    */
    
    
    
    Serial.print(timer_diff);
    Serial.print("; ");
    Serial.print(bat.voltage);
    Serial.print(", ");
    Serial.print((int)light_values, BIN);
    Serial.print(", ");
    Serial.print((int)light_i, BIN);
    Serial.print("; ");
    //float yaw = atan2(2*(myIMU.q0*myIMU.q1 + myIMU.q2*myIMU.q3), 1 - 2*(myIMU.q1*myIMU.q1 + myIMU.q2*myIMU.q2)) * 180.0 / PI;
    Serial.print(myIMU.magn.heading());
    Serial.print("; ");
/*
    Serial.print(quat[0]);
    Serial.print(", ");
    Serial.print(quat[1]);
    Serial.print(", ");
    Serial.print(quat[2]);
    Serial.print(", ");
    Serial.print(quat[3]);
    Serial.print("; ");    
   
*/

    Serial.print(rx_values[0]);
    Serial.print(", ");
    Serial.print(rx_values[1]);
    Serial.print(", ");
    Serial.print(rx_values[2]);
    Serial.print(", ");
    Serial.print(rx_values[3]);
    Serial.print("; ");   

    Serial.print(myIMU.wx);
    Serial.print(", ");
    Serial.print(myIMU.wy);
    Serial.print(", ");
    Serial.print(myIMU.wz);
    Serial.print("; ");    
    
/*    
    Serial.print(myIMU.g_scale.x);
    Serial.print(", ");
    Serial.print(myIMU.g_bias.x);
    Serial.print(", ");
    Serial.print(myIMU.imu.g.x);
    Serial.print("; ");
*/

/*    
    Serial.print(timer_counter);
    Serial.print("; ");
    Serial.print(myIMU.m_max.x);
    Serial.print(", ");
    Serial.print(myIMU.m_max.y);
    Serial.print(", ");
    Serial.print(myIMU.m_max.z);
    Serial.print("; ");
    Serial.print(myIMU.m_min.x);
    Serial.print(", ");
    Serial.print(myIMU.m_min.y);
    Serial.print(", ");
    Serial.print(myIMU.m_min.z);
    Serial.print("; ");
    
    Serial.print(myIMU.magn.m.x);
    Serial.print(", ");
    Serial.print(myIMU.magn.m.y);
    Serial.print(", ");
    Serial.print(myIMU.magn.m.z);
    Serial.print("; ");

    Serial.print(myIMU.g_bias.x);
    Serial.print(", ");
    Serial.print(myIMU.g_bias.y);
    Serial.print(", ");
    Serial.print(myIMU.g_bias.z);
    Serial.print("; ");

    Serial.print(myIMU.imu.g.x);
    Serial.print(", ");
    Serial.print(myIMU.imu.g.y);
    Serial.print(", ");
    Serial.print(myIMU.imu.g.z);
    Serial.print(", ");
*/

/*    Serial.print(imu.a.x);
    Serial.print(", ");
    Serial.print(imu.a.y);
    Serial.print(", ");
    Serial.print(imu.a.z);
    Serial.print("; ");

    Serial.print(compass.a.x);
    Serial.print(", ");
    Serial.print(compass.a.y);
    Serial.print(", ");
    Serial.print(compass.a.z);
    Serial.print("; ");
    

*/    

    Serial.println("");     // prints carriage return
    
}

/*
  Precalculate the bytes to send for each time slice. Call everytime you update or change
  pwmvalues.

*/
void precalcBamBytes() {
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

#define chalf 127

void prepareColors() {

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

// interrupt counter
volatile int ticker = 0;
volatile byte odd = 0;

void setup_lights() {                
  
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
void sendSPI(volatile byte *valuesToSend, int from, int to) {
  // latch off
  digitalWrite(latchPin, LOW);  
  // send bytes the last register first
  for(int i = to-1; i>=from;i--) {
    SPI.transfer(valuesToSend[i]);
  }
   // latch on
  digitalWrite(latchPin, HIGH);
}

void setAllChannelsTo(int duty) {
  for(int i = 0;i<NUM_CHANNELS;i++) {
    pwmValues[i] = duty;
  }
  precalcBamBytes();
}

void setChannel(int ch,int duty) {
  pwmValues[ch] = duty;
  precalcBamBytes();
}


/*

  Bit Angle modulation. Faster version that uses precalculated data for each slice.
  Mighty fast compared to the other version.

  This fersion also divideds
  the bits 6 and 7 to 32 tick slices to smoothen the transitions over 126->127


*/
void iUpdateBAM3() {
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

