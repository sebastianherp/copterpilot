
#include <p18cxxx.h>
#include "rx2cppm.h"
#include "delays.h"

void high_isr (void); 
unsigned char count; 

#define CHANNELS 8
#define PPM_ADJ 66
#define PPM_BREAK 235 //300ms = 300 - 65
#define PPM_FRAMELENGTH 22500
#define PPM_MINSTARTFRAME 5000

char counter = 0;
char ppm_on = 0;
unsigned int framelength = 0;
unsigned int channels[CHANNELS];
volatile unsigned int values[CHANNELS];

unsigned int timer = 0, timer2;
unsigned char test = 0, prev = 0, diff = 0;
char i, c = 0;

//ISR................... 
#pragma code high_vector=0x08 
  void interrupt_at_high_vector(void) 
{ 
_asm GOTO high_isr _endasm 
} 
#pragma code /* return to the default code section */ 
#pragma interrupt high_isr 

void high_isr (void) 
{
	if(INTCONbits.TMR0IF) {
		if(ppm_on == 0) {
			PORTBbits.RB4 = 1;
			if(counter == CHANNELS) {
				// start frame
				framelength = 0;
				for(i=0;i<CHANNELS;i++)
					framelength += values[i];
				framelength -= PPM_ADJ*CHANNELS;
				framelength = PPM_FRAMELENGTH - framelength;
				if(framelength < PPM_MINSTARTFRAME)
					framelength = PPM_MINSTARTFRAME;

				// 10 ms break;
				TMR0H = 255 - (framelength / 256);
				TMR0L = 255 - (framelength % 256);
				counter = 0;
			} else {			
				// normales frame
				TMR0H = 255 - ((values[counter]-PPM_ADJ) / 256);
				TMR0L = 255 - ((values[counter]-PPM_ADJ) % 256);
				counter++;
			}
			ppm_on = 1;

		} else {
			PORTBbits.RB4 = 0;
			// 300 ms break;
			TMR0H = 255 - (PPM_BREAK / 256);
			TMR0L = 255 - (PPM_BREAK % 256);
			ppm_on = 0;
		}

		INTCONbits.TMR0IF = 0;
	}
  if(PIR1bits.TMR1IF) 
    { 
      count++; 
      PIR1bits.TMR1IF = 0;              //clears flag 
          
    }
} 

void setup(void) {
	// Initializing ports
	ADCON1 = 0x0F; // ports digital
    PORTA = 0;
    PORTB = 0;

    // Set RA4 as input and RB3-RB0 as output
    TRISA = 0b11111111;
    TRISB = 0b11101111;

	OSCCON |= 0b01110000; // set 8 MHz frequency

	//T1CON = 0b10000001;
	T1CONbits.TMR1CS = 0;                     //INTERNAL CLOCK (Fosc/4) 
	PIR1bits.TMR1IF = 0; 
	PIE1bits.TMR1IE = 1;                              // ENABLES TIMER1 INTERRUPT 
	T1CONbits.T1CKPS1 = 0; 
	T1CONbits.T1CKPS0 = 1;                    //PRESCALAR 1:1, DIVIDE THE FREQ. TO 8 
	T1CONbits.TMR1ON = 1; 

	INTCONbits.TMR0IF = 0;
	INTCONbits.TMR0IE = 1;
	T0CON = 0;
	TMR0H = 128;
	TMR0L = 128;
	T0CONbits.T0PS2 = 0;
	T0CONbits.T0PS1 = 0;
	T0CONbits.T0PS0 = 0;
	T0CONbits.TMR0ON = 1;

	INTCONbits.GIE = 1;                              //enable interrupts 
	INTCONbits.PEIE = 1;                           //ENABLE PERIPHERAL INTERRUPTS 
	
	values[0] = 1050;
	values[1] = 1450;
	values[2] = 1850;
	values[3] = 1234;
	values[4] = 756;
	values[5] = 2000;
	values[6] = 1050;
	values[7] = 1888;
}



void main(void)
{
	setup();
	test = 0;
	prev = 0;
	diff = 0;

	while(1) {
		test = (PORTA << 4) + (PORTB & 0xF);
 		diff = test ^ prev;
		prev = test;

		if(diff > 0) {
			timer = TMR1L + TMR1H * 256;
			//for(i=0;i<CHANNELS;i++) {
			i = 0;
			while(diff > 0) {
				if(diff & 1) {
					if(test & 1) {
						channels[i] = timer;
					} else  {
						values[i] = timer - channels[i];
					}
				}
				diff = diff >> 1;
				test = test >> 1;
				i++;
			}
			timer2 = TMR1L + TMR1H * 256 - timer;
		} //if
		
		
	}
}

