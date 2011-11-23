
#include <p18cxxx.h>
#include "rx2cppm.h"
#include "delays.h"

void high_isr (void);
void update_channel(char chan, int value); 


#define CHANNELS 8
#define PPM_ADJ 9
#define PPM_BREAK 300 //300ms = 300 - 65
#define PPM_FRAMELENGTH 22500
#define PPM_MINSTARTFRAME 5000

char ppm_on = 0;
char counter = 0;
unsigned int framelength = 0;
unsigned int channels[CHANNELS];
volatile unsigned int values[CHANNELS];

unsigned int timer = 0, timer2 = 0;
unsigned char test = 0, prev = 0, diff = 0;
char i;
char signal = 0;

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
	timer = TMR1L + TMR1H * 256;

	if(INTCONbits.TMR0IF) {
		if(ppm_on == 0) {
			PORTAbits.RA7 = 1;
			if(counter == CHANNELS) {
				// start frame
				/*framelength = 0;
				for(i=0;i<CHANNELS;i++)
					framelength += values[i];
				framelength -= PPM_ADJ*CHANNELS;
				framelength = PPM_FRAMELENGTH - framelength;
				if(framelength < PPM_MINSTARTFRAME)
				*/

				framelength = PPM_MINSTARTFRAME;

				// 10 ms break;
				TMR0H = 255 - (framelength / 256);
				TMR0L = 255 - (framelength % 256);
				counter = 0;
				if(signal-- <= 0) {
					values[2] = 500;
					signal = 0;
				}
				
			} else {			
				// normales frame
				TMR0H = 255 - ((values[counter]) / 256);
				TMR0L = 255 - ((values[counter]) % 256);
				counter++;
			}
			ppm_on = 1;

		} else {
			PORTAbits.RA7 = 0;
			// 300 ms break;
			TMR0H = 255 - (PPM_BREAK / 256);
			TMR0L = 255 - (PPM_BREAK % 256);
			ppm_on = 0;
		}

		INTCONbits.TMR0IF = 0;
	}
	/*
  	if(PIR1bits.TMR1IF) 
    {
		PIR1bits.TMR1IF = 0;              //clears flag 
    }*/

	if(INTCONbits.INT0IF)
	{
		
		if(INTCON2bits.INTEDG0) {
			INTCON2bits.INTEDG0 = 0;
			channels[0] = timer;
		} else {
			INTCON2bits.INTEDG0 = 1;
			update_channel(0, timer);
		}
		INTCONbits.INT0IF = 0;
	}

	if(INTCON3bits.INT1IF)
	{
		//timer = TMR1L + TMR1H * 256;
		if(INTCON2bits.INTEDG1) {
			INTCON2bits.INTEDG1 = 0;
			channels[1] = timer;
		} else {
			INTCON2bits.INTEDG1 = 1;
			update_channel(1, timer);
		}
		INTCON3bits.INT1IF = 0;
	}

	if(INTCON3bits.INT2IF)
	{
		//timer = TMR1L + TMR1H * 256;
		if(INTCON2bits.INTEDG2) {
			INTCON2bits.INTEDG2 = 0;
			channels[3] = timer;
		} else {
			INTCON2bits.INTEDG2 = 1;
			update_channel(3, timer);
		}
		INTCON3bits.INT2IF = 0;
	}

	if(INTCON3bits.INT3IF)
	{
		//timer = TMR1L + TMR1H * 256;
		if(INTCON2bits.INTEDG3) {
			INTCON2bits.INTEDG3 = 0;
			channels[2] = timer;
		} else {
			INTCON2bits.INTEDG3 = 1;
			update_channel(2, timer);
			signal = 3;
		}
		INTCON3bits.INT3IF = 0;
	}
} 

void setup(void) {
	// Initializing ports
	PWMCON0 = 0; // pwm off (wtf!)
	ADCON1 = 0x0F; // ports digital
    PORTA = 0x00;
    PORTB = 0x00;

    // Set RA4 as input and RB3-RB0 as output
    TRISA = 0b00001111;
    TRISB = 0b00001111;

	OSCCON |= 0b01110000; // set 8 MHz frequency
	OSCTUNEbits.PLLEN = 1; // *4 = 32 MHz

	//T1CON = 0b10000001;
	T1CONbits.TMR1CS = 0;                     //INTERNAL CLOCK (Fosc/4) 
	PIR1bits.TMR1IF = 0; 
	PIE1bits.TMR1IE = 0;                              // ENABLES TIMER1 INTERRUPT 
	T1CONbits.T1CKPS1 = 1; 
	T1CONbits.T1CKPS0 = 1;                    //PRESCALAR 1:1, DIVIDE THE FREQ. TO 8 
	T1CONbits.TMR1ON = 1; 

	INTCONbits.TMR0IF = 0;
	INTCONbits.TMR0IE = 1;
	T0CON = 0;
	TMR0H = 128;
	TMR0L = 128;
	T0CONbits.T0PS2 = 0;
	T0CONbits.T0PS1 = 1;
	T0CONbits.T0PS0 = 0;
	T0CONbits.TMR0ON = 1;

	INTCON2bits.INTEDG0 = 1;		// rising edge
	INTCONbits.INT0IF = 0;
	INTCONbits.INT0IE = 1;
	
	INTCON2bits.INTEDG1 = 1;
	INTCON3bits.INT1IF = 0;
	INTCON3bits.INT1IE = 1;

	INTCON2bits.INTEDG2 = 1;
	INTCON3bits.INT2IF = 0;
	INTCON3bits.INT2IE = 1;

	INTCON2bits.INTEDG3 = 1;
	INTCON3bits.INT3IF = 0;
	INTCON3bits.INT3IE = 1;

	INTCONbits.GIE = 1;                              //enable interrupts 
	INTCONbits.PEIE = 1;                           //ENABLE PERIPHERAL INTERRUPTS 
	
	update_channel(0, 1000);
	update_channel(1, 1000);
	update_channel(2, 1000);
	update_channel(3, 1000);
	update_channel(4, 1000);
	update_channel(5, 1000);
	update_channel(6, 1000);
	update_channel(7, 1000);
}

void update_channel(char chan, int value) {
	if(value < channels[chan])
		values[chan] = value + 65536 - channels[chan];
	else
		values[chan] = value - channels[chan];
}

void main(void)
{
	setup();
	test = 0;
	prev = 0;
	diff = 0;

	while(1) {
		test = (PORTA & 0b00001100) + (PORTB & 0b00000011);
 		diff = test ^ prev;

		timer2 = TMR1L + TMR1H * 256;
		i = 4;
		while(diff > 0) {
			if(diff & 1) {
				if(test >> (i-4) & 1) {
					channels[i] = timer2;
				} else  {
					update_channel(i, timer2);
				}
			}
			diff = diff >> 1;
			//test = test >> 1;
			i++;
		}
		prev = test;

		
	}
}

