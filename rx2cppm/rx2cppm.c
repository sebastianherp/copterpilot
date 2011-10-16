
#include <p18cxxx.h>
#include "rx2cppm.h"
#include "delays.h"

void high_isr (void);
void update_channel(char chan, int value); 


#define CHANNELS 8
#define PPM_ADJ 62
#define PPM_BREAK 235 //300ms = 300 - 65
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
	/*
  	if(PIR1bits.TMR1IF) 
    {
		PIR1bits.TMR1IF = 0;              //clears flag 
    }*/
	timer = TMR1L + TMR1H * 256;
	if(INTCONbits.INT0IF)
	{
		
		if(INTCON2bits.INTEDG0) {
			INTCON2bits.INTEDG0 = 0;
			channels[0] = timer;
		} else {
			INTCON2bits.INTEDG0 = 1;
			update_channel(0, timer - channels[0]);
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
			update_channel(1, timer - channels[1]);
		}
		INTCON3bits.INT1IF = 0;
	}

	if(INTCON3bits.INT2IF)
	{
		//timer = TMR1L + TMR1H * 256;
		if(INTCON2bits.INTEDG2) {
			INTCON2bits.INTEDG2 = 0;
			channels[2] = timer;
		} else {
			INTCON2bits.INTEDG2 = 1;
			update_channel(2, timer - channels[2]);
		}
		INTCON3bits.INT2IF = 0;
	}

	if(INTCON3bits.INT3IF)
	{
		//timer = TMR1L + TMR1H * 256;
		if(INTCON2bits.INTEDG3) {
			INTCON2bits.INTEDG3 = 0;
			channels[3] = timer;
		} else {
			INTCON2bits.INTEDG3 = 1;
			update_channel(3, timer - channels[3]);
		}
		INTCON3bits.INT3IF = 0;
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
	PIE1bits.TMR1IE = 0;                              // ENABLES TIMER1 INTERRUPT 
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
	
	update_channel(0, 1050);
	update_channel(1, 1450);
	update_channel(2, 1850);
	update_channel(3, 1234);
	update_channel(4, 756);
	update_channel(5, 2000);
	update_channel(6, 1050);
	update_channel(7, 1888);
}

void update_channel(char chan, int value) {
	if(value < 0)
		values[chan] = value + 65536;
	else
		values[chan] = value;
/*
	if(chan < CHANNELS && value < 2000 && value > 700) {
		//if(chan > 3 && values[chan] > value + 50)
		//	values[chan] = (values[chan]*19 + value) /20;
		//else
			values[chan] = value;
	}
*/
}

void main(void)
{
	setup();
	test = 0;
	prev = 0;
	diff = 0;

/*
	while(1) {
		test = PORTAbits.RA2;
		timer2 = TMR1L + TMR1H * 256;
		if(!prev && test) {
			channels[4] = timer2;
		} else if(prev && !test) {
			update_channel(4, timer2 - channels[4]);
		}
		prev = test;
	} */

	while(1) {
		test = (PORTA>>2 & 0b00000011) + (PORTB << 2 & 0b00001100);
 		diff = test ^ prev;

		timer2 = TMR1L + TMR1H * 256;
		//for(i=0;i<CHANNELS;i++) {
		i = 4;
		while(diff > 0) {
			if(diff & 1) {
				if(test >> (i-4) & 1) {
					channels[i] = timer2;
				} else  {
					update_channel(i, timer2 - channels[i]);
				}
			}
			diff = diff >> 1;
			//test = test >> 1;
			i++;
		}
		prev = test;

		
	}
}

