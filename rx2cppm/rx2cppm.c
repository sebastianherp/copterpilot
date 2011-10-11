
#include <p18cxxx.h>
#include "rx2cppm.h"
#include "delays.h"


void setup(void) {
	// Initializing ports
    PORTA = 0;
    PORTB = 0;

    // Set RA4 as input and RB3-RB0 as output
    TRISA &= 0x00;
    TRISB &= 0x00;

}

int counter = 0;

void loop(void) {
	
}


void main(void)
{
	setup();
	while(1) {
		LATB = 3; 
		LATA = 0;
	    Delay1KTCYx(100); 
	    LATB = 0;
		LATA = 12;
	    Delay1KTCYx(100); 
	}	
}

