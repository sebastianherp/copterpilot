
#include <p18f1230.h>
#include "rx2cppm.h"

/*

	_CONFIG1( JTAGEN_OFF &		//JTAG OFF
	 GCP_OFF & 					//CODE PROTECT OFF
	 GWRP_OFF & 				//WRITE PROTECT OFF
	 BKBUG_OFF &				//BACKGROUND DEBUGGER OFF
	 COE_OFF & 					//CLIP ON EMULATION (reserved, always 1 on PIC24F) OFF
	 ICS_PGx2 & 				//USING PGx2 pins
	 FWDTEN_OFF & 				//WATCHDOG OFF
	 WINDIS_OFF &				//WINDOWED WDT OFF
	 FWPSA_PR128 &				//WATCHDOG PRESCALER
	 WDTPS_PS32768)				//WATCHDOG POSTSCALER

	_CONFIG2( IESO_ON &			//TWO SPEED STARTUP ENABLED
	 //SOSCSEL_LPSOSC &			//ENABLE SECONDARY LOW POWER OSCILATOR (deactivated: causes weird startup
	 							// behaviour with newer chip revisions)
	 WUTSEL_FST &				//FAST WAKE-UP (unimplemented)
	 FNOSC_FRC & 				//USING INTERNAL FAST RC OSCILATOR
	 FCKSM_CSDCMD &	//CLOCK MONITOR / FAILSAFE DISABLED
	 OSCIOFNC_ON & 				//OSCI PINS CAN BE USED AS NORMAL IO PORTS
	 IOL1WAY_OFF &				//UNLIMITED WRITES TO RP REGISTERS
	 I2C1SEL_PRI &				//USE PRIMARY I2C1 pins
	 POSCMOD_NONE)	 			//NO PRIMARY OSCILATOR
*/



void setup(void) {

}

void loop(void) {

}


void main(void)
{
	setup();
	while(1) loop();	
}

