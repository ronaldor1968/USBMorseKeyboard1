/* Name: main.c
 * Project: Keyboard test
 * Author: Ronaldo Oliveira, baseado no exemplo de Christian Starkjohann
 * Creation Date: 2013-06-23
 * Tabsize: 4
 * Copyright: (c) 2006 by OBJECTIVE DEVELOPMENT Software GmbH
 * License: Proprietary, free under certain conditions. See Documentation.
 * This Revision: $Id: main.c 537 2008-02-28 21:13:01Z cs $
 * 
 * A  . _
 * B _ . . .
 * C _ . _ .
 * D _ . .
 * E .
 * F . . _ .
 * G _ _ .
 * H . . . .
 * I . .
 * J . _ _ _ 
 * K _ . _
 * L . _ . .
 * M _ _
 * N _ .
 * O _ _ _
 * P . _ _ .
 * Q _ _ . _
 * R . _ .
 * S . . .
 * T _
 * U . . _
 * V . . . _
 * W . _ _
 * X _ . . _
 * Y _ . _ _
 * Z _ _ . .
 * 1 . _ _ _ _
 * 2 . . _ _ _
 * 3 . . . _ _
 * 4 . . . . _
 * 5 . . . . .
 * 6 _ . . . .
 * 7 _ _ . . .
 * 8 _ _ _ . .
 * 9 _ _ _ _ .
 * 0 _ _ _ _ _
 * ESPACO . . _ _
 * CTRL + ALT + DEL . . . . . _
 * ENTER . . . . . .
 */

#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/eeprom.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include <stdlib.h>

#include "keycode.h"
#include "usbdrv.h"


#define BUTTON_PORT_B PORTB       	/* PORTx - register for BUTTON 1,2 e 3 output */
#define BUTTON_PIN_B  PINB        	/* PINx - register for BUTTON 1,2 e 3 */
#define BUTTON_BIT_B1 PB1          	/* bit for BUTTON 1 input/output */
#define BUTTON_BIT_B2 PB3          /* bit for BUTTON 3 input/output */
#define BUTTON_BIT_B3 PB4          /* bit for BUTTON 4 input/output */

#define DOT_TIME 5 // 64 é um segundo, maior que DOT_TIME = TRACO
#define WAIT_TIME 32 //

/* ------------------------------------------------------------------------- */

static uchar    reportBuffer[2];    /* buffer for HID reports */
static uchar    idleRate;           /* in 4 ms units */
static uchar    keyToSend = 0;		/* keys to send */

static unsigned int	timerDelay = 0;		/* counter for delay period */


/* ------------------------------------------------------------------------- */

const PROGMEM char usbHidReportDescriptor[USB_CFG_HID_REPORT_DESCRIPTOR_LENGTH] = { /* USB report descriptor */
    0x05, 0x01,                    // USAGE_PAGE (Generic Desktop)
    0x09, 0x06,                    // USAGE (Keyboard)
    0xa1, 0x01,                    // COLLECTION (Application)
    0x05, 0x07,                    //   USAGE_PAGE (Keyboard)
    0x19, 0xe0,                    //   USAGE_MINIMUM (Keyboard LeftControl)
    0x29, 0xe7,                    //   USAGE_MAXIMUM (Keyboard Right GUI)
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
    0x25, 0x01,                    //   LOGICAL_MAXIMUM (1)
    0x75, 0x01,                    //   REPORT_SIZE (1)
    0x95, 0x08,                    //   REPORT_COUNT (8)
    0x81, 0x02,                    //   INPUT (Data,Var,Abs)
    0x95, 0x01,                    //   REPORT_COUNT (1)
    0x75, 0x08,                    //   REPORT_SIZE (8)
    0x25, 0x65,                    //   LOGICAL_MAXIMUM (101)
    0x19, 0x00,                    //   USAGE_MINIMUM (Reserved (no event indicated))
    0x29, 0x65,                    //   USAGE_MAXIMUM (Keyboard Application)
    0x81, 0x00,                    //   INPUT (Data,Ary,Abs)
    0xc0                           // END_COLLECTION
};
/* We use a simplifed keyboard report descriptor which does not support the
 * boot protocol. We don't allow setting status LEDs and we only allow one
 * simultaneous key press (except modifiers). We can therefore use short
 * 2 byte input reports.
 * The report descriptor has been created with usb.org's "HID Descriptor Tool"
 * which can be downloaded from http://www.usb.org/developers/hidpage/.
 * Redundant entries (such as LOGICAL_MINIMUM and USAGE_PAGE) have been omitted
 * for the second INPUT item.
 */

//static uchar bufferSize = 0; 
static uchar command = 0;
static uchar lastcommand = 0;
static uchar nextkey = 0;
static uchar keymod  = 0; /* no modifiers */

static uchar tx0 = 0;
static uchar tx1 = 0;

static void checkButtonChange(void) {
	command = 0;
	if (bit_is_set(BUTTON_PIN_B, BUTTON_BIT_B1) == 0) {
		command = 1;
	} else if (bit_is_set(BUTTON_PIN_B, BUTTON_BIT_B2) == 0) {
		command = 2;
	}
	switch (command) {
		case 0: if (lastcommand == 1) {
					// verifica se é um ponto ou um traco
					tx0 <<= 1; // onde existe tracos
					tx1 ++;  // tamanho
					if (timerDelay > DOT_TIME) { // 0,3 segundos é um ponto
						tx0 |= 1;
					}
					// reinicia contador
					timerDelay = 0;
				} else if (lastcommand == 2) {
					// verifica se é um ponto ou um traco
					if (timerDelay < DOT_TIME) { // 0,3 segundos é um ponto
						nextkey = KEY_PERIOD;
					} else {
						nextkey = KEY_SLASH;
					}
					keyToSend = 2;
					// reinicia contador
					timerDelay = 0;
				} else if (lastcommand == 0) {
					// verifica o timeout
					if (timerDelay > WAIT_TIME) { //1 segundo
						// se timeout, define a tecla a enviar
						switch (tx0) {
							case 0: switch (tx1) {
										case 1: nextkey = KEY_E;
												break;
										case 2: nextkey = KEY_I;
												break;		
										case 3: nextkey = KEY_S;
												break;
										case 4: nextkey = KEY_H;
												break; 
										case 5: nextkey = KEY_5;
												break;
										case 6: nextkey = KEY_ENTER;
												break;		
										default: nextkey = 0;
									}
									break;
							case 1: switch (tx1) {
										case 1: nextkey = KEY_T;
												break;
										case 2: nextkey = KEY_A;
												break;		
										case 3: nextkey = KEY_U;
												break;
										case 4: nextkey = KEY_V;
												break; 
										case 5: nextkey = KEY_4;
												break;
										case 6: nextkey = KEY_DELETE; // CTRL + ALT + DEL
												keymod = KEY_CTRL + KEY_ALT;
												break;		
										default: nextkey = 0;
									}
									break;
							case 2: switch (tx1) {
										case 2: nextkey = KEY_N;
												break;		
										case 3: nextkey = KEY_R;
												break;
										case 4: nextkey = KEY_F;
												break;		
										default: nextkey = 0;
									}
									break;
							case 3: switch (tx1) {
										case 2: nextkey = KEY_M;
												break;		
										case 3: nextkey = KEY_W;
												break;
										case 4: nextkey = KEY_SPACE;
												break;
										case 5: nextkey = KEY_3;
												break;
										case 6: nextkey = KEY_7;
												keymod = KEY_SHIFT;
												break;				
										default: nextkey = 0;
									}
									break;
							case 4: switch (tx1) {
										case 3: nextkey = KEY_D;
												break;		
										case 4: nextkey = KEY_L;
												break;
										default: nextkey = 0;
									}
									break;
							case 5: switch (tx1) {
										case 3: nextkey = KEY_K;
												break;		
										case 4: nextkey = KEY_C;
												break;	
										default: nextkey = 0;
									}
									break;
							case 6: switch (tx1) {
										case 3: nextkey = KEY_G;
												break;		
										case 4: nextkey = KEY_P;
												break;
										default: nextkey = 0;
									}
									break;
							case 7: switch (tx1) {
										case 3: nextkey = KEY_O;
												break;		
										case 4: nextkey = KEY_J;
												break;
										case 5: nextkey = KEY_2;
												break;		
										default: nextkey = 0;
									}
									break;
							case 8: switch (tx1) {
										case 4: nextkey = KEY_B;
												break;
										case 5: nextkey = KEY_6;
												keymod = KEY_SHIFT;
												break;		
										default: nextkey = 0;
									}
									break;
							case 9: switch (tx1) {
										case 4: nextkey = KEY_X;
												break;
										case 7: nextkey = KEY_4;
												keymod = KEY_SHIFT;
												break;		
										default: nextkey = 0;
									}
									break;
							case 10: switch (tx1) {
										case 4: nextkey = KEY_C;
												break;
										default: nextkey = 0;
									}
									break;
							case 11: switch (tx1) {
										case 4: nextkey = KEY_Y;
												break;
										default: nextkey = 0;
									}
									break;
							case 12: switch (tx1) {
										case 4: nextkey = KEY_Z;
												break;
										case 6: nextkey = KEY_MINUS;
												keymod = KEY_SHIFT;
												break;		
										default: nextkey = 0;
									}
									break;
							case 13: switch (tx1) {
										case 4: nextkey = KEY_Q;
												break;
										case 6: nextkey = KEY_SLASH;
												keymod = KEY_SHIFT;
												break;		
										default: nextkey = 0;
									}
									break;
							case 14: switch (tx1) {
										case 4: nextkey = KEY_J;
												break;
										default: nextkey = 0;
									}
									break;
							case 15:switch (tx1) {
										case 5: nextkey = KEY_1;
												break;
										default: nextkey = 0;
									}
									break;		
							case 16:nextkey = KEY_6;
									break;
							case 17:switch (tx1) {
										case 5: nextkey = KEY_0;
												keymod = KEY_SHIFT;
												break;
										default: nextkey = 0;
									}
									break;		
							case 18:switch (tx1) {
										case 5: nextkey = KEY_7;
												keymod = KEY_SHIFT;
												break;
										case 6: nextkey = KEY_2;
												keymod = KEY_SHIFT;
												break;		
										default: nextkey = 0;
									}
									break;		
							case 21:switch (tx1) {
										case 6: nextkey = KEY_PERIOD;
												break;
										default: nextkey = 0;
									}
									break;
							case 20:switch (tx1) {
										case 5: nextkey = KEY_SEMICOLON; //ç
												break;
										default: nextkey = 0;
									}
									break;		
							case 22:switch (tx1) {
										case 5: nextkey = KEY_8;
												keymod = KEY_SHIFT;
												break;
										default: nextkey = 0;
									}
									break;						
							case 24:nextkey = KEY_7;
									break;		
							case 28:nextkey = KEY_8;
									break;
							case 26:switch (tx1) {
										case 6: nextkey = KEY_2;
												keymod = KEY_RIGHT_ALT;
												break;
										default: nextkey = 0;
									}
									break;				
							case 30:switch (tx1) {
										case 5: nextkey = KEY_9;
												break;
										case 6: nextkey = KEY_MINUS;
												break;		
										default: nextkey = 0;
									}
									break;
							case 31:nextkey = KEY_0;
									break;
							case 33:switch (tx1) {
										case 6: nextkey = KEY_SLASH;
												break;
										default: nextkey = 0;
									}
									break;		
							case 42:switch (tx1) {
										case 6: nextkey = KEY_COMMA; /* ; */
												keymod = KEY_SHIFT;
												break;
										default: nextkey = 0;
									}
									break;		
							case 43:switch (tx1) {
										case 6: nextkey = KEY_1; /* ! */
												keymod = KEY_SHIFT;
												break;
										default: nextkey = 0;
									}
									break;
							case 45:switch (tx1) {
										case 6: nextkey = KEY_9; /* ) */
												keymod = KEY_SHIFT;
												break;
										default: nextkey = 0;
									}
									break;				
							case 51:switch (tx1) {
										case 6: nextkey = KEY_COMMA; /* \ */
												break;
										default: nextkey = 0;
									}
									break;
							case 120:switch (tx1) {
										case 7: nextkey = KEY_PERIOD; /* : */
												keymod = KEY_SHIFT;
												break;
										default: nextkey = 0;
									}
									break;				
													
							default: nextkey = 0;						
						}
						if (nextkey != 0) {
							keyToSend = 2;
							timerDelay = 0;
						}
						tx0 = 0;
						tx1 = 0;
					}
				}
				break;
		case 1: if (lastcommand == 0) {
					// mudou para pressionado, inicia contador
					timerDelay = 0;
				}
				break;
		case 2: if (lastcommand == 0) {
					timerDelay = 0;
				} 
				break;
	}
	timerDelay++;
	lastcommand = command;  

}

static void buildReport(void)
{
	reportBuffer[0] = keymod;    
	reportBuffer[1] = nextkey;
	
}

static void verifyCommand(void)
{
	if(TIFR & (1 << TOV1)){
		TIFR = (1 << TOV1); /* clear overflow */

		checkButtonChange();

	}
}

/* ------------------------------------------------------------------------- */

static void timerInit(void)
{
    TCCR1 = 0x0b;           /* select clock: 16.5M/1k -> overflow rate = 16.5M/256k = 62.94 Hz */
}


/* ------------------------------------------------------------------------- */
/* ------------------------ interface to USB driver ------------------------ */
/* ------------------------------------------------------------------------- */

uchar	usbFunctionSetup(uchar data[8])
{
usbRequest_t    *rq = (void *)data;

	usbMsgPtr = reportBuffer;
	if((rq->bmRequestType & USBRQ_TYPE_MASK) == USBRQ_TYPE_CLASS){    /* class request type */
		if(rq->bRequest == USBRQ_HID_GET_REPORT){  /* wValue: ReportType (highbyte), ReportID (lowbyte) */
			/* we only have one report type, so don't look at wValue */
			buildReport();
			return sizeof(reportBuffer);
		} else if(rq->bRequest == USBRQ_HID_GET_IDLE) {
			usbMsgPtr = &idleRate;
			return 1;
		} else if(rq->bRequest == USBRQ_HID_SET_IDLE){
			idleRate = rq->wValue.bytes[1];
		}
	} else{
		/* no vendor specific requests implemented */
	}
	return 0;
}

/* ------------------------------------------------------------------------- */
/* ------------------------ Oscillator Calibration ------------------------- */
/* ------------------------------------------------------------------------- */

/* Calibrate the RC oscillator to 8.25 MHz. The core clock of 16.5 MHz is
 * derived from the 66 MHz peripheral clock by dividing. Our timing reference
 * is the Start Of Frame signal (a single SE0 bit) available immediately after
 * a USB RESET. We first do a binary search for the OSCCAL value and then
 * optimize this value with a neighboorhod search.
 * This algorithm may also be used to calibrate the RC oscillator directly to
 * 12 MHz (no PLL involved, can therefore be used on almost ALL AVRs), but this
 * is wide outside the spec for the OSCCAL value and the required precision for
 * the 12 MHz clock! Use the RC oscillator calibrated to 12 MHz for
 * experimental purposes only!
 */
static void calibrateOscillator(void)
{
	uchar       step = 128;
	uchar       trialValue = 0, optimumValue;
	int         x, optimumDev, targetValue = (unsigned)(1499 * (double)F_CPU / 10.5e6 + 0.5);

	/* do a binary search: */
	do{
		  OSCCAL = trialValue + step;
		  x = usbMeasureFrameLength();    /* proportional to current real frequency */
		  if(x < targetValue)  {           /* frequency still too low */
		    trialValue += step;
			}
		  step >>= 1;
	}while(step > 0);
	/* We have a precision of +/- 1 for optimum OSCCAL here */
	/* now do a neighborhood search for optimum value */
	optimumValue = trialValue;
	optimumDev = x; /* this is certainly far away from optimum */
	for(OSCCAL = trialValue - 1; OSCCAL <= trialValue + 1; OSCCAL++){
	  x = usbMeasureFrameLength() - targetValue;
	  if(x < 0) {
	  	x = -x;
		}
	  if(x < optimumDev){
	  	optimumDev = x;
	  	optimumValue = OSCCAL;
	  }
	}
	OSCCAL = optimumValue;
}
/*
Note: This calibration algorithm may try OSCCAL values of up to 192 even if
the optimum value is far below 192. It may therefore exceed the allowed clock
frequency of the CPU in low voltage designs!
You may replace this search algorithm with any other algorithm you like if
you have additional constraints such as a maximum CPU clock.
For version 5.x RC oscillators (those with a split range of 2x128 steps, e.g.
ATTiny25, ATTiny45, ATTiny85), it may be useful to search for the optimum in
both regions.
*/

void    usbEventResetReady(void)
{
    calibrateOscillator();
    eeprom_write_byte(0, OSCCAL);   /* store the calibrated value in EEPROM */
}

/* ------------------------------------------------------------------------- */
/* --------------------------------- main ---------------------------------- */
/* ------------------------------------------------------------------------- */



int main(void)
{
	uchar   i;
	uchar   calibrationValue;

    calibrationValue = eeprom_read_byte(0); /* calibration value from last time */
    if(calibrationValue != 0xff){
        OSCCAL = calibrationValue;
    }
    //odDebugInit();	
    usbDeviceDisconnect();
    for(i=0;i<20;i++){  /* 300 ms disconnect */
        _delay_ms(15);
    }
    usbDeviceConnect();

    wdt_enable(WDTO_1S);
    
    /* turn on internal pull-up resistor for the switches */
    BUTTON_PORT_B |= _BV(BUTTON_BIT_B1);
		BUTTON_PORT_B |= _BV(BUTTON_BIT_B2);
		BUTTON_PORT_B |= _BV(BUTTON_BIT_B3);
    
    timerInit();
	
    usbInit();
    sei();
    for(;;){    /* main event loop */
        wdt_reset();
        usbPoll();

			/* A USB keypress cycle is defined as a scancode being present in a report, and
			then absent from a later report. To press and release the Caps Lock key, instead of
			holding it down, we need to send the report with the Caps Lock scancode and
			then an empty report. */

			if (keyToSend > 0) {
				if(usbInterruptIsReady()){ /* we can send another key */
					buildReport();
					usbSetInterrupt(reportBuffer, sizeof(reportBuffer));
					keyToSend--;
					nextkey = 0;
					keymod = 0; /* no modifiers */
				}
			} else {
				verifyCommand();
			}
		}
    return 0;
}
