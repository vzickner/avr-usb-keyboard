/* Name: main.c
 * Project: hid-mouse, a very simple HID example
 * Author: Christian Starkjohann
 * Creation Date: 2008-04-07
 * Tabsize: 4
 * Copyright: (c) 2008 by OBJECTIVE DEVELOPMENT Software GmbH
 * License: GNU GPL v2 (see License.txt), GNU GPL v3 or proprietary (CommercialLicense.txt)
 * This Revision: $Id: main.c 692 2008-11-07 15:07:40Z cs $
 */

/*
This example should run on most AVRs with only little changes. No special
hardware resources except INT0 are used. You may have to change usbconfig.h for
different I/O pins for USB. Please note that USB D+ must be the INT0 pin, or
at least be connected to INT0 as well.

We use VID/PID 0x046D/0xC00E which is taken from a Logitech mouse. Don't
publish any hardware using these IDs! This is for demonstration only!
*/

#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>  /* for sei() */
#include <util/delay.h>     /* for _delay_ms() */

#include <avr/pgmspace.h>   /* required by usbdrv.h */
#include "usbdrv.h"
#include "oddebug.h"        /* This is also an example for using debug macros */
#include "keys.h"

#define XTAL        16e6                 // 16 MHz

#define PHASE_A     (PINC & 1<<PC5)     // an Pinbelegung anpassen
#define PHASE_B     (PINC & 1<<PC6)     // an Pinbelegung anpassen

volatile int8_t enc_delta;              // Drehgeberbewegung zwischen
                                        // zwei Auslesungen im Hauptprogramm

// Drehgeber
// Dekodertabelle für wackeligen Rastpunkt
// halbe Auflösung
const int8_t table[16] PROGMEM = {0,0,-1,0,0,0,0,1,1,0,0,0,0,-1,0,0};    
 
// Dekodertabelle für normale Drehgeber
// volle Auflösung
//const int8_t table[16] PROGMEM = {0,1,-1,0,-1,0,0,1,1,0,0,-1,0,-1,1,0};    
volatile uint8_t rotatingSet = 0;


/* ------------------------------------------------------------------------- */
/* ----------------------------- USB interface ----------------------------- */
/* ------------------------------------------------------------------------- */

PROGMEM char usbHidReportDescriptor[35] = {   /* USB report descriptor */
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

/* This is the same report descriptor as seen in a Logitech mouse. The data
 * described by this descriptor consists of 4 bytes:
 *      .  .  .  .  . B2 B1 B0 .... one byte with mouse button states
 *     X7 X6 X5 X4 X3 X2 X1 X0 .... 8 bit signed relative coordinate x
 *     Y7 Y6 Y5 Y4 Y3 Y2 Y1 Y0 .... 8 bit signed relative coordinate y
 *     W7 W6 W5 W4 W3 W2 W1 W0 .... 8 bit signed relative coordinate wheel
 */
typedef struct{
	uchar mod;
	uchar key;
}report_t;

static report_t reportBuffer;
static int      sinus = 7 << 6, cosinus = 0;
static uchar    idleRate;   /* repeat rate for keyboards, never used for mice */
volatile uint8_t test = 0;
volatile uint16_t counter = 0;
volatile uint16_t counterRotate = 0;

usbMsgLen_t usbFunctionSetup(uchar data[8]) {
	usbRequest_t    *rq = (void *)data;

	/* The following requests are never used. But since they are required by
	 * the specification, we implement them in this example.
	 */
	if((rq->bmRequestType & USBRQ_TYPE_MASK) == USBRQ_TYPE_CLASS){    /* class request type */
		DBG1(0x50, &rq->bRequest, 1);   /* debug output: print our request */
		if(rq->bRequest == USBRQ_HID_GET_REPORT){  /* wValue: ReportType (highbyte), ReportID (lowbyte) */
			/* we only have one report type, so don't look at wValue */
			usbMsgPtr = (void *)&reportBuffer;
			return sizeof(reportBuffer);
		} else if(rq->bRequest == USBRQ_HID_GET_IDLE) {
			usbMsgPtr = &idleRate;
			return 1;
		} else if(rq->bRequest == USBRQ_HID_SET_IDLE) {
			idleRate = rq->wValue.bytes[1];
		}
	} else {
		/* no vendor specific requests implemented */
	}
	return 0;   /* default for not implemented requests: return no data back to host */
}


ISR(TIMER0_OVF_vect) {
	counter++;
	counterRotate++;
	uint8_t oldSet = rotatingSet;
	rotatingSet = PHASE_A | PHASE_B;

	if (oldSet > 0 && rotatingSet > 0 && rotatingSet != oldSet) {
		if (oldSet > rotatingSet) {
			//reportBuffer.key = KEY_Y;
		} else {
			//reportBuffer.key = KEY_W;
		}
	}
}

int8_t encode_read( void )         // Encoder auslesen
{
	int8_t val;
 
	// atomarer Variablenzugriff  
	cli();
	val = enc_delta;
	enc_delta = 0;
	sei();
	return val;
}

int main(void) {
	uchar i;

	wdt_enable(WDTO_1S);
	/* Even if you don't use the watchdog, turn it off here. On newer devices,
	 * the status of the watchdog (on/off, period) is PRESERVED OVER RESET!
	 */
	DBG1(0x00, 0, 0);       /* debug output: main starts */
	/* RESET status: all port bits are inputs without pull-up.
	 * That's the way we need D+ and D-. Therefore we don't need any
	 * additional hardware initialization.
	 */
	odDebugInit();
	usbInit();
	usbDeviceDisconnect();  /* enforce re-enumeration, do this while interrupts are disabled! */
	i = 0;
	while (--i) {             /* fake USB disconnect for > 250 ms */
		wdt_reset();
		_delay_ms(1);
	}
	usbDeviceConnect();

	// Enable timer0
	TCCR0 = (1<<CS10) | (1<<CS01) | (1<<CS00);     // CTC, XTAL / 64
	TIMSK |= (1<<TOIE0);

	sei();
	DBG1(0x01, 0, 0);       /* debug output: main loop starts */
	DDRC = 0x00;
	DDRA = 0xff;
	PORTA = 0x00;

	int8_t rotateValue = 0;
	uint8_t pinC = 0x1F;
	uint8_t last = 0x1F;
	uint8_t history[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
	uint8_t first = 1;
	uint8_t use, j;
	uint8_t historyPosition = 0;
	uint8_t beep = 0;
	uint8_t sendStack[32] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
	uint8_t sendStackSend = 0;
	uint8_t sendStackAdd = 0;

	uint8_t rotateLast = 0x60;
	uint8_t rotateC = 0;

	for (;;) {                /* main event loop */
		DBG1(0x02, 0, 0);   /* debug output: main loop iterates */
		wdt_reset();
		usbPoll();

		//reportBuffer.mod = MOD_SHIFT_LEFT;
		//reportBuffer.key = KEY_A + (PINC & 0x0F);

		if (counter > 0x1FF && counter < 0x2FF && beep == 1) {
			PORTA |= (1<<PA6);
		} else {
			PORTA &= (0<<PA6);
		}

		history[(historyPosition++) & 0xF] = (PINC & 0x1F);
		first = history[(historyPosition+1) & 0xF];
		use = 1;
		for (j=1; j < 16; j++) {
			if (history[(historyPosition+1+j) & 0xF] != first) {
				use = 0;
				break;
			}
		}
		if (use == 1) {
			last = first;
			if (counter > 0x1FF && beep == 0) {
				if (last == 0x1C) {
					PORTA |= (1<<PA6);
					beep = 1;
					PORTA &= (0<<PA7);
					_delay_ms(1);
					_delay_ms(1);
					_delay_ms(1);
					PORTA |= (1<<PA7);
				}
				if (last == 0x13) {
					PORTA |= (1<<PA6);
					beep = 1;
					sendStack[sendStackAdd] = 0x50;
					sendStackAdd = (sendStackAdd+1)&0x03;
				}
			}
		}
		if (last != pinC) {
			sendStack[sendStackAdd] = KEY_A;
			if (pinC != 0x1F) {
//			PORTA |= (1<<PA6);
				if (counter < 0x1FF) {
					sendStack[sendStackAdd] = (pinC & 0x1F);
				} else {
					sendStack[sendStackAdd] = 0x20 + (pinC & 0x1F);
				}
				sendStackAdd = (sendStackAdd+1)&0x03;
				beep = 0;
			} else {
				beep = 1;
			}
			pinC = last;
			counter = 0;
		}

		rotateC = PINC & 0x60;
		if (rotateC != rotateLast) {
			if (rotateLast == 0x40 && rotateC == 0x60) {
				sendStack[sendStackAdd] = 0x40;
				sendStackAdd = (sendStackAdd+1)&0x03;
			} else if (rotateLast == 0x20 && rotateC == 0x60) {
				sendStack[sendStackAdd] = 0x41;
				sendStackAdd = (sendStackAdd+1)&0x03;
			}
			rotateLast = rotateC;
		}

		if(usbInterruptIsReady()) {
			reportBuffer.mod = 0;
			reportBuffer.key = 0;

			if (sendStackSend != sendStackAdd) {
				switch (sendStack[sendStackSend]) {
				case 0x1E: // button 1 short
					reportBuffer.mod = MOD_SHIFT_LEFT | MOD_CONTROL_LEFT;
					reportBuffer.key = KEY_TAB;
					break;
				case 0x1D: // button 2 short
					reportBuffer.mod = MOD_CONTROL_LEFT;
					reportBuffer.key = KEY_TAB;
					break;
				case 0x1B: // button 3 short
					reportBuffer.mod = 0;
					reportBuffer.key = KEY_UP_ARROW;
					break;
				case 0x17: // button 4 short
					reportBuffer.mod = 0;
					reportBuffer.key = KEY_DOWN_ARROW;
					break;
				case 0x0F: // button 5 short
					reportBuffer.mod = 0;
					reportBuffer.key = KEY_RETURN;
					break;
				case 0x3E: // button 1 long
					reportBuffer.mod = MOD_CONTROL_LEFT;
					reportBuffer.key = KEY_W;
					break;
				case 0x3D: // button 2 long
					reportBuffer.mod = MOD_SHIFT_LEFT | MOD_CONTROL_LEFT;
					reportBuffer.key = KEY_RETURN;
					break;
				case 0x3B: // button 3 long
					reportBuffer.mod = 0;
					reportBuffer.key = KEY_F5;
					break;
				case 0x37: // button 4 long
					reportBuffer.mod = MOD_ALT_LEFT;
					reportBuffer.key = KEY_POS1;
					break;
				case 0x2F: // button 5 long
					reportBuffer.mod = 0;
					reportBuffer.key = KEY_RETURN;
					break;
				case 0x41: // rotate right
					reportBuffer.mod = 0;
					reportBuffer.key = KEY_TAB;
					break;
				case 0x40: // rotate left
					reportBuffer.mod = MOD_SHIFT_LEFT;
					reportBuffer.key = KEY_TAB;
					break;
				case 0x50: // shutdown
					reportBuffer.mod = MOD_CONTROL_LEFT | MOD_ALT_LEFT;
					reportBuffer.key = KEY_BACKSPACE;
					break;
				}
				sendStackSend = (sendStackSend+1)&0x03;
			}
			DBG1(0x03, 0, 0);   /* debug output: interrupt report prepared */
			usbSetInterrupt((void *)&reportBuffer, sizeof(reportBuffer));
		}
	}
	return 0;
}

/* ------------------------------------------------------------------------- */
