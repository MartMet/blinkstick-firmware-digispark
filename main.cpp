/* Name: main.c
 * Project: BlinkStick
 * Author: Arvydas Juskevicius
 * Creation Date: 2013-04-02
 * Tabsize: 4
 * Copyright: (c) 2013 by Agile Innovative Ltd
 * License: GNU GPL v2 (see License.txt), GNU GPL v3 or proprietary (CommercialLicense.txt)
 *
 * Modified 22-06-2019 - Eric Draken - Added a LED inactivity shutdown timeout and pattern buffer
 */

#define USB_CFG_DEVICE_NAME         'B', 'l', 'i', 'n', 'k', 'S', 't', 'i', 'c', 'k'
#define USB_CFG_DEVICE_NAME_LEN     10

#define USB_CFG_SERIAL_NUMBER       'B', 'S', '0', '0', '0', '0', '0', '0', '-', '1', '.', '5'
#define USB_CFG_SERIAL_NUMBER_LEN   12

/*
 * Turn the LED off if this much time has elapsed without a USB write being registered.
 * This is useful if the LED is being used as a status indicator, say green, but the
 * CPU has gone away. The LED should turn off to indicate it is no longer functioning.
 */
#define LED_SHUTDOWN_TIMEOUT_MS     60000L

#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/eeprom.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <util/atomic.h>
#include <avr/pgmspace.h>   /* required by usbdrv.h */
extern "C"
{
    #include "usbdrv.h"
}
extern "C"
{
    #include "light_ws2812.h"
}

/* ------------------------------------------------------------------------- */
/* ----------------------------- USB interface ----------------------------- */
/* ------------------------------------------------------------------------- */

// NOTE: if descriptor changes, USB_CFG_HID_REPORT_DESCRIPTOR_LENGTH also has to be updated in usbconfig.h

#define PATTERN_BUFFER_LEN (uint8_t)(1 + 1 + (62*4))         // (count + (r,g,b,delay)*10)

// Swap R and G if you have an inverted LED
#define R_IDX 0
#define G_IDX 1
#define B_IDX 2

static uchar currentAddress;
static uchar bytesRemaining;
static uchar reportId = 0;

static uchar replyBuffer[33];           // 32 for data + 1 for report id

static volatile unsigned long lastUpdateMs = 0;

static volatile bool isBusy = false;

static volatile uint8_t patternCount = 0;
static uchar patternBuffer[PATTERN_BUFFER_LEN] = {};

/* USB report descriptor */
const PROGMEM char usbHidReportDescriptor[USB_CFG_HID_REPORT_DESCRIPTOR_LENGTH] = {
        '\x06', '\x00', '\xff',          // (GLOBAL) USAGE_PAGE
        '\x09', '\x01',                  // (GLOBAL) USAGE (Vendor Usage 1)
        '\xa1', '\x01',                  // (MAIN) COLLECTION
        '\x15', '\x00',                  //   (GLOBAL) LOGICAL_MINIMUM (0)
        '\x26', '\xff', '\x00',          //   (GLOBAL) LOGICAL_MAXIMUM (255)
        '\x75', '\x08',                  //   (GLOBAL) REPORT_SIZE (8) - 8 bits per field

        '\x85', '\x01',                  //   REPORT_ID (1)
        '\x95', '\x03',                  //   REPORT_COUNT (3) - 3 fields for R,G,B
        '\x09', '\x00',                  //   USAGE (Undefined)
        '\xb2', '\x02', '\x01',          //   FEATURE (Data,Var,Abs,Buf)

        '\x85', '\x14',                  //   REPORT_ID (20)
        '\x95', '\xFA', // PATTERN_BUFFER_LEN,      //   REPORT_COUNT (xx)
        '\x09', '\x00',                  //   USAGE (Undefined)
        '\xb2', '\x02', '\x01',          //   FEATURE (Data,Var,Abs,Buf)
        '\xc0'                           // (MAIN) END_COLLECTION
};

static unsigned long millis();

/* ------------------------------------------------------------------------- */
/* ---------------------------- Helper methods ----------------------------- */
/* ------------------------------------------------------------------------- */

/* Simple non-blocking, responsive delay */
void delayMs(unsigned ms) {
    for (
            unsigned i = 0; /* requires at least C99 */
            i < ms;
            i++) {
        _delay_ms(1);
        usbPoll();
    }
}

// Quick, gentle green pulse
void pulse(uint8_t count) {
    for (uint8_t i = 0; i < count; i++) {
        uint8_t led[3];
        led[0] = 0;
        led[1] = 32;
        led[2] = 0;

        ws2812_sendarray_mask(&led[0], 3, _BV(ws2812_pin));
        _delay_ms(50);

        led[0] = 0;
        led[1] = 0;
        led[2] = 0;
        ws2812_sendarray_mask(&led[0], 3, _BV(ws2812_pin));
        _delay_ms(50);
    }
}

static inline void resetTransfer() {
    ATOMIC_BLOCK(ATOMIC_FORCEON)
    {
        isBusy = false;
        patternCount = 0;
        bytesRemaining = 0;
        lastUpdateMs = millis();
    }
}

static void calibrateOscillator(void) {
    uchar step = 128;
    uchar trialValue = 0, optimumValue;
    int x, optimumDev, targetValue = (unsigned) (1499 * (double) F_CPU / 10.5e6 + 0.5);

    /* do a binary search: */
    do {
        OSCCAL = trialValue + step;
        x = usbMeasureFrameLength();    // proportional to current real frequency
        if (x < targetValue)             // frequency still too low
            trialValue += step;
        step >>= 1;
    } while (step > 0);
    /* We have a precision of +/- 1 for optimum OSCCAL here */
    /* now do a neighborhood search for optimum value */
    optimumValue = trialValue;
    optimumDev = x; // this is certainly far away from optimum
    for (OSCCAL = trialValue - 1; OSCCAL <= trialValue + 1; OSCCAL++) {
        x = usbMeasureFrameLength() - targetValue;
        if (x < 0)
            x = -x;
        if (x < optimumDev) {
            optimumDev = x;
            optimumValue = OSCCAL;
        }
    }
    OSCCAL = optimumValue;
}

/* ------------------------------------------------------------------------- */
/* ------------------------- Dynamic USB methods --------------------------- */
/* ------------------------------------------------------------------------- */

/* usbFunctionWrite() is called when the host sends a chunk of data to the device. */
uchar usbFunctionWrite(uchar *data, uchar len)
{
    // Single color
    if (reportId == 1)
    {
        // Save the last color for report inspection
        patternBuffer[0] = data[0];     // Report id
        patternBuffer[1] = 0;           // One pattern
        patternBuffer[2] = data[1];     // Red
        patternBuffer[3] = data[2];     // Green
        patternBuffer[4] = data[3];     // Blue
        patternBuffer[5] = 0;           // Delay

        // Set the LED immediately
        ws2812_sendarray_mask(&data[1], 3, _BV(ws2812_pin));
    
        return 1;
    }

    // Color pattern
    else if (reportId == 20)
    {
        if ( bytesRemaining == 0 ) {
            isBusy = true;
            return 1;
        }

        if(len > bytesRemaining)
            len = bytesRemaining;

        if(currentAddress == 0)
            patternCount = data[1]; // Second byte is the count

        for( uint8_t i = 0; i < len; i++ )
            patternBuffer[currentAddress + i] = data[i];

        currentAddress += len;
        bytesRemaining -= len;

        if ( bytesRemaining == 0 )
            isBusy = true;

        return bytesRemaining == 0; // end of transfer
    }
    else
    {
        return 1;
    }
}

extern "C" usbMsgLen_t usbFunctionSetup(uchar data[8])
{
    lastUpdateMs = millis();    // For the inactivity timeout

    usbRequest_t *rq = (usbRequest_t *)data;
    reportId = rq->wValue.bytes[0];

    if((rq->bmRequestType & USBRQ_TYPE_MASK) == USBRQ_TYPE_CLASS){ /* HID class request */
        if(rq->bRequest == USBRQ_HID_GET_REPORT){ /* wValue: ReportType (highbyte), ReportID (lowbyte) */
            usbMsgPtr = replyBuffer;
            if(reportId == 1){ // Device colors
                replyBuffer[0] = 1; // report id
                replyBuffer[1] = patternBuffer[2];
                replyBuffer[2] = patternBuffer[3];
                replyBuffer[3] = patternBuffer[4];
                return 4;
            } else if (reportId == 20) { // Get pattern
                usbMsgPtr = patternBuffer;
                return PATTERN_BUFFER_LEN > rq->wLength.word ? rq->wLength.word : PATTERN_BUFFER_LEN;
            }
            return 0;
        }else if(rq->bRequest == USBRQ_HID_SET_REPORT){
            // Do not try to set anything, but stay responsive
            if(isBusy)
                return 0;

            if(reportId == 1){ // Device colors
                bytesRemaining = 3;
                currentAddress = 0;
                return USB_NO_MSG; /* use usbFunctionWrite() to receive data from host */
            } else if (reportId == 20) { // Set pattern
                currentAddress = 0;
                bytesRemaining = (rq->wLength.word > PATTERN_BUFFER_LEN ? PATTERN_BUFFER_LEN : rq->wLength.word);
                return USB_NO_MSG;
            }
            return 0;
        }
    }

    return 0;   /* default for not implemented requests: return no data back to host */
}

extern "C" void usbEventResetReady(void)
{
    // usbMeasureFrameLength() counts CPU cycles, so disable interrupts.
    ATOMIC_BLOCK(ATOMIC_FORCEON)
    {
        calibrateOscillator();
        eeprom_write_byte(0, OSCCAL);   // store the calibrated value in EEPROM
    }
}

/* ------------------------------------------------------------------------- */
/* ------------------------------- millis() -------------------------------- */
/* ------------------------------------------------------------------------- */

/*
  wiring.c - Partial implementation of the Wiring API for the ATmega8.
  Part of Arduino - http://www.arduino.cc/

  Copyright (c) 2005-2006 David A. Mellis

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General
  Public License along with this library; if not, write to the
  Free Software Foundation, Inc., 59 Temple Place, Suite 330,
  Boston, MA  02111-1307  USA

  $Id: wiring.c 970 2010-05-25 20:16:15Z dmellis $

  Modified 28-08-2009 for attiny84 R.Wiersma
  Modified 14-10-2009 for attiny45 Saposoft
  Modified 20-11-2010 - B.Cook - Rewritten to use the various Veneers.
  Modified 22-06-2019 - Eric Draken - Distilled down to a simple millis() library for the ATTiny85
*/

/*
  For various reasons, Timer 1 is a better choice for the millis timer on the
  '85 processor.
*/

// core_macros.h
#ifndef MASK3
#define MASK3(b1,b2,b3)             ( (1<<b1) | (1<<b2) | (1<<b3) )
#define MASK6(b1,b2,b3,b4,b5,b6)    ( (1<<b1) | (1<<b2) | (1<<b3) | (1<<b4) | (1<<b5) | (1<<b6) )
#endif

#if F_CPU >= 3000000L
#define Timer1_Prescale_Index  (7)  /* B0111 */
    #define Timer1_Prescale_Value  (64)
#else
#define Timer1_Prescale_Index  (4)  /* B0100 */
#define Timer1_Prescale_Value  (8)
#endif

#define clockCyclesToMicroseconds(a) ( ((a) * 1000L) / (F_CPU / 1000L) )

// the prescaler is set so that the millis timer ticks every Timer1_Prescale_Value (64) clock cycles, and the
// the overflow handler is called every 256 ticks.
#define MICROSECONDS_PER_MILLIS_OVERFLOW (clockCyclesToMicroseconds(Timer1_Prescale_Value * 256))

// the whole number of milliseconds per millis timer overflow
#define MILLIS_INC (MICROSECONDS_PER_MILLIS_OVERFLOW / 1000)

// the fractional number of milliseconds per millis timer overflow. we shift right
// by three to fit these numbers into a byte. (for the clock speeds we care
// about - 8 and 16 MHz - this doesn't lose precision.)
#define FRACT_INC ((MICROSECONDS_PER_MILLIS_OVERFLOW % 1000) >> 3)
#define FRACT_MAX (1000 >> 3)

volatile unsigned long millis_timer_overflow_count = 0;
volatile unsigned long millis_timer_millis = 0;
static unsigned char millis_timer_fract = 0;

ISR(TIMER1_OVF_vect, ISR_NOBLOCK)
{
    // copy these to local variables so they can be stored in registers
    // (volatile variables must be read from memory on every access)
    unsigned long m = millis_timer_millis;
    unsigned char f = millis_timer_fract;

    f += FRACT_INC;

    if (f >= FRACT_MAX)
    {
        f -= FRACT_MAX;
        m = m + MILLIS_INC + 1;
    }
    else
    {
        m += MILLIS_INC;
    }

    millis_timer_fract = f;
    millis_timer_millis = m;
    millis_timer_overflow_count++;
}

static unsigned long millis()
{
    unsigned long m;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        m = millis_timer_millis;
    }
    
    return m;
}

static inline void Timer1_SetToPowerup( void )
{
    // Turn off Clear on Compare Match, turn off PWM A, disconnect the timer from the output pin, stop the clock
    TCCR1 = (0<<CTC1) | (0<<PWM1A) | (0<<COM1A1) | (0<<COM1A0) | (0<<CS13) | (0<<CS12) | (0<<CS11) | (0<<CS10);
    // Turn off PWM A, disconnect the timer from the output pin, no Force Output Compare Match, no Prescaler Reset
    GTCCR &= ~MASK6(PWM1B,COM1B1,COM1B0,FOC1B,FOC1A,PSR1);
    // Reset the count to zero
    TCNT1 = 0;
    // Set the output compare registers to zero
    OCR1A = 0;
    OCR1B = 0;
    OCR1C = 0;
    // Disable all Timer1 interrupts
    TIMSK &= ~MASK3(OCIE1A,OCIE1B,TOIE1);
    // Clear the Timer1 interrupt flags
    TIFR |= MASK3(OCF1A,OCF1B,TOV1);
}

static inline void Timer1_ClockSelect( unsigned int cs )
{
    TCCR1 = (TCCR1 & ~MASK3(CS12,CS11,CS10)) | (cs << CS10);
}

static inline void Timer1_EnableOverflowInterrupt( void )
{
    TIMSK |= (1<<TOIE1);
}

/* ------------------------------------------------------------------------- */
/* ------------------------------ Main method ------------------------------ */
/* ------------------------------------------------------------------------- */

int main(void)
{
    cli();

    wdt_disable();  // Not needed. There is an inactivity timeout now.

    usbInit();

    usbDeviceDisconnect();  /* enforce re-enumeration, do this while interrupts are disabled! */
    uchar i = 0;
    while(--i){             /* fake USB disconnect for > 500 ms */
        _delay_ms(2);
    }
    usbDeviceConnect();

    USB_DDRPORT(ws2812_port) |= _BV(ws2812_pin);

    // This needs to be called before setup() or some functions won't work
    sei();

    // In case the bootloader left our millis timer in a bad way
    Timer1_SetToPowerup();

    // Millis timer is always processor clock divided by Timer1_Prescale_Value (64)
    Timer1_ClockSelect( Timer1_Prescale_Index );

    // Enable the overflow interrupt (this is the basic system tic-toc for millis)
    Timer1_EnableOverflowInterrupt();

    // Pulse the LED on startup to indicate the device has entered the main loop
    pulse(2);

    /* Main event loop */
    for(;;){
        usbPoll();

        // Process any color patterns
        if(isBusy) {
            for ( uint8_t currPatternIndex = 0; currPatternIndex < patternCount; currPatternIndex++ ) {

                uint8_t led[3];
                led[R_IDX] = patternBuffer[ 2 + (4 * currPatternIndex) + 0 ];
                led[G_IDX] = patternBuffer[ 2 + (4 * currPatternIndex) + 1 ];
                led[B_IDX] = patternBuffer[ 2 + (4 * currPatternIndex) + 2 ];
                unsigned int delay = patternBuffer[ 2 + (4 * currPatternIndex) + 3 ] * 10;    // Restore the delay resolution

                // Set the LED color
                cli();
                ws2812_sendarray_mask(&led[0], 3, _BV(ws2812_pin));
                sei();

                if (delay > 0)
                    delayMs(delay); // This will also call usbPoll() repeatedly for responsive IO
            }
            resetTransfer();
        }

        // Turn off the LED after long no activity
        if ( millis() - lastUpdateMs >= LED_SHUTDOWN_TIMEOUT_MS ) {
            pulse(1);

            // Turn off the LED
            uint8_t led[3] = {};
            ws2812_sendarray_mask(&led[0], 3, _BV(ws2812_pin));

            // Just in case there is a runaway
            resetTransfer();
        }
    }
    return 0;
}
