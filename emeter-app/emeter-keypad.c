/*******************************************************************************
 *  emeter-keypad.c -
 *
 *  Copyright (C) 2011 Texas Instruments Incorporated - http://www.ti.com/ 
 * 
 *  Redistribution and use in source and binary forms, with or without 
 *  modification, are permitted provided that the following conditions 
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright 
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the 
 *    documentation and/or other materials provided with the   
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************/

/*! \file emeter-structs.h */

#include <inttypes.h>
#include <stdlib.h>
#if !defined(__MSP430__)
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#endif
#if defined(__GNUC__)
#include <signal.h>
#endif
#if defined(__MSP430__)
#include <msp430.h>
#endif
#define __MAIN_PROGRAM__

#include "emeter-template.h"

#include <emeter-toolkit.h>
#include <emeter-metrology.h>

#include "emeter-app.h"
#include "emeter-rtc.h"
#include "emeter-lcd.h"
#include "emeter-basic-display.h"
#include "emeter-keypad.h"

#if defined(IO_EXPANDER_SUPPORT)
/* This routine supports the use of a device like the 74HC595 to expand the number of
   output bits available on the lower pin count MSP430s. */
void set_io_expander(int what, int which)
{
    static uint8_t io_state = 0;
    int i;
    int j;

    if (what < 0)
        io_state &= ~which;
    else if (what > 0)
        io_state |= which;
    else
        io_state = which;
    /* Pump the data into the shift register */
    for (i = 8, j = io_state;  i > 0;  i--)
    {
        P1OUT &= ~BIT4;
        if ((j & 0x80))
            P1OUT |= BIT7;
        else
            P1OUT &= ~BIT7;
        P1OUT |= BIT4;
        j <<= 1;
    }
    /* Clock the data into the output register */
    P1OUT &= ~BIT6;
    P1OUT |= BIT6;
}
#endif

#if defined(__MSP430__)
    #if defined(BASIC_KEYPAD_SUPPORT)
void keypad_handler(void)
{
    if ((key_states & KEY_1_DOWN))
    {
        update_display();
        key_states &= ~KEY_1_DOWN;
    }
    if ((key_states & KEY_1_REPEAT_DOWN))
    {
        update_display();
        key_states &= ~KEY_1_REPEAT_DOWN;
    }
}
    #endif
#endif

/* This keypad debounce code provides for 1 to 4 keys, with debounce + long
   press detect, of debounce + auto-repeat on long press selectable for each
   key. Definitions in emeter.h control this. A long press means >2s.
   Auto-repeat means holding the key >1s starts repeats at 3 per second. */
#if defined(__MSP430__)  &&  (defined(BASIC_KEYPAD_SUPPORT)  ||  defined(CUSTOM_KEYPAD_SUPPORT))
int keypad_debounce(void)
{
    int kick_foreground;
    
    kick_foreground = false;
    #if defined(sense_key_1_up)
    switch (debounce(&debounce_key_1, sense_key_1_up()))
    {
    case DEBOUNCE_JUST_RELEASED:
        key_timer_1 = 0;
        break;
    case DEBOUNCE_JUST_HIT:
        #if defined(KEY_1_LONG_DOWN)
        /* Start a 2s timer to detect mode change request */
        key_timer_1 = samples_per_second << 1;
        #elif defined(KEY_1_REPEAT_DOWN)
        /* Start an initial 1s timeout for repeats */
        key_timer_1 = samples_per_second;
        #endif
        key_states |= KEY_1_DOWN;
        kick_foreground = true;
        break;
    case DEBOUNCE_HIT:
        if (key_timer_1  &&  --key_timer_1 == 0)
        {
        #if defined(KEY_1_LONG_DOWN)
            key_states |= KEY_1_LONG_DOWN;
        #elif defined(KEY_1_REPEAT_DOWN)
            /* Start a 1/3s timeout for repeats */
            #if defined(LIMP_MODE_SUPPORT)
            if (operating_mode == OPERATING_MODE_LIMP)
                key_timer_1 = 273;
            else
            #endif
                key_timer_1 = 1092;
            key_states |= KEY_1_REPEAT_DOWN;
        #endif
            kick_foreground = true;
        }
        break;
    }
    #endif
    #if defined(sense_key_2_up)
    switch (debounce(&debounce_key_2, sense_key_2_up()))
    {
    case DEBOUNCE_JUST_RELEASED:
        key_timer_2 = 0;
        break;
    case DEBOUNCE_JUST_HIT:
        #if defined(KEY_2_LONG_DOWN)
        /* Start a 2s timer to detect mode change request */
        key_timer_2 = samples_per_second << 1;
        #elif defined(KEY_2_REPEAT_DOWN)
        /* Start an initial 1s timeout for repeats */
        key_timer_2 = samples_per_second;
        #endif
        key_states |= KEY_2_DOWN;
        kick_foreground = true;
        break;
    case DEBOUNCE_HIT:
        if (key_timer_2  &&  --key_timer_2 == 0)
        {
        #if defined(KEY_2_LONG_DOWN)
            key_states |= KEY_2_LONG_DOWN;
        #elif defined(KEY_2_REPEAT_DOWN)
            /* Start a 1/3s timeout for repeats */
            key_timer_2 = 1092;
            key_states |= KEY_2_REPEAT_DOWN;
        #endif
            kick_foreground = true;
        }
        break;
    }
    #endif
    #if defined(sense_key_3_up)
    switch (debounce(&debounce_key_3, sense_key_3_up()))
    {
    case DEBOUNCE_JUST_RELEASED:
        key_timer_3 = 0;
        break;
    case DEBOUNCE_JUST_HIT:
        #if defined(KEY_3_LONG_DOWN)
        /* Start a 2s timer to detect mode change request */
        key_timer_3 = samples_per_second << 1;
        #elif defined(KEY_3_REPEAT_DOWN)
        /* Start an initial 1s timeout for repeats */
        key_timer_3 = samples_per_second;
        #endif
        key_states |= KEY_3_DOWN;
        kick_foreground = true;
        break;
    case DEBOUNCE_HIT:
        if (key_timer_3  &&  --key_timer_3 == 0)
        {
        #if defined(KEY_3_LONG_DOWN)
            key_states |= KEY_3_LONG_DOWN;
        #elif defined(KEY_3_REPEAT_DOWN)
            /* Start a 1/3s timeout for repeats */
            key_timer_3 = 1092;
            key_states |= KEY_3_REPEAT_DOWN;
        #endif
            kick_foreground = true;
        }
        break;
    }
    #endif
    #if defined(sense_key_4_up)
    switch (debounce(&debounce_key_4, sense_key_4_up()))
    {
    case DEBOUNCE_JUST_RELEASED:
        key_timer_4 = 0;
        break;
    case DEBOUNCE_JUST_HIT:
        #if defined(KEY_4_LONG_DOWN)
        /* Start a 2s timer to detect mode change request */
        key_timer_4 = samples_per_second << 1;
        #elif defined(KEY_4_REPEAT_DOWN)
        /* Start an initial 1s timeout for repeats */
        key_timer_4 = samples_per_second;
        #endif
        key_states |= KEY_4_DOWN;
        kick_foreground = true;
        break;
    case DEBOUNCE_HIT:
        if (key_timer_4  &&  --key_timer_4 == 0)
        {
        #if defined(KEY_4_LONG_DOWN)
            key_states |= KEY_3_LONG_DOWN;
        #elif defined(KEY_4_REPEAT_DOWN)
            /* Start a 1/3s timeout for repeats */
            key_timer_4 = 1092;
            key_states |= KEY_4_REPEAT_DOWN;
        #endif
            kick_foreground = true;
        }
        break;
    }
    #endif
    return kick_foreground;
}
#endif 
