/*******************************************************************************
 *  emeter-rtc-c.c - RTC-C and RTC-CE support
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
#if defined(__MSP430__)
#include <msp430.h>
#endif

#include "emeter-template.h"

#include <emeter-toolkit.h>
#include <emeter-metrology.h>

#include "emeter-app.h"
#include "emeter-rtc.h"

#if !defined(NULL)
#define NULL    (void *) 0
#endif

/*
    Notes on RTC-C
    --------------
    The RTC correction works over periods of 1 minute, and works with cycles of the
    16384Hz clock derived by dividing the 32kHz crystal oscillator output by 2. There
    are 16384*60 => 983040 cycles of the 16384Hz clock in a minute, when there is no
    correction. This number is close enough to 1 million, so injecting one extra cycle
    of the 16384Hz once per minute adds close to 1ppm to the clock speed.

    Every 4096 cycles of the 16384Hz clock (1/4 of a second) the correction hardware
    decides whether to inject an extra cycle, drop a cycle, or leave the cycle
    rate the same as the crystal. By injecting a cycle in the first 0 to 240 1/4s parts
    of a minute, the corrector speeds up the clock by between 0 and 240ppm. Similarly,
    by dropping a cycle in the first 0 to 240 1/4s parts of a minute, the corrector
    slows down the clock by between 0 and 240ppm.

    Because the correction process works over a minute, it is necessary to feed correction
    updates to the correction hardware only once per minute. The correction cycles are
    synchronised to the minute updates in the RTC counters. A correction update can be
    written at any time during the minute, apart from a window of one 16384Hz clock cycle
    which occurs immediately before the minute changes. The correction will start to execute
    at the beginning of the minute following the write.
*/

static const int8_t month_lengths[13] =
{
    00,
    31, 28, 31,
    30, 31, 30,
    31, 31, 30,
    31, 30, 31
};

static const int16_t crystal_base_error = -118;

#if defined(CORRECTED_RTC_SUPPORT)
int16_t rtc_correction = 0;
#endif

#define MAX_TEMPERATURE_CORRECTION              240

/* This value is the temperature, in 1/16 degrees C, of the parabolic error curve for the crystal. */
#define CRYSTAL_QUADRATIC_CENTRE_TEMPERATURE    250

/* This value must be 167772.16 times the parts per million per degree Celcius squared error specified
   for the crystal. The following is for a -0.034ppm per C^2 crystal. */
#define CRYSTAL_QUADRATIC_COEFF                 ((int16_t) ((-34L*65536L*256L)/100000L)) //-5704

uint8_t rtc_status;

#if  defined(POWER_DOWN_SUPPORT)  &&  defined(POWER_UP_BY_VOLTAGE_PULSES)
uint8_t pd_pin_debounce;
#endif

/* We need a small seconds counter, so we can do things like a display update every 2 seconds. */
uint8_t seconds;

uint8_t rtc_offset_correction = 0;
uint8_t rtc_offset_direction = 0;
uint8_t rtc_offset_correction_step = 5;

/* Find the current day of the week, based on the values in the global
   RTC structure, as a number from 0 (Sunday) to 6 (Saturday). */
static int weekdayx(const uint8_t buf[])
{
    int i;
    int days;

    /* This works for years 2000-2099 */
    /* Result is 0=Sunday, 1=Monday, etc. */
    /* Allow for the day of the month */
    days = buf[2] + 6;
    /* Allow for the months to date this year... */
    for (i = 1;  i < buf[1];  i++)
        days += month_lengths[i];
    /* ...with a little offset if we are early in a leap year */
    if ((buf[0] & 0x03) == 0  &&  buf[1] <= 2)
        days--;
    /* Allow for the years... */
    days += buf[0];
    /* ...and a little extra for the leap years */
    days += (buf[0] >> 2);
    days %= 7;
    return days;
}

void get_rtc(uint8_t buf[6])
{
    buf[5] = RTCSEC;
    buf[4] = RTCMIN;
    buf[3] = RTCHOUR;
    buf[2] = RTCDAY;
    buf[1] = RTCMON;
    buf[0] = RTCYEAR - 2000;
}

void set_rtc(const uint8_t buf[6])
{
    /* Unlock RTC and RTC ready interrupt enable */
    RTCCTL0 = RTCKEY | RTCRDYIE | RTCTEVIE;
    RTCSEC = buf[5];
    RTCMIN = buf[4];
    RTCHOUR = buf[3];
    RTCDOW = weekdayx(buf);
    RTCDAY = buf[2];
    RTCMON = buf[1];
    RTCYEAR = buf[0] + 2000;
    /* Lock RTC */
    RTCCTL0 &= ~RTCKEY;
}

void rtc_init(void)
{
    /* Unlock RTC and RTC ready interrupt enable */
    RTCCTL0 = RTCKEY | RTCRDYIE | RTCTEVIE;
    RTCSEC = RTC_DEFAULT_SECOND;
    RTCMIN = RTC_DEFAULT_MINUTE;
    RTCHOUR = RTC_DEFAULT_HOUR;
    RTCDOW = RTC_DEFAULT_DOW;
    RTCDAY = RTC_DEFAULT_DAY;
    RTCMON = RTC_DEFAULT_MONTH;
    RTCYEAR = 2000 + RTC_DEFAULT_YEAR;
    /* 1min event interrupt, RTC sends 1Hz pulse to the RTCCLK pin */
    RTCCTL13 |= RTCTEV_0 | RTCCALF_3;
    /* Set binary mode and start the RTC */
    RTCCTL13 &= ~(RTCBCD | RTCHOLD);
    /* Lock RTC */
    RTCCTL0 = RTCRDYIE | RTCTEVIE;

    /* Unlock RTC and RTC ready interrupt enable */
    RTCCTL0 = RTCKEY | RTCRDYIE | RTCTEVIE;
    RTCOCAL = rtc_offset_correction + (rtc_offset_direction << 8);
    /* Lock RTC */
    RTCCTL0 &= ~RTCKEY;

    rtc_status = 0;
}

void correct_rtc(void)
{
}

void update_rtc_temperature_correction(void)
{
    int i;
    uint16_t correction;

    if (rtc_correction < 0)
        correction = 0x8000 | (-rtc_correction);
    else
        correction = rtc_correction;
    for (i = 0;  i < 500;  i++)
    {
        /* RTCTCRDY only becomes unready for 4 cycles of the 32kHz clock, just before
           each minute ticks over. This means we could stall here for 122us, which is
           3052 machine cycles, with a 24MHz core clock. However, if used sanely this
           routine will never be called during the relevant time window. */
        if ((RTCTCMP & RTCTCRDY))
            break;
    }
    RTCTCMP = correction;
    /* Now verify that we wrote the register successfully */
    //if (!(RTCTCMP & RTCTCOK))
    //    rtc_c_status |= RTC_C_STATUS_TCMP_FAILED;
}

void update_rtc_base_offset(int base_error)
{
    if (base_error < 0)
        base_error = 0x8000 | (-base_error);
    /* Unlock the RTC and RTC ready interrupt enable */
    RTCCTL0 = RTCKEY | RTCRDYIE | RTCTEVIE;
    RTCOCAL = base_error;
    /* Lock the RTC */
    RTCCTL0 &= ~RTCKEY;
}

void estimate_current_rtc_correction(void)
{
    int32_t temp;

    /* The RTC error consists of a constant error, due to the crystal's manufacturing
       tolerance, and a temperature dependant error, which varies in a parabolic
       manner. The centre of the parabola varies from crystal to crystal, but is generally
       around 25C. The coefficient of the parabola varies from crystal to crystal as
       well, but is generally around -0.035ppm per C^2. The crystal's data sheet should
       give guidance about these two parameters. We need to know them to determine the
       variable component of the crystal error, as the temperature varies. */

    /* We start with the temperature in 1/10th Celcius increments */
    temp = temperature();

    /* Now we need to calculate the ppm of clock error due to the current temperature. */

    /* Subtract the centre point of the crystal curve. */
    temp -= CRYSTAL_QUADRATIC_CENTRE_TEMPERATURE;

    /* Do the parabolic curve calculation, to find the current ppm of
       error due to temperature. */
    temp = temp*temp;
    /* Avoid overflow in the following multiply */
    temp >>= 4;
    /* Now apply the quadratic coefficient. The coefficient is 2^24 times the real
       value, and we just shifted by 4 bits, so we have 20 more to go. */
    temp = (temp*CRYSTAL_QUADRATIC_COEFF) >> 20;
    /* We should now have the temperature dependant error scaled to PPM, so we just
       need to add the basic manufacturing error of the crystal. */
    temp += crystal_base_error;

    /* Now we just need to clamp this PPM error value to the maximum + and - values
      supported by the RTC-C module. */
    if (temp > MAX_TEMPERATURE_CORRECTION)
        rtc_correction = MAX_TEMPERATURE_CORRECTION;
    else if (temp < -MAX_TEMPERATURE_CORRECTION)
        rtc_correction = -MAX_TEMPERATURE_CORRECTION;
    else
        rtc_correction = temp;
}

void per_second_stuff(void)
{
    #if defined(POWER_DOWN_SUPPORT)  &&  defined(POWER_UP_BY_SUPPLY_SENSING)
        #if defined(__MSP430_HAS_COMPA__)  ||  (defined(POWER_GOOD_SENSE)  &&  defined(POWER_GOOD_THRESHOLD_HIGH))
    int i;
    int j;
        #endif
    #endif

    #if defined(POWER_DOWN_SUPPORT)
        #if defined(POWER_UP_BY_VOLTAGE_PULSES)
    /* One method to detect power being restored is to look
       for pulses on an input pin, caused by the voltage signal. */
    if (metrology_status() & POWER_DOWN)
    {
        pd_pin_debounce <<= 1;
        if (power_up_voltage_pulse())
            pd_pin_debounce |= 1;
            #if defined(__MSP430__)
        //if ((pd_pin_debounce & 0xF) == 0xF)
        //    _BIC_SR_IRQ(LPM3_bits);
            #endif
    }
    else
    {
            #if defined(__MSP430__)
        //_BIC_SR_IRQ(LPM0_bits);
            #endif
    }
        #endif
        #if defined(POWER_UP_BY_SUPPLY_SENSING)
    /* If the meter has a limp mode, where it can be powered from the live
       or neutral only, getting wakeup voltage pulses is not so easy. Current
       pulses are not much easier. Here we look for the pre-regulator power
       supply voltage being of an adequate level. We use comparator A as the
       sensor, and only switch it on for the minimum possible time. */
            #if defined(__MSP430_HAS_COMPA__)
                #if defined(__MSP430__)
    if (operating_mode == OPERATING_MODE_POWERFAIL)
    {
        /* Select the lower comparator threshold for going to the LCD on, but other
           functions off, condition. Current consumption should be low enough to not
           be too significant for reasonable periods. */
        CACTL1 = CAREF_1 | CAON;
        /* We are required to start quickly, so we cannot do much
           debouncing here */
        power_down_debounce = POWER_RESTORE_DEBOUNCE;
        i = CACTL2 & CAOUT;
        while (--power_down_debounce >= 0)
        {
            j = CACTL2 & CAOUT;
            if (i != j)
            {
                i = j;
                power_down_debounce = POWER_RESTORE_DEBOUNCE;
            }
        }
        if (!j)
        {
            /* This appears to be a real power-up. We have reached 4.2V. This
               should be OK for running the internal LCD controller, as it only
               takes a few uA. For a small LCD, as little as 2uA. */
            operating_mode = OPERATING_MODE_LCD_ONLY;
                    #if defined(LCD_DISPLAY_SUPPORT)
            display_power_4v2_message();
                    #endif
            custom_lcd_wakeup_handler();
        }
        power_down_debounce = 0;
        CACTL1 &= ~(CAON);
    }
    else if (operating_mode == OPERATING_MODE_LCD_ONLY)
    {
        /* Select the higher comparator threshold for power restored. That should
           mean that if the MCU is woken up, it can definitely run for a while at
           full speed, just from the charge on the main capacitor. */
        CACTL1 = CAREF_2 | CAON;
        /* We are required to start quickly, so we cannot do much
           debouncing here. */
        power_down_debounce = POWER_RESTORE_DEBOUNCE;
        i = CACTL2 & CAOUT;
        while (--power_down_debounce >= 0)
        {
            j = CACTL2 & CAOUT;
            if (i != j)
            {
                i = j;
                power_down_debounce = POWER_RESTORE_DEBOUNCE;
            }
        }
        if (!j)
        {
            /* This appears to be a real power-up. */
                    #if defined(LCD_DISPLAY_SUPPORT)
            display_power_normal_message();
                    #endif
            //_BIC_SR_IRQ(LPM3_bits);
        }
        else
        {
            /* The power hasn't reached the high water mark. See if it has
               dropped back below the low water mark. */
            CACTL1 = CAREF_1 | CAON;
            /* We are required to start quickly, so we cannot do much
               debouncing here. */
            power_down_debounce = POWER_RESTORE_DEBOUNCE;
            i = CACTL2 & CAOUT;
            while (--power_down_debounce >= 0)
            {
                j = CACTL2 & CAOUT;
                if (i != j)
                {
                    i = j;
                    power_down_debounce = POWER_RESTORE_DEBOUNCE;
                }
            }
            if (j)
            {
                /* This appears to be a real power drop. */
                operating_mode = OPERATING_MODE_POWERFAIL;
                custom_lcd_sleep_handler();
            }
        }
        power_down_debounce = 0;
        CACTL1 &= ~(CAON);
    }
                #endif
            #else
    /* Use an I/O pin to sense the power falling */
                #if defined(__MSP430__)
    if (operating_mode == OPERATING_MODE_POWERFAIL)
    {
        /* Select the higher comparator threshold for starting up. This ensures we should have
           enough energy in the capacitors to keep the meter running until it works out what to
           do next. */
        POWER_GOOD_THRESHOLD_HIGH;
        /* We are required to start quickly, so we cannot do much
           debouncing here */
        power_down_debounce = POWER_RESTORE_DEBOUNCE;
        i = POWER_GOOD_SENSE;
        while (--power_down_debounce >= 0)
        {
            j = POWER_GOOD_SENSE;
            if (i != j)
            {
                i = j;
                power_down_debounce = POWER_RESTORE_DEBOUNCE;
            }
        }
        if (j)
        {
            /* This appears to be a real power-up. */
                    #if defined(LCD_DISPLAY_SUPPORT)
            display_power_normal_message();
                    #endif
            custom_lcd_wakeup_handler();
            //_BIC_SR_IRQ(LPM3_bits);
        }
        power_down_debounce = 0;
    }
                #endif
            #endif
        #endif
        #if defined(LIMP_MODE_SUPPORT)
    if (operating_mode == OPERATING_MODE_LIMP)
    {
        /* We need to kick things, to give the foreground activities a chance
           to do their work. */
        //_BIC_SR_IRQ(LPM0_bits);
    }
        #endif
    #endif
}

#if defined(__MSP430__)
ISR(RTC, rtc_isr)
{
    switch (__even_in_range(RTCIV, RTC_RT1PSIFG))
    {
    case RTC_NONE:
        break;
    case RTC_RTCOFIFG:
        break;
    case RTC_RTCRDYIFG:
        __no_operation();
        __no_operation();
        kick_watchdog();
        estimate_current_rtc_correction();
        if (RTCSEC == 59)
        {
            /* Set up the next minute's RTC correction */
            update_rtc_temperature_correction();
        }
        if (++seconds & 1)
            rtc_status |= RTC_STATUS_TICKER;  /* Kick the ain loop every 2 seconds */
        per_second_stuff();
        kick_watchdog();
        break;
    case RTC_RTCTEVIFG:
        break;
    case RTC_RTCAIFG:
        break;
    case RTC_RT0PSIFG:
        break;                         
    case RTC_RT1PSIFG:
        break;
    }
}
#endif
