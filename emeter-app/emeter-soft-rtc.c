/*******************************************************************************
 *  emeter-soft-rtc.c -
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

#if defined(RTC_SUPPORT)

#if !defined(NULL)
#define NULL    (void *) 0
#endif

/* We want the rtc to be uninitialized, so it can persist past a processor
   reset. The rtc structure contains a sumcheck, so we can do a consistency
   check on the rtc as we come out of reset. */
__uninitialized__ rtc_t rtc;
__uninitialized__ int32_t time_to_next_hop;

static const int8_t month_lengths[13] =
{
    00,
    31, 28, 31,
    30, 31, 30,
    31, 31, 30,
    31, 30, 31
};

#define MAX_TEMPERATURE_CORRECTION              240
#define CRYSTAL_QUADRATIC_CENTRE_TEMPERATURE    2500

#if defined(CORRECTED_RTC_SUPPORT)
int16_t rtc_correction = 0;
int32_t temp_correction;
int32_t rtc_integrated_correction;
#endif
uint8_t rtc_status;

#if  defined(POWER_DOWN_SUPPORT)  &&  defined(POWER_UP_BY_VOLTAGE_PULSES)
uint8_t pd_pin_debounce;
#endif

/* We need a small seconds counter, so we can do things like a display update every 2 seconds. */
uint8_t seconds;

static void set_rtc_sumcheck(void)
{
    rtc.sumcheck = ~(rtc.second + rtc.minute + rtc.hour + rtc.day + rtc.month + rtc.year);
}

int bump_rtc(void)
{
    /* First check the consistency of the current RTC setting. If it is inconsistent,
       (i.e. has a bad sumcheck) do not bump it. */
    if (!check_rtc_sumcheck())
        return RTC_INCONSISTENT;
    if (++rtc.second < 60)
    {
        set_rtc_sumcheck();
        return RTC_CHANGED_SECOND;
    }
    rtc.second = 0;
    if (++rtc.minute < 60)
    {
        set_rtc_sumcheck();
        return RTC_CHANGED_MINUTE;
    }
    rtc.minute = 0;
    if (++rtc.hour < 24)
    {
        set_rtc_sumcheck();
        return RTC_CHANGED_HOUR;
    }
    rtc.hour = 0;
    if ((rtc.month == 2  &&  (rtc.year & 3) == 0  &&  rtc.day < 29)
        ||
        rtc.day < month_lengths[rtc.month])
    {
        ++rtc.day;
        set_rtc_sumcheck();
        return RTC_CHANGED_DAY;
    }
    rtc.day = 1;
    if (++rtc.month <= 12)
    {
        set_rtc_sumcheck();
        return RTC_CHANGED_MONTH;
    }
    rtc.month = 1;
    ++rtc.year;
    set_rtc_sumcheck();
    return RTC_CHANGED_YEAR;
}

int check_rtc_sumcheck(void)
{
    return rtc.sumcheck == ((~(rtc.second + rtc.minute + rtc.hour + rtc.day + rtc.month + rtc.year)) & 0xFF);
}

/* Find the current day of the week, based on the values in the global
   RTC structure, as a number from 0 (Sunday) to 6 (Saturday). */
int weekday(rtc_t *s)
{
    int i;
    int days;

    /* This works for years 2000-2099 */
    /* Result is 0=Sunday, 1=Monday, etc. */
    /* Allow for the day of the month */
    days = s->day + 6;
    /* Allow for the months to date this year... */
    for (i = 1;  i < s->month;  i++)
        days += month_lengths[i];
    /* ...with a little offset if we are early in a leap year */
    if ((s->year & 0x03) == 0  &&  s->month <= 2)
        days--;
    /* Allow for the years... */
    days += s->year;
    /* ...and a little extra for the leap years */
    days += (s->year >> 2);
    days %= 7;
    return days;
}

#if defined(RTC_SUPPORT)
void get_rtc(uint8_t buf[6])
{
    buf[5] = rtc.second;
    buf[4] = rtc.minute;
    buf[3] = rtc.hour;
    buf[2] = rtc.day;
    buf[1] = rtc.month;
    buf[0] = rtc.year;
}

void set_rtc(const uint8_t buf[6])
{
    rtc.year = buf[0];
    rtc.month = buf[1];
    rtc.day = buf[2];
    rtc.hour = buf[3];
    rtc.minute = buf[4];
    rtc.second = buf[5];
    set_rtc_sumcheck();
}

void rtc_bumper(void)
{
#if defined(PER_YEAR_ACTIVITY_SUPPORT)  ||  defined(PER_MONTH_ACTIVITY_SUPPORT)  ||  defined(PER_DAY_ACTIVITY_SUPPORT)  ||  defined(PER_HOUR_ACTIVITY_SUPPORT)  ||  defined(PER_MINUTE_ACTIVITY_SUPPORT)  ||  defined(PER_SECOND_ACTIVITY_SUPPORT)
    int i;

    i = bump_rtc();
#else
    bump_rtc();
#endif

    /* And now, a series of optional routines to get actions to take
       place at various intervals. Remember, we are in an interrupt
       routine. Do not do anything very complex here. If a complex action
       is needed set a flag in a simple routine and do the main work in
       the non-interrupt code's main loop. */
    #if defined(PER_YEAR_ACTIVITY_SUPPORT)
    if (i >= RTC_CHANGED_YEAR)
        per_year_activity();    
    #endif
    #if defined(PER_MONTH_ACTIVITY_SUPPORT)
    if (i >= RTC_CHANGED_MONTH)
        per_month_activity();
    #endif
    #if defined(PER_DAY_ACTIVITY_SUPPORT)  ||  defined(MULTI_RATE_SUPPORT)
    if (i >= RTC_CHANGED_DAY)
    {
        #if defined(PER_MINUTE_ACTIVITY_SUPPORT)
        per_day_activity();
        #endif
        #if defined(MULTI_RATE_SUPPORT)
        tariff_flags |= TARIFF_NEW_DAY;
        #endif
    }
    #endif
    #if defined(PER_HOUR_ACTIVITY_SUPPORT)
    if (i >= RTC_CHANGED_HOUR)
        per_hour_activity();
    #endif
    #if defined(PER_MINUTE_ACTIVITY_SUPPORT)  ||  defined(MULTI_RATE_SUPPORT)  ||  defined(BATTERY_MONITOR_SUPPORT)
    if (i >= RTC_CHANGED_MINUTE)
    {
        #if defined(PER_MINUTE_ACTIVITY_SUPPORT)
        per_minute_activity();
        #endif
        #if defined(MULTI_RATE_SUPPORT)
        tariff_flags |= TARIFF_NEW_MINUTE;
        #endif
        #if defined(BATTERY_MONITOR_SUPPORT)
        test_battery();
        #endif
    }
    #endif
    #if defined(PER_SECOND_ACTIVITY_SUPPORT)
    if (i >= RTC_CHANGED_SECOND)
        per_second_activity();
    #endif
}

#if defined(CORRECTED_RTC_SUPPORT)
void correct_rtc(void)
{
    int32_t temp;

    /* Correct the RTC to allow for basic error in the crystal, and
       temperature dependant changes. This is called every two seconds,
       so it must accumulate two seconds worth of error at the current
       temperature. */
    /* The temperature routine returns 0x8000 if a real temperature measurement
       is not available */
    if ((temp = temperature()) != 0x8000)
    {
        /* The temperature is now in degrees C. */
        /* Subtract the centre point of the crystal curve. */
        temp -= CRYSTAL_QUADRATIC_CENTRE_TEMPERATURE;
        /* Do the parabolic curve calculation, to find the current PPM of
           error due to temperature, and then the scaled RTC correction
           value for 2 seconds at this temperature. */
        temp = temp*temp*(2589L*4295L >> 5);
        temp >>= 11;
        temp = -temp;
        /* Clamp to the maximum correction allowed by the RTC module */
        temp_correction = (temp > MAX_TEMPERATURE_CORRECTION)  ?  MAX_TEMPERATURE_CORRECTION  :  temp;
    }
    else
    {
        temp = 0;
    }
    #if defined(CORRECTED_RTC_SUPPORTx)
    /* Allow for the basic manufacturing tolerance error of the crystal, found
       at calibration time. */
    temp += nv_parms.seg_a.s.cal_data_crystal_base_correction;
    #endif
    if (rtc_integrated_correction >= 0)
    {
        rtc_integrated_correction += temp;
        if (rtc_integrated_correction < 0)
        {
            rtc_integrated_correction -= 0x80000000;
            /* We need to add an extra second to the RTC */
            rtc_bumper();
        }
    }
    else
    {
        rtc_integrated_correction += temp;
        if (rtc_integrated_correction >= 0)
        {
            rtc_integrated_correction += 0x80000000;
            /* We need to drop a second from the RTC */
            rtc_status |= RTC_STATUS_SKIP_A_SECOND;
        }
    }
}
#endif

#if defined(CORRECTED_RTC_SUPPORT)  &&  defined(__MSP430_HAS_TA3__)
int32_t assess_rtc_speed(void)
{
    int32_t period;
    uint16_t this_capture;
    uint16_t last_capture;
    uint16_t step;
    int32_t counter;
    int limit;

    /* The fast clock should be an exact multiple of the crystal clock, once the FLL has
        settled. If we capture many cycles of an accurate external 32768Hz clock, using
        timer A (or B), we can measure the speed difference between the MSP430's crystal
        and the external clock in a reasonable time. */
    /* The SM clock should be running at 244*32768Hz at this time. */
    __disable_interrupt();
    /* Change timer A to running fast, and sample the external 32768Hz reference. */
    TACCR0 = 0xFFFF;
    TACCTL0 = CAP | CCIS_0 | CM_1;
    TACCTL2 = CAP | CCIS_0 | CM_1 | SCS;
    TACTL = TACLR | MC_2 | TASSEL_2;    /* start TIMER_A up mode, SMCLK as input clock */
    period = 0;
    last_capture = TACCR2;
    limit = -1;
    TACCTL2 &= ~CCIFG;
    for (counter = 0;  counter < 32768*5 + 1;  counter++)
    {
        limit = 1000;
        while (!(TACCTL2 & CCIFG))
        {
            if (--limit <= 0)
                break;
        }
        if (limit <= 0)
            break;
        TACCTL2 &= ~CCIFG;
        this_capture = TACCR2;
        step = this_capture - last_capture;
        last_capture = this_capture;
        /* Skip the first sample, as it will be meaningless */
        if (counter)
        {
    #if 0
            if (step < (244 - 5)  ||  step > (244 + 5))
            {
                limit = -2;
                break;
            }
    #endif
            period += step;
        }
        kick_watchdog();
    }
    if (limit <= 0)
        period = limit;
    TACTL = TACLR | MC_1 | TASSEL_1;
    TACCTL0 = CCIE;
    __enable_interrupt();
    return  period;
}
#endif

void rtc_init(void)
{
    if (!check_rtc_sumcheck())
    {
        rtc.year = RTC_DEFAULT_YEAR;
        rtc.month = RTC_DEFAULT_MONTH;
        rtc.day = RTC_DEFAULT_DAY;
        rtc.hour = RTC_DEFAULT_HOUR;
        rtc.minute = RTC_DEFAULT_MINUTE;
        rtc.second = RTC_DEFAULT_SECOND;
        set_rtc_sumcheck();
    }
    rtc_status = 0;
}
#endif

#if defined(__MSP430__)
    #if defined(__MSP430_HAS_BT__)  ||  defined(__MSP430_HAS_BT_RTC__)
ISR(BASICTIMER, one_second_ticker)
    #else
ISR(WDT, one_second_ticker)
    #endif
{
    #if defined(POWER_DOWN_SUPPORT)  &&  defined(POWER_UP_BY_SUPPLY_SENSING)
        #if defined(__MSP430_HAS_COMPA__)  ||  (defined(POWER_GOOD_SENSE)  &&  defined(POWER_GOOD_THRESHOLD_HIGH))
    int i;
    int j;
        #endif
    #endif

    kick_watchdog();
    #if defined(CORRECTED_RTC_SUPPORT)
    /* Allow for RTC correction. */
    if ((rtc_status & RTC_STATUS_SKIP_A_SECOND))
        rtc_status &= ~RTC_STATUS_SKIP_A_SECOND;
    else
        rtc_bumper();
    #else
    rtc_bumper();
    #endif
    if (++seconds & 1)
        rtc_status |= RTC_STATUS_TICKER;  /* Kick every 2 seconds */

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
    kick_watchdog();
}
#endif
#endif
