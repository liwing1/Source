/*******************************************************************************
 *  emeter-basic-display.c -
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

#include <inttypes.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#if defined(__MSP430__)
#include <msp430.h>
#endif

#include "stdarg.h"

#include "emeter-template.h"

#include <emeter-toolkit.h>
#include <emeter-metrology.h>

#include "emeter-main.h"
#include "emeter-app.h"
#include "emeter-lcd.h"
#include "emeter-rtc.h"
#include "emeter-basic-display.h"
#if defined(MULTI_RATE_SUPPORT)
#include "emeter-multirate.h"
#endif
#include "emeter-oled.h"

#if defined(LCD_DISPLAY_SUPPORT) 

int key_states;

#if defined(MULTI_RATE_SUPPORT)
uint8_t info_step;
uint8_t info_substep;
#endif

static const char lcd_high[] = "High";

enum
{
    DISPLAY_ITEM_SELECT_RESTART = -6,
    DISPLAY_ITEM_SELECT_NEUTRAL = -1 - FAKE_PHASE_NEUTRAL,
    DISPLAY_ITEM_SELECT_TOTAL = -1 - FAKE_PHASE_TOTAL,
    DISPLAY_ITEM_SELECT_PHASE_3 = -1 - PHASE_BLUE,
    DISPLAY_ITEM_SELECT_PHASE_2 = -1 - PHASE_YELLOW,
    DISPLAY_ITEM_SELECT_PHASE_1 = -1 - PHASE_RED,
    DISPLAY_ITEM_ACTIVE_POWER,
#if defined(ACTIVE_ENERGY_SUPPORT)  ||  defined(TOTAL_ACTIVE_ENERGY_SUPPORT)
    DISPLAY_ITEM_ACTIVE_ENERGY,
#endif
#if defined(REACTIVE_POWER_SUPPORT)
    DISPLAY_ITEM_REACTIVE_POWER,
#endif
#if defined(REACTIVE_ENERGY_SUPPORT)  ||  defined(TOTAL_REACTIVE_ENERGY_SUPPORT)
    DISPLAY_ITEM_REACTIVE_ENERGY,
#endif
#if defined(APPARENT_POWER_SUPPORT)
    DISPLAY_ITEM_APPARENT_POWER,
#endif
#if defined(APPARENT_ENERGY_SUPPORT)  ||  defined(TOTAL_APPARENT_ENERGY_SUPPORT)
    DISPLAY_ITEM_APPARENT_ENERGY,
#endif
#if defined(MAINS_FREQUENCY_SUPPORT)
    DISPLAY_ITEM_MAINS_FREQUENCY,
#endif
#if defined(VRMS_SUPPORT)
    DISPLAY_ITEM_VOLTAGE,
#endif
#if defined(IRMS_SUPPORT)
    DISPLAY_ITEM_CURRENT,
#endif
#if defined(POWER_FACTOR_SUPPORT)
    DISPLAY_ITEM_POWER_FACTOR,
#endif
#if defined(RTC_SUPPORT)
    DISPLAY_ITEM_DATE,
    DISPLAY_ITEM_TIME,
#endif
#if defined(TEMPERATURE_SUPPORT)
    DISPLAY_ITEM_TEMPERATURE,
#endif
#if defined(MULTI_RATE_SUPPORT)
    DISPLAY_ITEM_CURRENT_TARIFF,
#endif
    DISPLAY_ITEM_DUMMY
};

 

void display_clear_periphery(void)
{
    /* Clear all the symbols around the display, which we are not using */
    custom_lcd_clear_periphery();
 
}
 
void display_startup_message(void)
{
    display_clear_periphery();
    lcd_string("Start", 1);
}


void display_power_4v2_message(void)
{
    lcd_string("4V2", 1);
}

void display_power_normal_message(void)
{
    lcd_string("8V", 1);
}

void display_clear_lines(void)
{
    /* Clear the digits */
    lcd_string(" ", 1);
    /* Clear the associated tags */
    custom_lcd_clear_line_1_tags();
#if NUM_DISPLAY_FIELDS > 1
    /* Clear the digits */
    lcd_string(" ", 2);
    /* Clear the associated tags */
    custom_lcd_clear_line_2_tags();
#endif
    display_clear_periphery();
}

#if NUM_PHASES > 1  &&  defined(ICON_PHASE_A)
void display_phase_icon(int ph)
{
    int i;
    static const uint8_t phase_icons[NUM_PHASES] =
    {
        ICON_PHASE_A,
    #if NUM_PHASES >= 2
        ICON_PHASE_B,
    #endif
    #if NUM_PHASES >= 3
        ICON_PHASE_C
    #endif
    };

    for (i = 0;  i < NUM_PHASES;  i++)
        lcd_icon(phase_icons[i], i == ph);
}
#else
#define display_phase_icon(x) /**/
#endif



#if defined(MAINS_FREQUENCY_SUPPORT)

static __inline__ void display_mains_frequency(int ph)
{
    frequency_t x;

    /* Display mains frequency in 0.1Hz or 0.01Hz increments */
    x = mains_frequency(ph);

    lcd_decu32(x, FREQUENCY_DISPLAY_FIELD, -FREQUENCY_FRACTIONAL_DIGITS);
    #if defined(ICON_HERTZ)
    lcd_icon(ICON_HERTZ, true);
    #elif defined(FREQUENCY_TAG)
    lcd_string_only(FREQUENCY_TAG, DISPLAY_TYPE_FIELD, DISPLAY_TYPE_POSITION);
    #endif
}
#endif

#if defined(VRMS_SUPPORT)

static __inline__ void display_rms_voltage(int ph)
{
    rms_voltage_t x;

    /* Display RMS voltage in 0.1V or 0.01V increments */
    x = rms_voltage(ph);
    

    
    if (x == RMS_VOLTAGE_OVERRANGE)
    {
        lcd_string(lcd_high, VOLTAGE_DISPLAY_FIELD);
    }
    else
    {
        lcd_dec32(x, VOLTAGE_DISPLAY_FIELD, -RMS_VOLTAGE_FRACTIONAL_DIGITS);
    #if defined(ICON_V)
        lcd_icon(ICON_V, true);
    #endif
    }
    #if defined(ICON_VOLTAGE)
    lcd_icon(ICON_VOLTAGE, true);
    #elif defined(VOLTAGE_TAG)
    lcd_string_only(VOLTAGE_TAG, DISPLAY_TYPE_FIELD, DISPLAY_TYPE_POSITION);
    #endif
  
}
#endif

#if defined(IRMS_SUPPORT)

static __inline__ void display_rms_current(int ph)
{
    rms_current_t x;

    /* Display RMS current in 1mA increments */
    x = rms_current(ph);
    
    if (x == RMS_CURRENT_OVERRANGE)
    {
        lcd_string(lcd_high, CURRENT_DISPLAY_FIELD);
    }
    else
    {
        lcd_dec32(x, CURRENT_DISPLAY_FIELD, -RMS_CURRENT_FRACTIONAL_DIGITS);
    #if defined(ICON_A)
        lcd_icon(ICON_A, true);
    #endif
    }
    #if defined(ICON_CURRENT)
    lcd_icon(ICON_CURRENT, true);
    #elif defined(CURRENT_TAG)
    lcd_string_only(CURRENT_TAG, DISPLAY_TYPE_FIELD, DISPLAY_TYPE_POSITION);
    #endif
}
#endif


static __inline__ void display_active_power(int ph)
{
    power_t x;

    /* Display per phase or total active power */
    x = active_power(ph);
    
    if (x == POWER_OVERRANGE)
    {
        lcd_string(lcd_high, ACTIVE_POWER_DISPLAY_FIELD);
    }
    else
    {
#if defined(ACTIVE_POWER_DISPLAY_IN_KW)
        lcd_dec32(x, ACTIVE_POWER_DISPLAY_FIELD, -(POWER_FRACTIONAL_DIGITS + 3));
#else
        lcd_dec32(x, ACTIVE_POWER_DISPLAY_FIELD, -POWER_FRACTIONAL_DIGITS);
#endif
#if defined(ICON_kW)
        lcd_icon(ICON_kW, true);
#endif
    }
#if defined(ICON_ACTIVE_POWER)
    lcd_icon(ICON_ACTIVE_POWER, true);
    #elif defined(ACTIVE_POWER_TAG)
    lcd_string_only(ACTIVE_POWER_TAG, DISPLAY_TYPE_FIELD, DISPLAY_TYPE_POSITION);
#endif
   
}

#if defined(REACTIVE_POWER_SUPPORT)  ||  defined(TOTAL_REACTIVE_POWER_SUPPORT)

static __inline__ void display_reactive_power(int ph)
{
    power_t x;

    /* Display reactive power in 0.01W increments */
    x = reactive_power(ph);
    
    if (x == POWER_OVERRANGE)
    {
        lcd_string(lcd_high, REACTIVE_POWER_DISPLAY_FIELD);
    }
    else
    {
    #if defined(REACTIVE_POWER_DISPLAY_IN_KVAR)
         lcd_dec32(x, REACTIVE_POWER_DISPLAY_FIELD, -(POWER_FRACTIONAL_DIGITS + 3));
    #else
         lcd_dec32(x, REACTIVE_POWER_DISPLAY_FIELD, -POWER_FRACTIONAL_DIGITS);
    #endif
    #if defined(ICON_kvar)
        lcd_icon(ICON_kvar, true);
    #endif
    }
    #if defined(ICON_REACTIVE_POWER)
    lcd_icon(ICON_REACTIVE_POWER, true);
    #elif defined(REACTIVE_POWER_TAG)
    lcd_string_only(REACTIVE_POWER_TAG, DISPLAY_TYPE_FIELD, DISPLAY_TYPE_POSITION);
    #endif
}
#endif

#if defined(APPARENT_POWER_SUPPORT)  ||  defined(TOTAL_APPARENT_POWER_SUPPORT)

static __inline__ void display_apparent_power(int ph)
{
    power_t x;

    /* Display apparent (VA) power in 0.01W increments */
    x = apparent_power(ph);
    
    if (x == POWER_OVERRANGE)
    {
        lcd_string(lcd_high, APPARENT_POWER_DISPLAY_FIELD);
    }
    else
    {
    #if defined(APPARENT_POWER_DISPLAY_IN_KVA)
        lcd_dec32(x, APPARENT_POWER_DISPLAY_FIELD, -(POWER_FRACTIONAL_DIGITS + 3));
    #else
        lcd_dec32(x, APPARENT_POWER_DISPLAY_FIELD, -POWER_FRACTIONAL_DIGITS);
    #endif
    #if defined(ICON_kVA)
        lcd_icon(ICON_kVA, true);
    #endif
    }
    #if defined(ICON_APPARENT_POWER)
    lcd_icon(ICON_APPARENT_POWER, true);
    #elif defined(APPARENT_POWER_TAG)
    lcd_string_only(APPARENT_POWER_TAG, DISPLAY_TYPE_FIELD, DISPLAY_TYPE_POSITION);
    #endif
}
#endif

#if defined(IRMS_SUPPORT)  &&  defined(VRMS_SUPPORT)  &&  defined(POWER_FACTOR_SUPPORT)

static __inline__ void display_power_factor(int ph)
{
    power_factor_t x;

    x = power_factor(ph);
 
    if (x < 0)
    {
        lcd_char('L', 1, 1);
        x = -x;
    }
    else
    {
        lcd_char('C', 1, 1);
    }
    lcd_dec16(x, POWER_FACTOR_DISPLAY_FIELD, POWER_FACTOR_FRACTIONAL_DIGITS);
    #if defined(ICON_POWER_FACTOR_DECIMAL_POINT)
    lcd_icon(ICON_POWER_FACTOR_DECIMAL_POINT, true);
    #endif
    #if defined(ICON_COS_PHI)
    lcd_icon(ICON_COS_PHI, true);
    #elif defined(POWER_FACTOR_TAG)
    lcd_string_only(POWER_FACTOR_TAG, DISPLAY_TYPE_FIELD, DISPLAY_TYPE_POSITION);
    #endif
}
#endif

#if defined(ACTIVE_ENERGY_SUPPORT)  ||  defined(TOTAL_ACTIVE_ENERGY_SUPPORT)


static __inline__ void display_imported_active_energy(int ph)
{
    energy_t x;

    /* Display per phase or total imported active energy */
    x = energy_consumed[ph][APP_ACTIVE_ENERGY_IMPORTED];
    
    #if defined(ACTIVE_ENERGY_DISPLAY_IN_KWH)
    lcd_decu64(x, ACTIVE_ENERGY_DISPLAY_FIELD, -(ENERGY_FRACTIONAL_DIGITS + 3));
    #else
    lcd_decu64(x, ACTIVE_ENERGY_DISPLAY_FIELD, -ENERGY_FRACTIONAL_DIGITS);
    #endif
    #if defined(ICON_kWH)
    lcd_icon(ICON_kWH, true);
    #elif defined(ICON_kW)  &&  defined(ICON_H_FOR_kW)
    lcd_icon(ICON_kW, true);
    lcd_icon(ICON_H_FOR_kW, true);
    #endif
    #if defined(ICON_ACTIVE_ENERGY)
    lcd_icon(ICON_ACTIVE_ENERGY, true);
    #elif defined(ACTIVE_ENERGY_TAG)
    lcd_string_only(ACTIVE_ENERGY_TAG, DISPLAY_TYPE_FIELD, DISPLAY_TYPE_POSITION);
    #endif
}
#endif

#if defined(REACTIVE_ENERGY_SUPPORT)  ||  defined(TOTAL_REACTIVE_ENERGY_SUPPORT)

static __inline__ void display_imported_reactive_energy(int ph)
{
    energy_t x;

    /* Display per phase or total imported reactive energy */
    x = energy_consumed[ph][APP_REACTIVE_ENERGY_QUADRANT_I] + energy_consumed[ph][APP_REACTIVE_ENERGY_QUADRANT_IV];

    #if defined(REACTIVE_ENERGY_DISPLAY_IN_KVARH)
    lcd_decu64(x, REACTIVE_ENERGY_DISPLAY_FIELD, -(ENERGY_FRACTIONAL_DIGITS + 3));
    #else
    lcd_decu64(x, REACTIVE_ENERGY_DISPLAY_FIELD, -ENERGY_FRACTIONAL_DIGITS);
    #endif
    #if defined(ICON_kvarH)
    lcd_icon(ICON_kvarH, true);
    #elif defined(ICON_kvar)  &&  defined(ICON_H_FOR_kvar)
    lcd_icon(ICON_kvar, true);
    lcd_icon(ICON_H_FOR_kvar, true);
    #endif
    #if defined(ICON_REACTIVE_ENERGY)
    lcd_icon(ICON_REACTIVE_ENERGY, true);
    #elif defined(REACTIVE_ENERGY_TAG)
    lcd_string_only(REACTIVE_ENERGY_TAG, DISPLAY_TYPE_FIELD, DISPLAY_TYPE_POSITION);
    #endif
}
#endif

#if defined(RTC_SUPPORT)
static __inline__ void display_date(int year, int month, int day)
{
  
    lcd_decu16_sub_field(year, DATE_DISPLAY_FIELD, 2, YEAR_DISPLAY_POSITION, 2);
    lcd_decu16_sub_field(month, DATE_DISPLAY_FIELD, 2, MONTH_DISPLAY_POSITION, 2);
    lcd_decu16_sub_field(day, DATE_DISPLAY_FIELD, 2, DAY_DISPLAY_POSITION, 2);
        #if defined(ICON_DATE_COLON_1)
    lcd_icon(ICON_DATE_COLON_1, true);
        #endif
        #if defined(ICON_DATE_COLON_1A)
    lcd_icon(ICON_DATE_COLON_1A, true);
        #endif
        #if defined(ICON_DATE_COLON_2)
    lcd_icon(ICON_DATE_COLON_2, true);
        #endif
        #if defined(ICON_DATE_COLON_2A)
    lcd_icon(ICON_DATE_COLON_2A, true);
        #endif

}

static __inline__ void display_time(int hour, int minute, int second)
{

    lcd_decu16_sub_field(hour, TIME_DISPLAY_FIELD, 2, HOUR_DISPLAY_POSITION, 2);
    lcd_decu16_sub_field(minute, TIME_DISPLAY_FIELD, 2, MINUTE_DISPLAY_POSITION, 2);
        #if defined(ICON_TIME_COLON_1)
    lcd_icon(ICON_TIME_COLON_1, true);
        #endif
        #if defined(ICON_TIME_COLON_1A)
    lcd_icon(ICON_TIME_COLON_1A, true);
        #endif
        #if defined(SECONDS_DISPLAY_POSITION)
    lcd_decu16_sub_field(second, TIME_DISPLAY_FIELD, 2, SECONDS_DISPLAY_POSITION, 2);
            #if defined(ICON_TIME_COLON_2)
    lcd_icon(ICON_TIME_COLON_2, true);
            #endif
            #if defined(ICON_TIME_COLON_2A)
    lcd_icon(ICON_TIME_COLON_2A, true);
            #endif
        #endif

}

static /*__inline__*/ void display_current_date(void)
{
    uint8_t date[6];

        #if defined(ZAP_COLON_CELL)
    lcd_char(' ', DATE_DISPLAY_FIELD, 1);
        #endif
    get_rtc(date);
    display_date(date[0], date[1], date[2]);
        #if defined(ICON_DATE)
    lcd_icon(ICON_DATE, true);
        #elif defined(DATE_TAG)
    lcd_string_only(DATE_TAG, DISPLAY_TYPE_FIELD, DISPLAY_TYPE_POSITION);
        #endif
}

static __inline__ void display_current_time(void)
{
    uint8_t date[6];

    get_rtc(date);
        #if defined(ZAP_COLON_CELL)
    lcd_char(' ', TIME_DISPLAY_FIELD, 1);
        #endif
    display_time(date[3], date[4], date[5]);
        #if defined(ICON_TIME)
    lcd_icon(ICON_TIME, true);
        #elif defined(TIME_TAG)
    lcd_string_only(TIME_TAG, DISPLAY_TYPE_FIELD, DISPLAY_TYPE_POSITION);
        #endif
}
#endif

#if defined(TEMPERATURE_SUPPORT)
static __inline__ void display_temperature(void)
{
  
    int x = temperature(); 
    
    lcd_dec16(x, TEMPERATURE_DISPLAY_FIELD, TEMPERATURE_DISPLAY_RESOLUTION);
        #if !defined(ICON_TEMPERATURE)  &&  defined(DISPLAY_TYPE_POSITION)
    lcd_char('C', TEMPERATURE_DISPLAY_FIELD, 1);
        #endif
  
}
#endif

#if defined(MULTI_RATE_SUPPORT)
static __inline__ void display_current_tariff(void)
{
    lcd_decu16(current_tariff + 1, TARRIF_DISPLAY_FIELD, 0);
    #if !defined(ICON_DATE)  &&  defined(DISPLAY_TYPE_POSITION)
    lcd_string_only('Tarr ', TARRIF_DISPLAY_FIELD, DISPLAY_TYPE_POSITION);
    #endif
}

void display_tariff_holiday(void)
{
    int i;
    eeprom_holiday_t holiday;

    info_step = 0;
    for (i = info_step;  i < MULTIRATE_MAX_HOLIDAYS;  i++)
    {
        iicEEPROM_read(EEPROM_START_HOLIDAYS + i*sizeof(eeprom_holiday_t), (void *) &holiday, sizeof(holiday));
        if (holiday.year)
        {
            info_step = i;
    #if defined(ZAP_COLON_CELL)
            lcd_char(' ', TARRIF_DISPLAY_FIELD, 1);
    #endif
            display_date(holiday.year, holiday.month, holiday.day);
    #if defined(ICON_DATE)
            lcd_icon(ICON_DATE, true);
    #elif !defined(ICON_DATE)  &&  defined(DISPLAY_TYPE_POSITION)
            lcd_char('D', TARRIF_DISPLAY_FIELD, DISPLAY_TYPE_POSITION);
        #if FIRST_POSITION > 2
            lcd_char('t', TARRIF_DISPLAY_FIELD, DISPLAY_TYPE_POSITION + 1);
        #endif
    #endif
            return;
        }
    }
}
#endif



void display_item(int item, int ph)
{
 
    switch (item)
    {
    case DISPLAY_ITEM_ACTIVE_ENERGY:
        display_imported_active_energy(ph);
        break;
    case DISPLAY_ITEM_ACTIVE_POWER:
        display_active_power(ph);
        break;
#if defined(REACTIVE_POWER_SUPPORT)
    case DISPLAY_ITEM_REACTIVE_POWER:
        display_reactive_power(ph);
        break;
#endif
#if defined(REACTIVE_ENERGY_SUPPORT)  ||  defined(TOTAL_REACTIVE_ENERGY_SUPPORT)
    case DISPLAY_ITEM_REACTIVE_ENERGY:
        display_imported_reactive_energy(ph);
        break;
#endif
#if defined(MAINS_FREQUENCY_SUPPORT)
    case DISPLAY_ITEM_MAINS_FREQUENCY:
        display_mains_frequency(ph);
        break;
#endif
#if defined(IRMS_SUPPORT)
    case DISPLAY_ITEM_CURRENT:
        display_rms_current(ph);
        break;
#endif
#if defined(VRMS_SUPPORT)
    case DISPLAY_ITEM_VOLTAGE:
        display_rms_voltage(ph);
        break;
#endif
#if defined(POWER_FACTOR_SUPPORT)
    case DISPLAY_ITEM_POWER_FACTOR:
        display_power_factor(ph);
        break;
#endif
#if defined(APPARENT_POWER_SUPPORT)
    case DISPLAY_ITEM_APPARENT_POWER:
        display_apparent_power(ph);
        break;
#endif
#if defined(RTC_SUPPORT)
    case DISPLAY_ITEM_DATE:
        display_current_date();
        break;
    case DISPLAY_ITEM_TIME:
        display_current_time();
        break;
#endif
#if defined(TEMPERATURE_SUPPORT)
    case DISPLAY_ITEM_TEMPERATURE:
        display_temperature();
        break;
#endif
#if defined(MULTI_RATE_SUPPORT)
    case DISPLAY_ITEM_CURRENT_TARIFF:
        display_current_tariff();
        break;
#endif
    }
}


int display_step = 0;
int display_select = 0;

void update_display(void)
{
    static const int8_t display_steps[] =
    {
        /* The following display sequence table should be defined in the hardware specific
           header file. */
        DISPLAY_STEP_SEQUENCE
    };


    /* Deal with the next stage of the sequenced display */
    display_clear_lines();
    for (;;)
    {
        if (display_steps[display_step] < 0)
        {
            if (display_steps[display_step] == DISPLAY_ITEM_SELECT_RESTART)
            {
                display_step = 0;
                continue;
            }
            display_select = -1 - display_steps[display_step++];
            break;
        }
        display_phase_icon(display_select);
        display_item(display_steps[display_step++], display_select);
    }



#if defined(DEDICATED_TIME_FIELD)
    // colocar aqui rotina para mostrar a hora constantemente tipo : display_current_time(); // TODO
#endif
    
#if defined(BATTERY_MONITOR_SUPPORT)  &&  defined(ICON_BATTERY)
    lcd_icon(ICON_BATTERY, (meter_status & STATUS_BATTERY_OK));
#endif
    
#if defined(LIMP_MODE_SUPPORT)
    if (operating_mode == OPERATING_MODE_LIMP)
        lcd_char('L', 1, 1);
#endif

}

#endif
