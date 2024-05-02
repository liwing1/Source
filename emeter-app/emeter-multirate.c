/*******************************************************************************
 *  emeter-multirate.c -
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
#include <stdbool.h>
#include <string.h>
#if !defined(__MSP430__)
#include <stdio.h>
#endif
#if defined(__MSP430__)
#include <msp430.h>
#endif

#include "emeter-template.h"

#include <emeter-toolkit.h>
#include <emeter-metrology.h>

#include "emeter-multirate.h"

#if defined(MULTI_RATE_SUPPORT)

enum host_multirate_commands_e
{
    HOST_CMD_MULTIRATE_SET_PARAMETERS           = 0xC0,
    HOST_CMD_MULTIRATE_GET_PARAMETERS           = 0xC1,
    HOST_CMD_MULTIRATE_CLEAR_USAGE              = 0xC2,
    HOST_CMD_MULTIRATE_GET_USAGE                = 0xC3
};

uint8_t tariff_flags;

eeprom_history_t current_history;
uint8_t current_history_dirty;

/*! The current slot number in the usage history table */
uint8_t current_history_slot;

/*! The tariff type of the current day */
uint8_t current_day_type;

uint8_t current_tariff = 0xFF;

eeprom_daily_peak_t daily_peak;
uint8_t daily_peak_slot;

/*! The current slot in the five minute usage table. */
uint8_t current_five_minute_slot;

/*! This table is 30 minutes long, in 5 minute chunks. Usage is accumulated in
    5 minute chunks, to the centre of the 30 minute period of peak usage may be
    assessed to 5 minute accuracy. */
uint16_t five_minute_usages[6];

/* Add a one byte sumcheck to the passed message, in the byte after the message */
void add_sumcheck(void *buf, int len)
{
    uint8_t *s;
    int sum;

    sum = 0;
    s = (uint8_t *) buf;
    while (--len > 0)
        sum += *s++;
    *s = 0xFF - sum;
}

/* Check the passed message, which must include a one byte sumcheck, is OK */
int test_sumcheck(const void *buf, int len)
{
    const uint8_t *s;
    int sum;

    sum = 0;
    s = (uint8_t *) buf;
    while (len-- > 0)
        sum += *s++;
    return ((sum & 0xFF) == 0xFF);
}

void multirate_energy_pulse(void)
{
    five_minute_usages[current_five_minute_slot]++;
    if (++current_history.energy_lo == 0)
        ++current_history.energy_hi;
    current_history_dirty = true;
}

int find_next_cutoff_date(void)
{
    int year;
    int month;
    int day;
    int best_year;
    int best_month;
    int best_day;
    int best_slot;
    int i;
    eeprom_cutoff_date_t cutoff_date;

    /* We need to find the smallest date which is greater than the current date */
    inhibit_rtc_updates();
    year = rtc.year;
    month = rtc.month;
    day = rtc.day;
    enable_rtc_updates();
    best_year = 99;
    best_month = 12;
    best_day = 31;
    best_slot = -1;
    for (i = 0;  i < MULTIRATE_MAX_CUTOFF_DATES;  i++)
    {
        iicEEPROM_read(EEPROM_START_CUTOFF_DATES + i*sizeof(eeprom_cutoff_date_t), (void *) &cutoff_date, sizeof(cutoff_date));
        if (year < cutoff_date.year
            ||
                (year == cutoff_date.year
                &&
                    (month < cutoff_date.month
                    ||
                    (month == cutoff_date.month  &&  day < cutoff_date.day))))
        {
            if (best_year > cutoff_date.year
                ||
                    (best_year == cutoff_date.year
                    &&
                        (best_month > cutoff_date.month
                        ||
                        (best_month == cutoff_date.month  &&  best_day > cutoff_date.day))))
            {
                /* This is earlier, so use it */
                best_slot = i;
                best_year = cutoff_date.year;
                best_month = cutoff_date.month;
                best_day = cutoff_date.day;
            }
        }
    }
    return best_slot;
}

int find_previous_cutoff_date(void)
{
    int year;
    int month;
    int day;
    int best_year;
    int best_month;
    int best_day;
    int best_slot;
    int i;
    eeprom_cutoff_date_t cutoff_date;
    
    /* We need to find the largest date which is less than the current date */
    inhibit_rtc_updates();
    year = rtc.year;
    month = rtc.month;
    day = rtc.day;
    enable_rtc_updates();
    best_year = 0;
    best_month = 0;
    best_day = 0;
    best_slot = -1;
    for (i = 0;  i < MULTIRATE_MAX_CUTOFF_DATES;  i++)
    {
        iicEEPROM_read(EEPROM_START_CUTOFF_DATES + i*sizeof(eeprom_cutoff_date_t), (void *) &cutoff_date, sizeof(cutoff_date));
        if (year > cutoff_date.year
            ||
                (year == cutoff_date.year
                &&
                    (month > cutoff_date.month
                    ||
                    (month == cutoff_date.month  &&  day >= cutoff_date.day))))
        {
            if (best_year < cutoff_date.year
                ||
                    (best_year == cutoff_date.year
                    &&
                        (best_month < cutoff_date.month
                        ||
                        (best_month == cutoff_date.month  &&  best_day < cutoff_date.day))))
            {
                /* This is later, so use it */
                best_slot = i;
                best_year = cutoff_date.year;
                best_month = cutoff_date.month;
                best_day = cutoff_date.day;
            }
        }
    }
    return best_slot;
}

void new_tariff_day(void)
{
    int i;
    uint8_t day_type;
    int year;
    int month;
    int day;
    int wday;
    eeprom_holiday_t holiday;
    eeprom_cutoff_date_t cutoff_date;

    /* Should be called when the day changes and at reset, to work out
       today's type */
    inhibit_rtc_updates();
    year = rtc.year;
    month = rtc.month;
    day = rtc.day;
    enable_rtc_updates();
    for (i = 0;  i < MULTIRATE_MAX_HOLIDAYS;  i++)
    {
        iicEEPROM_read(EEPROM_START_HOLIDAYS + i*sizeof(eeprom_holiday_t), (void *) &holiday, sizeof(holiday));
        if (year == holiday.year
            &&
            month == holiday.month
            &&
            day == holiday.day)
        {
            /* Its a holiday */
            current_day_type = holiday.day_type;
            return;
        }
    }
    /* Not a holiday. Just use the regular weekday pattern */
    wday = weekday(&rtc);
    iicEEPROM_read(EEPROM_START_WEEKDAYS + (wday >> 1), (void *) &day_type, sizeof(day_type));
    current_day_type = ((wday & 1)  ?  (day_type >> 4)  :  day_type) & 0x0F;

    /* TODO: what if the real peak were a little before midnight */
    iicEEPROM_write(EEPROM_START_PEAKS + daily_peak_slot*sizeof(eeprom_daily_peak_t), (void *) &daily_peak, sizeof(daily_peak));
    if (++daily_peak_slot >= MULTIRATE_MAX_DAILY_PEAKS)
        daily_peak_slot = 0;

    daily_peak.usage = 0;
    daily_peak.hour = 0;
    daily_peak.minute = 0;
    
    /* Check if we have reached a billing cutoff point */
    iicEEPROM_read(EEPROM_START_CUTOFF_DATES + current_history_slot*sizeof(eeprom_cutoff_date_t), (void *) &cutoff_date, sizeof(cutoff_date));
    if (year > cutoff_date.year
        ||
        (year == cutoff_date.year
            &&
                (month > cutoff_date.month
                ||
                (month == cutoff_date.month  &&  day > cutoff_date.day))))
    {
        /* Its a cutoff point - find the next cutoff point, and start using the
           history slot specified for it. */
        /* If we didn't find a suitable slot, we continue accumulating where we
           are. At least that way we are gathering all the usage. */
        if ((i = find_next_cutoff_date()) >= 0)
            current_history_slot = i;
    }
}


void new_tariff_minute(void)
{
    int i;
    int n;
    int hour;
    int minute;
    eeprom_day_schedule_timeslot_t tariff;
    uint32_t energy;

    inhibit_rtc_updates();
    hour = rtc.hour;
    minute = rtc.minute;
    enable_rtc_updates();
    /* Default to the first timeslot */
    iicEEPROM_read(EEPROM_START_DAY_SCHEDULES + current_day_type*MULTIRATE_DAY_SCHEDULE_TIMESLOTS*sizeof(eeprom_day_schedule_timeslot_t), (void *) &tariff, sizeof(tariff));
    n = tariff.tariff;
    for (i = 1;  i < MULTIRATE_DAY_SCHEDULE_TIMESLOTS;  i++)
    {
        iicEEPROM_read(EEPROM_START_DAY_SCHEDULES + (current_day_type*MULTIRATE_DAY_SCHEDULE_TIMESLOTS + i)*sizeof(eeprom_day_schedule_timeslot_t), (void *) &tariff, sizeof(tariff));
        if (tariff.tariff)
        {
            if (tariff.start_hour > hour
                ||
                (tariff.start_hour == hour  &&  tariff.start_minute > minute))
            {
                n = tariff.tariff;
                break;
            }
        }
    }
    
    if (n != current_tariff)
    {
        /* Save current tariff values, and load the new set */
        write_history_slot(current_history_slot, current_tariff);
        read_history_slot(current_history_slot, n);
        current_tariff = n;
    }
    
    if (rtc.minute%5 == 0)
    {
        /* Deal with the time and size of the daily peak demand */
        energy = 0;
        for (i = 0;  i < 6;  i++)
            energy += five_minute_usages[i];
        if (energy > daily_peak.usage)
        {
            daily_peak.usage = energy;
            daily_peak.hour = rtc.hour;
            daily_peak.minute = rtc.minute;
        }
        five_minute_usages[current_five_minute_slot] = 0;
        if (++current_five_minute_slot >= 6)
            current_five_minute_slot = 0;
    }
    current_tariff = n;
}

int read_history_slot(int slot, int tariff)
{
    int i;
    int pos;
    eeprom_history_t history[3];
    int ok[3];

    if (!current_history_dirty)
        return 0;
    current_history_dirty = false;
    pos = (slot*MULTIRATE_DAY_SCHEDULES + tariff)*sizeof(eeprom_history_t);
    iicEEPROM_read(EEPROM_START_HISTORIES_A + pos, (void *) &history[0], sizeof(history[0]));
    iicEEPROM_read(EEPROM_START_HISTORIES_B + pos, (void *) &history[1], sizeof(history[1]));
    iicEEPROM_read(EEPROM_START_HISTORIES_C + pos, (void *) &history[2], sizeof(history[2]));
    /* Check the sumcheck of each copy */
    for (i = 0;  i < 3;  i++)
        ok[i] = test_sumcheck(&history[i], sizeof(history[0]));
    if (ok[0]  &&  ok[1])
    {
        if (memcmp(&history[0], &history[1], sizeof(history[0])) == 0)
        {
            /* Use the first copy */
            memcpy(&current_history, &history[0], sizeof(current_history));
            return 0;
        }
    }
    if (ok[1]  &&  ok[2])
    {
        if (memcmp(&history[1], &history[2], sizeof(history[1])) == 0)
        {
            /* Use the second copy */
            memcpy(&current_history, &history[1], sizeof(current_history));
            return 0;
        }
    }
    if (ok[0]  &&  ok[2])
    {
        if (memcmp(&history[0], &history[2], sizeof(history[0])) == 0)
        {
            /* Use the first copy */
            memcpy(&current_history, &history[0], sizeof(current_history));
            return 0;
        }
    }
    /* We don't have two matching copies, so we need to look for something
       who's sumcheck looks OK. */
    for (i = 0;  i < 3;  i++)
    {
        if (ok[i])
        {
            /* At least this one doesn't look corrupt. Use it. */
            memcpy(&current_history, &history[i], sizeof(current_history));
            return 0;
        }
    }
    /* Very bad - we can't find anything which looks reasonable */
    /* I guess we have to use something, so use the first copy. There is some
       hope the error is something not too serious. */
    memcpy(&current_history, &history[0], sizeof(current_history));
    return -1;
}

int write_history_slot(int slot, int tariff)
{
    int res;
    int pos;
    
    if (current_tariff >= MULTIRATE_DAY_SCHEDULES)
        return -1;
    /* Write all three copies in the EEPROM, setting the sumcheck before writing. */
    add_sumcheck(&current_history, sizeof(current_history));

    pos = (slot*MULTIRATE_DAY_SCHEDULES + tariff)*sizeof(eeprom_history_t);
    res = iicEEPROM_write(EEPROM_START_HISTORIES_A + pos, (void *) &current_history, sizeof(current_history));
    res |= iicEEPROM_write(EEPROM_START_HISTORIES_B + pos, (void *) &current_history, sizeof(current_history));
    res |= iicEEPROM_write(EEPROM_START_HISTORIES_C + pos, (void *) &current_history, sizeof(current_history));
    current_history_dirty = false;
    return res;
}

void tariff_management(void)
{
    if ((tariff_flags & TARIFF_NEW_DAY))
    {
        new_tariff_day();
        tariff_flags &= ~TARIFF_NEW_DAY;
    }
    if ((tariff_flags & TARIFF_NEW_MINUTE))
    {
        new_tariff_minute();
        tariff_flags &= ~TARIFF_NEW_MINUTE;
    }
}

void tariff_initialise(void)
{
    int i;
    
    /* Initial tariff information after starting from reset */
    if ((i = find_next_cutoff_date()) < 0)
        i = find_previous_cutoff_date();
    current_history_slot = i;
    new_tariff_day();
    new_tariff_minute();
}

void multirate_align_with_rtc(void)
{
    int i;
    /* The RTC has just been changed, so we need to align the multi-rate actiivities
       with the new time and date. */
    /* We may have hopped between cutoff dates. We need a full re-alignment with
       the new date. */
    if ((i = find_next_cutoff_date()) < 0)
        i = find_previous_cutoff_date();
    current_history_slot = i;
    /* Treat this like any new day, to pull the rest of the information into line. */
    new_tariff_day();
    new_tariff_minute();
}

int multirate_put(uint8_t *msg)
{
    switch (msg[1])
    {
    case 0x00:
        {
            eeprom_day_schedule_timeslot_t tariff;

            if (msg[2] >= MULTIRATE_DAY_SCHEDULES  ||  msg[3] >= MULTIRATE_DAY_SCHEDULE_TIMESLOTS)
                break;
            tariff.start_hour = msg[4];
            tariff.start_minute = msg[5];
            tariff.tariff = msg[6];
            return iicEEPROM_write(EEPROM_START_DAY_SCHEDULES + (msg[2]*MULTIRATE_DAY_SCHEDULE_TIMESLOTS + msg[3])*sizeof(eeprom_day_schedule_timeslot_t), (void *) &tariff, sizeof(tariff));
        }
    case 0x01:
        {
            eeprom_holiday_t holiday;

            if (msg[2] >= MULTIRATE_MAX_HOLIDAYS)
                break;
            holiday.year = msg[4];
            holiday.month = msg[5];
            holiday.day = msg[6];
            holiday.day_type = msg[7];
            holiday.spare = 0;
            return iicEEPROM_write(EEPROM_START_HOLIDAYS + msg[2]*sizeof(eeprom_holiday_t), (void *) &holiday, sizeof(holiday));
        }
    case 0x02:
        {
            uint8_t weekdays[4];

            weekdays[0] = msg[2] | (msg[3] << 4);
            weekdays[1] = msg[4] | (msg[5] << 4);
            weekdays[2] = msg[6] | (msg[7] << 4);
            weekdays[3] = msg[8] | (msg[9] << 4);
            return iicEEPROM_write(EEPROM_START_WEEKDAYS, (void *) weekdays, 4);
        }
    case 0x03:
        {
            eeprom_cutoff_date_t cutoff_date;
            if (msg[2] >= MULTIRATE_MAX_CUTOFF_DATES)
                break;
            cutoff_date.year = msg[4];
            cutoff_date.month = msg[5];
            cutoff_date.day = msg[6];
            cutoff_date.spare = 0;
            return iicEEPROM_write(EEPROM_START_CUTOFF_DATES + msg[2]*sizeof(eeprom_cutoff_date_t), (void *) &cutoff_date, sizeof(cutoff_date));
        }
    }
    return 0;
}

int multirate_get(uint8_t *msg, uint8_t *txmsg)
{
    txmsg[0] = HOST_CMD_MULTIRATE_GET_PARAMETERS;
    txmsg[1] = 0x80;
    txmsg[2] = msg[2];
    txmsg[3] = msg[3];
    switch (msg[1])
    {
    case 0x00:
        {
            eeprom_day_schedule_timeslot_t tariff;
            
            if (msg[2] >= MULTIRATE_DAY_SCHEDULES  ||  msg[3] >= MULTIRATE_DAY_SCHEDULE_TIMESLOTS)
                break;
            if (iicEEPROM_read(EEPROM_START_DAY_SCHEDULES + (msg[2]*MULTIRATE_DAY_SCHEDULE_TIMESLOTS + msg[3])*sizeof(eeprom_day_schedule_timeslot_t), (void *) &tariff, sizeof(tariff)))
            {
                txmsg[4] = tariff.start_hour;
                txmsg[5] = tariff.start_minute;
                txmsg[6] = tariff.tariff;
                txmsg[7] = 0;
                return 8;
            }
        }
        break;
    case 0x01:
        {
            eeprom_holiday_t holiday;
            
            if (msg[2] >= MULTIRATE_MAX_HOLIDAYS)
                break;
            if (iicEEPROM_read(EEPROM_START_HOLIDAYS + msg[2]*sizeof(eeprom_holiday_t), (void *) &holiday, sizeof(holiday)))
            {
                txmsg[4] = holiday.year;
                txmsg[5] = holiday.month;
                txmsg[6] = holiday.day;
                txmsg[7] = holiday.day_type;
                return 8;
            }
        }
        break;
    case 0x02:
        {
            uint8_t weekdays[4];

            if (iicEEPROM_read(EEPROM_START_WEEKDAYS, (void *) weekdays, 4))
            {
                txmsg[2] = weekdays[0] & 0x0F;
                txmsg[3] = (weekdays[0] >> 4) & 0x0F;
                txmsg[4] = weekdays[1] & 0x0F;
                txmsg[5] = (weekdays[1] >> 4) & 0x0F;
                txmsg[6] = weekdays[2] & 0x0F;
                txmsg[7] = (weekdays[2] >> 4) & 0x0F;
                txmsg[8] = weekdays[3] & 0x0F;
                txmsg[9] = (weekdays[3] >> 4) & 0x0F;
                return 10;
            }
        }
        break;
    case 0x03:
        {
            eeprom_cutoff_date_t cutoff_date;
        
            if (msg[2] >= MULTIRATE_MAX_CUTOFF_DATES)
                break;
            if (iicEEPROM_read(EEPROM_START_CUTOFF_DATES + msg[2]*sizeof(eeprom_cutoff_date_t), (void *) &cutoff_date, sizeof(cutoff_date)))
            {
                txmsg[4] = cutoff_date.year;
                txmsg[5] = cutoff_date.month;
                txmsg[6] = cutoff_date.day;
                txmsg[7] = 0;
                return 8;
            }
        }
        break;
    }
    txmsg[1] = 0x81;
    return 4;
}

int multirate_clear_usage(uint8_t *msg)
{
    static const int base[3] =
    {
        EEPROM_START_HISTORIES_A,
        EEPROM_START_HISTORIES_B,
        EEPROM_START_HISTORIES_C
    };

    switch (msg[1])
    {
    case 0x00:
        {
            eeprom_daily_peak_t daily_peak;
            
            if (msg[2] >= MULTIRATE_MAX_DAILY_PEAKS)
                break;
            memset(&daily_peak, 0, sizeof(daily_peak));
            return iicEEPROM_write(EEPROM_START_PEAKS + msg[2]*sizeof(eeprom_daily_peak_t), (void *) &daily_peak, sizeof(daily_peak));
        }
    case 0x01:
        {
            eeprom_history_t history;
        
            if (msg[2] >= MULTIRATE_MAX_CUTOFF_DATES  ||  msg[3] >= MULTIRATE_DAY_SCHEDULES)
                break;
            memset(&history, 0, sizeof(history));
            return iicEEPROM_write(base[0] + (msg[2]*MULTIRATE_DAY_SCHEDULES + msg[3])*sizeof(eeprom_history_t), (void *) &history, sizeof(history));
        }
    }
    return 0;
}

int multirate_get_usage(uint8_t *msg, uint8_t *txmsg)
{
    static const int base[3] =
    {
        EEPROM_START_HISTORIES_A,
        EEPROM_START_HISTORIES_B,
        EEPROM_START_HISTORIES_C
    };

    txmsg[0] = HOST_CMD_MULTIRATE_GET_USAGE;
    txmsg[1] = 0x80;
    txmsg[2] = msg[2];
    txmsg[3] = msg[3];
    switch (msg[1])
    {
    case 0x00:
        {
            eeprom_daily_peak_t daily_peak;
            
            if (msg[2] >= MULTIRATE_MAX_DAILY_PEAKS)
                break;
            if (iicEEPROM_read(EEPROM_START_PEAKS + msg[2]*sizeof(eeprom_daily_peak_t), (void *) &daily_peak, sizeof(daily_peak)))
            {
                ((uint32_t *) txmsg)[1] = daily_peak.usage;
                txmsg[8] = daily_peak.hour;
                txmsg[9] = daily_peak.minute;
                return 10;
            }
        }
        break;
    case 0x01:
        {
            eeprom_history_t history;
        
            if (msg[2] >= MULTIRATE_MAX_CUTOFF_DATES  ||  msg[3] >= MULTIRATE_DAY_SCHEDULES)
                break;
            if (iicEEPROM_read(base[0] + (msg[2]*MULTIRATE_DAY_SCHEDULES + msg[3])*sizeof(eeprom_history_t), (void *) &history, sizeof(history)))
            {
                txmsg[2] = 0;
                txmsg[3] = 0;
                ((uint32_t *) txmsg)[1] = history.energy_hi;
                ((uint32_t *) txmsg)[2] = history.energy_lo;
                return 12;
            }
        }
        break;
    }
    txmsg[1] = 0x81;
    return 4;
}
#endif
