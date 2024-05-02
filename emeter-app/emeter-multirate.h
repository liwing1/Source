/*******************************************************************************
 *  emeter-multirate.h -
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

/* Several timeslots may be required through the day, to allow for lunchtime,
   early evening, etc.
   Three types of day are usually enough - weekdays, Saturday and Sunday/public
   holidays.
   The maximum number of actual tariffs is generally only 2 or 3 - something like
   normal, peak and overnight cheap rate
*/

/*! These structures are images of entries in the EEPROM */
/*! This structure contains the information about a daily tariff schedule. It is an image
    of the data stored in external EEPROM */
typedef struct
{
    /*! \brief The tariff type */
    uint8_t tariff:3;
    /*! \brief The starting hour of the day (0-23). */
    uint8_t start_hour:5;
    /*! \brief The starting minute of the hour (0-59). */
    uint8_t start_minute;
} eeprom_day_schedule_timeslot_t;

/*! This structure contains the information about a single public holiday. It is an image
    of the data stored in external EEPROM */
typedef struct
{
    /*! \brief The year of the holiday (0 = 2000, to 255 = 2255). */
    uint8_t year;
    /*! \brief The month of the holiday (1-12). */
    uint8_t month:4;
    /*! \brief Spare bits */
    uint8_t spare:4;
    /*! \brief The day of the month of the holiday (1-31). */
    uint8_t day:5;
    /*! \brief The tariff schedule type for the day. */
    uint8_t day_type:3;
} eeprom_holiday_t;

/*! This structure contains the information about a single billing cutoff date. It is an image
    of the data stored in external EEPROM */
typedef struct
{
    /*! \brief Billing cutoff year (0 = 2000, to 255 = 2255). */
    uint8_t year;
    /*! \brief Billing cutoff month (1-12). */
    uint8_t month:4;
    /*! \brief Spare bits */
    uint8_t spare:4;
    /*! \brief Billing cutoff day of the month (1-31). */
    uint8_t day;
} eeprom_cutoff_date_t;

/*! This structure contains the information about a daily peak usage record. It is an image
    of the data stored in external EEPROM */
typedef struct
{
    /*! \brief The peak 32 bit energy */
    uint32_t usage;
    /*! \brief The hour of the peak (0-23). */
    uint8_t hour;
    /*! \brief The minute of the hour of the peak (0-59). */
    uint8_t minute;
} eeprom_daily_peak_t;

/*! This structure contains the information about a usage record. It is an image of the data
    stored in external EEPROM */
typedef struct
{
    /*! \brief High 32 bits of the 64 bit energy */
    uint32_t energy_hi;
    /*! \brief Low 32 bits of the 64 bit energy */
    uint32_t energy_lo;
    /*! \brief Sumcheck used to check for corrupt usage information. */
    uint8_t sumcheck;
} eeprom_history_t;

/* Limits to size the various sections in the EEPROM */

#define MULTIRATE_DAY_SCHEDULES             8
#define MULTIRATE_DAY_SCHEDULE_TIMESLOTS    8
#define MULTIRATE_MAX_HOLIDAYS              32
#define MULTIRATE_MAX_CUTOFF_DATES          12
#define MULTIRATE_MAX_DAILY_PEAKS           31

/* Positions of the various data sections in the EEPROM */

/* A signature to identify the EEPROM status */
#define EEPROM_START_SIGNATURE      0
/* Tariff schedule */
#define EEPROM_START_DAY_SCHEDULES  4
/* Weekday tariff schedule pattern */
#define EEPROM_START_WEEKDAYS       (EEPROM_START_DAY_SCHEDULES + MULTIRATE_DAY_SCHEDULES*MULTIRATE_DAY_SCHEDULE_TIMESLOTS*sizeof(eeprom_day_schedule_timeslot_t))
/* Holidays, where special tariff schedules apply */
#define EEPROM_START_HOLIDAYS       (EEPROM_START_WEEKDAYS + 4)
/* Cutoff days */
#define EEPROM_START_CUTOFF_DATES   (EEPROM_START_HOLIDAYS + MULTIRATE_MAX_HOLIDAYS*sizeof(eeprom_holiday_t))
/* Daily usage peaks */
#define EEPROM_START_PEAKS          (EEPROM_START_CUTOFF_DATES + MULTIRATE_MAX_CUTOFF_DATES*sizeof(eeprom_cutoff_date_t))
/* Usage history entries */
#define EEPROM_START_HISTORIES_A    (EEPROM_START_PEAKS + MULTIRATE_MAX_DAILY_PEAKS*sizeof(eeprom_daily_peak_t))
#define EEPROM_START_HISTORIES_B    (EEPROM_START_HISTORIES_A + MULTIRATE_MAX_CUTOFF_DATES*MULTIRATE_DAY_SCHEDULES*sizeof(eeprom_history_t))
#define EEPROM_START_HISTORIES_C    (EEPROM_START_HISTORIES_B + MULTIRATE_MAX_CUTOFF_DATES*MULTIRATE_DAY_SCHEDULES*sizeof(eeprom_history_t))
/* End of EEPROM */
#define EEPROM_END                  (EEPROM_START_HISTORIES_C + MULTIRATE_MAX_CUTOFF_DATES*MULTIRATE_DAY_SCHEDULES*sizeof(eeprom_history_t))

extern uint8_t current_day_type;

extern eeprom_history_t current_history;

extern uint8_t current_five_minute_slot;
extern uint16_t five_minute_usages[6];
extern uint16_t peak_usage;
extern uint8_t peak_hour;
extern uint8_t peak_minute;

int read_history_slot(int slot, int tariff);
int write_history_slot(int slot, int tariff);
void new_tariff_minute(void);
void new_tarrif_day(void);

#if !defined(__MSP430__)
int dump_eeprom_data(void);
#endif
