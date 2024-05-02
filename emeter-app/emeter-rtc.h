/*******************************************************************************
 *  emeter-rtc.h -
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

/*! \file */

#if !defined(_EMETER_RTC_H_)
#define _EMETER_RTC_H_

#define RTC_DEFAULT_YEAR    14
#define RTC_DEFAULT_MONTH   2
#define RTC_DEFAULT_DOW     3   /* Day of week - 0=Sunday, 1=Monday, etc. */
#define RTC_DEFAULT_DAY     26
#define RTC_DEFAULT_HOUR    12
#define RTC_DEFAULT_MINUTE  0
#define RTC_DEFAULT_SECOND  0

typedef struct rtc_s
{
    uint8_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
    uint8_t sumcheck;
} rtc_t;

#if !defined(__MSP430_HAS_RTC_Cx__)
extern __uninitialized__ rtc_t rtc;

#if defined(CORRECTED_RTC_SUPPORT)
extern int16_t rtc_correction;
#endif

/* Return values for bump_rtc. These indicate the most significant element
   of the rtc which was changed. The rtc is not updated if it's sumcheck is
   incorrect, so nothing is updated for an inconsistent RTC setting. */
enum
{
    RTC_INCONSISTENT = 0,
    RTC_CHANGED_SECOND = 1,
    RTC_CHANGED_MINUTE = 2,
    RTC_CHANGED_HOUR = 3,
    RTC_CHANGED_DAY = 4,
    RTC_CHANGED_MONTH = 5,
    RTC_CHANGED_YEAR = 6
};

int bump_rtc(void);

int check_rtc_sumcheck(void);

void rtc_bumper(void);

void correct_rtc(void);

#if defined(CORRECTED_RTC_SUPPORT)
int32_t assess_rtc_speed(void);
#endif
#endif

enum
{
    RTC_STATUS_SKIP_A_SECOND = 0x01,
    RTC_STATUS_TICKER = 0x02
};

extern uint8_t rtc_status;

int weekday(rtc_t *s);

void get_rtc(uint8_t buf[6]);

void set_rtc(const uint8_t buf[6]);

void rtc_init(void);

#endif
