/*******************************************************************************
 *  emeter-app.h -
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

#if !defined(_METER_APP_H_)
#define _METER_APP_H_

#define SCFI0_LOW       (FLLD_1)        /* Freq = 1.024MHz */
#define SCFQCTL_LOW     (32 - 1)

#if defined(__MSP430_HAS_SD24_B__)
/* Run at 16.777216MHz */
#define DCO_CLOCK_SPEED 16
#define SCFI0_HIGH      (FN_3 | FLLD_4)
#define SCFQCTL_HIGH    (128 - 1)
#define IR_38K_DIVISOR  (220)
#elif defined(__MSP430_HAS_SD16_A3__)  ||  defined(__MSP430_HAS_SD16_A4__)  ||  defined(__MSP430_HAS_SD16_A6__)  ||  defined(__MSP430_HAS_SD16_A7__)
/* Run at 16.777216MHz */
#define DCO_CLOCK_SPEED 16
#define SCFI0_HIGH      (FN_3 | FLLD_4)
#define SCFQCTL_HIGH    (128 - 1)
#define IR_38K_DIVISOR  (220)
#else
/* Run at 8.388608MHz */
#define DCO_CLOCK_SPEED 8
#define SCFI0_HIGH      (FN_3 | FLLD_4)
#define SCFQCTL_HIGH    (64 - 1)
#define IR_38K_DIVISOR  (110)
#endif

/*! \brief Initialise all the data and peripherals, and prepare the machine to run
    after reset. */
void system_setup(void);

void display_startup_message(void);

#if defined(__MSP430__)  &&  defined(USE_WATCHDOG)
#define kick_watchdog()             WDTCTL = WDT_ARST_1000
#else
#define kick_watchdog()             /**/
#endif

#endif
