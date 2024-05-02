/*******************************************************************************
 *  hal_ucs.c -
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
#if defined(__GNUC__)
#include <signal.h>
#endif
#if defined(__MSP430__)
#include <msp430.h>
#endif
#include "hal_UCS.h"

static void wait_for_osc_ok(void)
{
    /* Check OFIFG fault flag */
    while (SFRIFG1 & OFIFG)
    {
        /* Clear OSC fault flags */
        UCSCTL7 &= ~(DCOFFG
#if defined(XT1LFOFFG)
                   | XT1LFOFFG
#endif
#if defined(XT1HFOFFG)
                   | XT1HFOFFG
#endif
#if defined(XT2OFFG)
                   | XT2OFFG
#endif
                   );
        /* Clear OFIFG fault flag */
        SFRIFG1 &= ~OFIFG;
    }
}

static unsigned int wait_for_osc_ok_with_timeout(unsigned int timeout)
{
    /* Check OFIFG fault flag */
    while ((SFRIFG1 & OFIFG)  &&  timeout--)
    {
        /* Clear OSC fault flags */
        UCSCTL7 &= ~(DCOFFG
#if defined(XT1LFOFFG)
                   | XT1LFOFFG
#endif
#if defined(XT1HFOFFG)
                   | XT1HFOFFG
#endif
#if defined(XT2OFFG)
                   | XT2OFFG
#endif
                   );
        /* Clear OFIFG fault flag */
        SFRIFG1 &= ~OFIFG;
    }
    return timeout;
}

/* Startup routine for 32kHz Crystal on LFXT1 */
void LFXT_Start(unsigned int xtdrive)
{
    /* Set the highest drive setting for XT1 startup */
    UCSCTL6_L |= (XT1DRIVE1_L | XT1DRIVE0_L);
    wait_for_osc_ok();
    /* Set the drive mode */
    UCSCTL6 = (UCSCTL6 & ~(XT1DRIVE_3)) | xtdrive;
}

/* Startup routine for 32kHz crystal on LFXT1 with timeout counter */
unsigned int LFXT_Start_Timeout(unsigned int xtdrive, unsigned int timeout)
{
    /* Set the highest drive setting for XT1 startup */
    UCSCTL6_L |= (XT1DRIVE1_L | XT1DRIVE0_L);
    timeout = wait_for_osc_ok_with_timeout(timeout);
    /* Set drive mode */
    UCSCTL6 = (UCSCTL6 & ~(XT1DRIVE_3)) | xtdrive;
    if (timeout)
        return UCS_STATUS_OK;
    return UCS_STATUS_ERROR;
}

/* Startup routine for XT1 */
void XT1_Start(unsigned int xtdrive)
{
    /* Enable XT1 and set XT1Drive */
    UCSCTL6 &= ~(XT1OFF & XT1DRIVE_3);
#if defined(XTS)
    UCSCTL6 |= (XTS & xtdrive);
#endif
    wait_for_osc_ok();
}

/* Startup routine for XT1 with timeout counter */
unsigned int XT1_Start_Timeout(unsigned int xtdrive, unsigned int timeout)
{
    /* Enable XT1 and set XT1Drive */
    UCSCTL6 &= ~(XT1OFF & XT1DRIVE_3);
#if defined(XTS)
    UCSCTL6 |= (XTS & xtdrive);
#endif
    timeout = wait_for_osc_ok_with_timeout(timeout);
    if (timeout)
        return UCS_STATUS_OK;
    return UCS_STATUS_ERROR;
}

/* Use XT1 in bypasss mode */
void XT1_Bypass(void)
{
    UCSCTL6 = XT1BYPASS;
    wait_for_osc_ok();
}

/* Startup routine for XT2 */
void XT2_Start(unsigned int xtdrive)
{
    /* Enable XT2 and set XT2Drive */
    UCSCTL6 &= ~(XT2OFF & XT1DRIVE_3);
    UCSCTL6 |= (xtdrive);
    wait_for_osc_ok();
}

/* Startup routine for XT2 with timeout counter */
unsigned int XT2_Start_Timeout(unsigned int xtdrive, unsigned int timeout)
{
    /* Enable XT2 */
    UCSCTL6 &= ~(XT2OFF & XT1DRIVE_3);
    /* Set XT2Drive */
    UCSCTL6 |= (xtdrive);
    timeout = wait_for_osc_ok_with_timeout(timeout);
    if (timeout)
        return UCS_STATUS_OK;
    return UCS_STATUS_ERROR;
}

/* Use XT2 in bypass mode */
/* On devices without XT2 this function will be present, but do nothing. */
void XT2_Bypass(void)
{
#if defined(XT2BYPASS)
    UCSCTL6 |= XT2BYPASS;
    wait_for_osc_ok();
#endif
}

/**
  * Initialize FLL of the UCS
  *
  * \param fsystem  required system frequency (MCLK) in kHz
  * \param ratio    ratio between fsystem and FLLREFCLK
  */
void Init_FLL(unsigned int fsystem, const unsigned int ratio)
{
    unsigned int d;
    unsigned int dco_div_bits;
    unsigned int mode;
    unsigned short globalInterruptState;

    /* Save the current state of FLL loop control, and disable loop control. */
    globalInterruptState = __get_SR_register() & SCG0;
    __bic_SR_register(SCG0);

    d = ratio;
    /* Have at least a divider of 2 */
    dco_div_bits = FLLD__2;
    if (fsystem > 16000)
    {
        d >>= 1;
        mode = 1;
    }
    else
    {
        fsystem <<= 1;
        mode = 0;
    }
    while (d > 512)
    {
        /* Set next higher div level */
        dco_div_bits = dco_div_bits + FLLD0;
        d >>= 1;
    }

    /* Set DCO to lowest tap */
    UCSCTL0 = 0x000;
    /* Reset FN bits */
    UCSCTL2 &= ~(0x3FF);
    UCSCTL2 = dco_div_bits | (d - 1);

    if (fsystem <= 630)
        UCSCTL1 = DCORSEL_0;
    else if (fsystem <  1250)
        UCSCTL1 = DCORSEL_1;
    else if (fsystem <  2500)
        UCSCTL1 = DCORSEL_2;
    else if (fsystem <  5000)
        UCSCTL1 = DCORSEL_3;
    else if (fsystem <  10000)
        UCSCTL1 = DCORSEL_4;
    else if (fsystem <  20000)
        UCSCTL1 = DCORSEL_5;
    else if (fsystem <  40000)
        UCSCTL1 = DCORSEL_6;
    else
        UCSCTL1 = DCORSEL_7;
    wait_for_osc_ok();
    if (mode == 1)                                              /* fsystem > 16000 */
        SELECT_MCLK_SMCLK(SELM__DCOCLK | SELS__DCOCLK);         /* Select DCOCLK */
    else
        SELECT_MCLK_SMCLK(SELM__DCOCLKDIV | SELS__DCOCLKDIV);   /* Select DCODIVCLK */

    /* Restore previous state */
    __bis_SR_register(globalInterruptState);
}

/* Initialize FLL of the UCS, and wait until it has settled
 *
 * \param fsystem  required system frequency (MCLK) in kHz
 * \param ratio    ratio between fsystem and FLLREFCLK
 */
void Init_FLL_Settle(unsigned int fsystem, const unsigned int ratio)
{
    /* We have 32 steps in the DCO / loop takes at least three cycles */
    /* (int)(32/3) = 10 */
    volatile unsigned int x = ratio*10;

    Init_FLL(fsystem, ratio);
    while (x--)
        __no_operation();
}
