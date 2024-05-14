/*******************************************************************************
 *  setdco.c - Set the DCO frequency.
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
#include <msp430.h>
#include "emeter-toolkit.h"

#if defined(BCSCTL1_)  &&  defined(TACCR0_)
/* Set the DCO to a frequency specified in KHz */
void set_dco(int freq)
{
    unsigned int old_capture;
    unsigned int compare;
    unsigned int delta;
    
    delta = freq >> 4;

    CCTL2 = CCIS0 | CM0 | CAP;              /* Define CCR2, CAP, ACLK */
    TACTL = TASSEL1 | TACLR | MC1;          /* SMCLK, continuous mode */
    old_capture = 0;
    for (;;)
    {
        while (!(CCTL2 & CCIFG))
            /*dummy loop*/;                 /* Wait until capture occurs! */
        CCTL2 &= ~CCIFG;                    /* Capture occured, clear flag */
        compare = CCR2;                     /* Get current captured SMCLK */
        compare -= old_capture;             /* SMCLK difference */
        old_capture = CCR2;                 /* Save current captured SMCLK */
        if (delta == compare)
            break;                          /* If equal we are done */
        if (delta < compare)
        {
            /* DCO is too fast, slow it down */
            DCOCTL--;
            /* Did DCO role under? */
            if (DCOCTL == 0xFF)
                BCSCTL1--;                  /* Select next lower RSEL */
        }
        else
        {
            DCOCTL++;
            /* Did DCO role over? */
            if (DCOCTL == 0x00)
                BCSCTL1++;                  /* Select next higher RSEL */
        }
    }
    /* Stop CCR2 function */
    CCTL2 = 0;
    /* Stop Timer_A */
    TACTL = 0;
}
#endif
