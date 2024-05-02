/*******************************************************************************
 *  dc_filter16.c - DC estimation and removal for 16 bit signals
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
#if defined(__MSP430__)
#include <msp430.h>
#endif
#include "emeter-toolkit.h"

int16_t dc_filter(int32_t *p, int16_t x)
{
#if defined(__GNUC__)  &&  defined(__MSP430__)
    int32_t z;

    z = 0x12345678;
    __asm__ (
        " mov   %[x],%B[z] \n"
        " clr   %A[z] \n"
        " sub   0(%[p]),%A[z] \n"       // subtract filter_state from (sample << 16)
        " subc  2(%[p]),%B[z] \n"
        " rla   %A[z] \n"               // >> 14
        " rlc   %B[z] \n"
        " jc    1f \n"
        " rlc   %A[z] \n"
        " rlc   %B[z] \n"
        " rlc   %A[z] \n"
        " and   #0x0003,%A[z] \n"
        " jmp   2f \n"
        "1: \n"
        " rlc   %A[z] \n"
        " rlc   %B[z] \n"
        " rlc   %A[z] \n"
        " bis   #0xFFFC,%A[z] \n"
        "2: \n"
        " add   0(%[p]),%B[z] \n"       // + filter_state
        " addc  2(%[p]),%A[z] \n"
        " mov   %B[z],0(%[p]) \n"       // save new filter_state
        " mov   %A[z],2(%[p]) \n"
        " sub   %A[z],%[x] \n"          // sample - filter_state
        : [x] "+r"(x)
        : [p] "r"(p), [z] "r"(z));
    return x;
#else
    *p += ((((int32_t) x << 16) - *p) >> 14);
    x -= (*p >> 16);
    return x;
#endif
}

int16_t dc_filter_no_update(const int32_t *p, int16_t x)
{
    return x - (*p >> 16);
}

void dc_filter_init(int32_t *p, int16_t x)
{
    *p = (int32_t) x << 16;
}

int32_t dc_filter16_estimate(const int32_t *p)
{
    return *p >> 8;
}

