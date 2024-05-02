/*******************************************************************************
 *  accum48.c - Accumulate a 32 bit number into a 48 bit one.
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

void accum48(int16_t x[3], int32_t y)
{
#if defined(__MSP430__)
#if defined(__GNUC__)
    int16_t y_ex;

    __asm__ __volatile__ (
        " mov   %B[y],%[y_ex] \n"
        " rla   %[y_ex] \n"
        " subc  %[y_ex],%[y_ex] \n"
        " inv   %[y_ex] \n"
        " add   %A[y],0(%[x]) \n"
        " addc  %B[y],2(%[x]) \n"
        " addc  %[y_ex],4(%[x]) \n"
        : 
        : [x] "r"(x), [y] "r"(y), [y_ex] "r"(y_ex));
#else
#error "Don't know how to accum48"
#endif
#else
    int64_t acc;

    acc = (uint16_t) x[2];
    acc <<= 16;
    acc |= (uint16_t) x[1];
    acc <<= 16;
    acc |= (uint16_t) x[0];
    acc += y;
    x[0] = acc;
    acc >>= 16;
    x[1] = acc;
    acc >>= 16;
    x[2] = acc;
#endif
}
