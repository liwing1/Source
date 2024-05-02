/*******************************************************************************
 *  q1_15_mul.c - Multiply two 16 bit numbers
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

int16_t Q1_15_mul(int16_t x, int16_t y)
{
    int16_t x1;
    int32_t z;

#if defined(__GNUC__)  &&  defined(__MSP430__)
#if (defined(__MSP430_HAS_MPY__)  ||  defined(__MSP430_HAS_MPY32__))  &&  !defined(__TOOLKIT_USE_SOFT_MPY__)
    __asm__ (
"__MPY=0x130 \n"
"__MPYS=0x132 \n"
"__MAC=0x134 \n"
"__MACS=0x136 \n"
"__OP2=0x138 \n"
"__RESLO=0x13a \n"
"__RESHI=0x13c \n"
"__SUMEXT=0x13e \n"
        " push.w  r2 \n"
        " dint \n"
        " mov   %[x],&__MPYS \n"
        " mov   %[y],&__OP2 \n"
        " mov   &__RESHI,%B[z] \n"
        " mov   &__RESLO,%A[z] \n"
        " pop.w r2 \n"
        //Shift to Q1.15 format (i.e. the top 16 bits are returned)
        " rla   %A[z] \n"
        " rlc   %B[z] \n"
        " mov   %B[z],%[x1] \n"
        : [z] "=r"(z), [x1] "=r"(x1)
        : [x] "r"(x), [y] "r"(y));
#else
    z = 0;
    x1 = 0;
    __asm__ (
        " tst   %[x] \n"
        " jge   2f \n"
        " mov   #-1,%[x1] \n"
        " jmp   2f \n"
        "6: \n"
        " add   %[x],%A[z] \n"
        " addc  %[x1],%B[z] \n"
        "1: \n"
        " rla   %[x] \n"
        " rlc   %[x1] \n"
        "2: \n"
        " rra   %[y] \n"
        " jc    5f \n"
        " jne   1b \n"
        " jmp   4f \n"
        "5: \n"
        " sub   %[x],%A[z] \n"
        " subc  %[x1],%B[z] \n"
        "3: \n"
        " rla   %[x] \n"
        " rlc   %[x1] \n"
        " rra   %[y] \n"
        " jnc   6b \n"
        " cmp   #0xFFFF,%[y] \n"
        " jne   3b \n"
        "4: \n"
        //Shift to Q1.15 format (i.e. the top 16 bits are returned)
        " rla   %A[z] \n"
        " rlc   %B[z] \n"
        " mov   %B[z],%[x1] \n"
        : [z] "+r"(z), [x1] "+r"(x1)
        : [x] "r"(x), [y] "r"(y));
#endif
#else
    z = (int32_t) x*y;
    x1 = (z >> 15);
#endif
    return x1;
}
