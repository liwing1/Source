/*******************************************************************************
 *  bin2bcd16.c - 16 bit binary to BCD conversion.
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
#if !(defined(__GNUC__)  &&  defined(__MSP430__))
#include <stdio.h>
#endif
#include "emeter-toolkit.h"

void bin2bcd16(uint8_t bcd[3], uint16_t bin)
{
#if defined(__GNUC__)  &&  defined(__MSP430__)
    int i;
    uint16_t decimal_0;
    uint16_t decimal_1;

    decimal_0 =
    decimal_1 = 0;
    i = 16;
    __asm__ __volatile__(
        "1: \n"
        " rla.w     %[bin] \n"
        " dadd.w    %[decimal_0],%[decimal_0] \n"
        " dadd.w    %[decimal_1],%[decimal_1] \n"
        " dec.w     %[i] \n"
        " jnz       1b \n"
        " mov.b     %[decimal_1],0(%[bcd]) \n"
        " mov.b     %[decimal_0],2(%[bcd]) \n"
        " swpb      %[decimal_0] \n"
        " mov.b     %[decimal_0],1(%[bcd]) \n"
        : [bcd] "+r"(bcd), [decimal_0] "+r"(decimal_0), [decimal_1] "+r"(decimal_1)
        : [bin] "r"(bin), [i] "r"(i));
#else
    int i;
    char buf[6 + 1];
    
    sprintf (buf, "%06d", bin);
    for (i = 0;  i < 3;  i++)
        bcd[i] = ((buf[2*i] & 0x0F) << 4) | (buf[2*i + 1] & 0x0F);
#endif
}
