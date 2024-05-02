/*******************************************************************************
 *  isqrt16.c - Square root of a 16 bit number.
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
#include "emeter-toolkit.h"

uint16_t isqrt16(uint16_t h)
{
    uint16_t x;
    uint16_t y;
    int i;

#if defined(__GNUC__)  &&  defined(__MSP430__)
    x = 0x8000;
    y = 0;
    i = 16;
    __asm__ (
        "1: \n"
        " setc \n"
        " rlc   %x \n"
        " sub   %x,%y \n"
        " jhs   2f \n"
        " add   %x,%y \n"
        " sub   #2,%x \n"
        "2: \n"
        " inc   %x \n"
        " rla   %h \n"
        " rlc   %y \n"
        " rla   %h \n"
        " rlc   %y \n"
        " dec   %[i] \n"
        " jne   1b \n"
        : [x] "+r"(x), [y] "+r"(y)
        : [h] "r"(h), [i] "r"(i));
#else
    x =
    y = 0;
    for (i = 0;  i < 16;  i++)
    {
        x = (x << 1) | 1;
        if (y < x)
            x -= 2;
        else
            y -= x;
        x++;
        y <<= 1;
        if ((h & 0x8000))
            y |= 1;
        h <<= 1;
        y <<= 1;
        if ((h & 0x8000))
            y |= 1;
        h <<= 1;
    }
#endif
    return  x;
}
