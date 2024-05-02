/*******************************************************************************
 *  mul48.c - Multiply a 32 bit number by a 16 bit number, and return the top
 *            32 bits of the result.
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

int32_t mul48(int32_t x, int16_t y)
{
    int32_t z;
    int16_t residue;
#if defined(EMETER_TOOLKIT_SUPPORT_64BIT)
    int64_t tmp;
#endif

    z = x & 0xFFFF;
    z *= y;
    residue = z & 0xFFFF;
    z >>= 16;
    x >>= 16;
    x *= y;
    z += x;
#if defined(__GNUC__)  &&  defined(__MSP430__)
    __asm__ (
        " rla %[residue] \n"
        " rlc %A[z] \n"
        " rlc %B[z] \n"
        : [z] "+r"(z)
        : [residue] "r"(residue));
#else
    tmp = z;
    tmp <<= 1;
    if ((residue & 0x8000))
        tmp |= 1;
    z = tmp;
#endif
    return z;
}
