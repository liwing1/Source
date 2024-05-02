/*******************************************************************************
 *  div_sh48.c - Divide a 48 bit number by a 16 bit number, with a prescaling shift.
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

int32_t div_sh48(int16_t x[3], int sh, int16_t y)
{
    /* Preshift a 48 bit integer upwards by a specified amount. Then divide
       a 16 bit integer into the shifted 48 bit one. Expect the answer to be
       no greater than 32 bits, so return the answer as a 32 bit integer.
       A somewhat domain specific divide operation, but pretty useful when
       handling dot products. */
    int32_t x1;
    int32_t z;
    int16_t xx[3];
    
    /* Avoid any divide by zero trouble */
    if (y == 0)
        return 0;

    xx[0] = x[0];
    xx[1] = x[1];
    xx[2] = x[2];
    shift48(xx, sh);
    x1 = xx[2]%y;
    x1 <<= 16;
    x1 |= (uint16_t) xx[1];
    z = x1/y;
    x1 = x1%y;
    x1 <<= 16;
    x1 |= (uint16_t) xx[0];
    z = (z << 16) + x1/y;
    return z;
}
