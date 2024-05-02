/*******************************************************************************
 *  dc_filter16.c - DC estimation and removal for 24 bit signals
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

int32_t dc_filter24(int16_t p[3], int32_t x)
{
    int64_t tmp;

    tmp = p[2];
    tmp = tmp << 16;
    tmp |= p[1];
    tmp = tmp << 16;
    tmp |= p[0];
    tmp += ((((int32_t) x << 16) - tmp) >> 14);
    x -= (tmp >> 16);
    p[2] = (tmp >> 32) & 0xFFFF;
    p[1] = (tmp >> 16) & 0xFFFF;
    p[0] = tmp & 0xFFFF;
    return x;
}

int32_t dc_filter24_no_update(const int16_t p[3], int32_t x)
{
    int64_t tmp;

    tmp = p[2];
    tmp = tmp << 16;
    tmp |= p[1];
    x -= tmp;
    return x;
}

void dc_filter24_init(int16_t p[3], int16_t x)
{
#if !defined(__MSP430__)
    int64_t tmp;
#endif

    tmp = x << 16;
    p[2] = (tmp >> 32) & 0xFFFF;
    p[1] = (tmp >> 16) & 0xFFFF;
    p[0] = tmp & 0xFFFF;
}

int32_t dc_filter24_estimate(const int16_t p[3])
{
    int64_t tmp;

    tmp = p[2];
    tmp = tmp << 16;
    tmp |= p[1];
    tmp = tmp << 16;
    tmp |= p[0];
    return tmp >> 8;
}
