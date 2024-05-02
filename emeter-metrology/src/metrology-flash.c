/*******************************************************************************
 *  metrology-flash.c - Flash reading and writing routines.
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

#include <signal.h>
#include <inttypes.h>
#if defined(__MSP430__)
#include <msp430.h>
#endif
#include <emeter-toolkit.h>

#include "metrology-flash.h"

#if defined(__MSP430__)

void flash_clr(int16_t *ptr)
{
    _DINT();
#if defined(__MSP430I4020__)
    if ((FCTL3 & LOCKSEG))
        FCTL3 = FWKEY | LOCKSEG;
#else
    FCTL3 = FWKEY;                      /* Lock = 0 */
#endif
    FCTL1 = FWKEY | ERASE;
    *((int *) ptr) = 0;                 /* Erase flash segment */
    _EINT();
}

void flash_write_int8(int8_t *ptr, int8_t value)
{
    _DINT();
#if defined(__MSP430I4020__)
    if ((FCTL3 & LOCKSEG))
        FCTL3 = FWKEY | LOCKSEG;
#else
    FCTL3 = FWKEY;                      /* Lock = 0 */
#endif
    FCTL1 = LOCK | WRT;
    *((int8_t *) ptr) = value;          /* Program the flash */
}

void flash_write_int16(int16_t *ptr, int16_t value)
{
    _DINT();
#if defined(__MSP430I4020__)
    if ((FCTL3 & LOCKSEG))
        FCTL3 = FWKEY | LOCKSEG;
#else
    FCTL3 = FWKEY;                      /* Lock = 0 */
#endif
    FCTL1 = FWKEY | WRT;
    *((int16_t *) ptr) = value;         /* Program the flash */
}

void flash_write_int32(int32_t *ptr, int32_t value)
{
    _DINT();
#if defined(__MSP430I4020__)
    if ((FCTL3 & LOCKSEG))
        FCTL3 = FWKEY | LOCKSEG;
#else
    FCTL3 = FWKEY;                      /* Lock = 0 */
#endif
    FCTL1 = FWKEY | WRT;
    *((int32_t *) ptr) = value;         /* Program the flash */
}

void flash_memcpy(void *to, const void *from, int len)
{
    const uint8_t *fromx;
    uint8_t *tox;

    fromx = (const uint8_t *) from;
    tox = (uint8_t *) to;
    _DINT();
#if defined(__MSP430I4020__)
    if ((FCTL3 & LOCKSEG))
        FCTL3 = FWKEY | LOCKSEG;
#else
    FCTL3 = FWKEY;                      /* Lock = 0 */
#endif
    FCTL1 = FWKEY | WRT;
    while (len)
    {
        *tox++ = *fromx++;
        len--;
    }
}

void flash_secure(void)
{
    _DINT();
    FCTL1 = FWKEY;                      /* Erase, write = 0 */
#if defined(__MSP430I4020__)
    if (!(FCTL3 & LOCKSEG))
        FCTL3 = FWKEY | LOCKSEG;
#else
    FCTL3 = FWKEY | LOCK;               /* Lock = 1 */
#endif
    _EINT();
}
#endif
