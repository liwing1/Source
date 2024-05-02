/*******************************************************************************
 *  flash_replace16.c - Flash memory copy routines.
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
#include "emeter-toolkit.h"

#if defined(__MSP430__)
#define FSEG_A 0x01080      // Flash Segment A start address
#define FSEG_B 0x01000      // Flash Segment B start address

// ***********************************************************************
//  This routine makes the flash looks like EEPROM. It will erase and
//  replace just one word
//  This routine copies will erase SEGA and then image SEGB to SEGA
//  It will then erase SEGB and copy from SEGA back to SEGB all 128 bytes
//  except the one to be replaced.
// ***********************************************************************
void flash_replace16(int16_t *ptr, int16_t word)
{
    int *read_ptr;
    int *write_ptr;
    int w;
  
    //Optimise the case where the new and old values are the same
    if (*ptr == word)
        return;
    flash_clr((int *) FSEG_A);

    _DINT();
    //Set to write mode to prepare for copy
    FCTL3 = FWKEY;          /* Lock = 0 */
    FCTL1 = FWKEY | WRT;

    //Copy block B to A
    read_ptr = (int *) FSEG_B;
    write_ptr = (int *) FSEG_A;
    for (w = 0;  w < 64;  w++)
        *write_ptr++ = *read_ptr++;
    flash_clr((int *) FSEG_B);

    //Set to write mode to prepare for copy
    FCTL3 = FWKEY;          /* Lock = 0 */
    FCTL1 = FWKEY | WRT;

    //Copy block A to B, slipping in the new value at the right location
    read_ptr = (int *) FSEG_A;
    write_ptr = (int *) FSEG_B;
    for (w = 0;  w < 64;  w++, read_ptr++, write_ptr++)
    {
        if (write_ptr == ptr)
            *write_ptr = word;
        else
            *write_ptr = *read_ptr;
    }
    flash_secure();
    _EINT();
}
#endif
