/*******************************************************************************
 *  metrology-flash.h - Flash reading and writing routines.
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

/*! \file */

#if !defined(_METROLOGY_FLASH_H_)
#define _METROLOGY_FLASH_H_

/*! Clearing a page of flash takes some time (see the device's data-sheet to
    find the exact time). Interrupts are disabled for the duration of this
    operation.
    \brief Clear a page of the flash memory.
    \param ptr A pointer to a location within the page to be cleared.
*/
void flash_clr(int16_t *ptr);

/*! Writing to flash takes some (see the device's data-sheet to
    find the exact time). Interrupts are disabled for the duration of this
    operation.
    \brief Write a byte of the flash memory.
    \param ptr A pointer to the location to be written.
    \param value The value to be written.
*/
void flash_write_int8(int8_t *ptr, int8_t value);

/*! Writing to flash takes some (see the device's data-sheet to
    find the exact time). Interrupts are disabled for the duration of this
    operation.
    \brief Write a 16 bit word of the flash memory.
    \param ptr A pointer to the location to be written.
    \param value The value to be written.
*/
void flash_write_int16(int16_t *ptr, int16_t value);

/*! Writing to flash takes some (see the device's data-sheet to
    find the exact time). Interrupts are disabled for the duration of this
    operation.
    \brief Write a 32 bit word of the flash memory.
    \param ptr A pointer to the location to be written.
    \param value The value to be written.
*/
void flash_write_int32(int32_t *ptr, int32_t value);

/*! Writing to flash takes some (see the device's data-sheet to
    find the exact time). Interrupts are disabled for the duration of this
    operation.
    \brief Copy a block of memory to the flash memory.
    \param to A pointer to the start of the memory block to be written.
    \param from A pointer to the start of the memory block to be copied from.
    \param len The number of consecutive bytes to be written.
*/
void flash_memcpy(void *to, const void *from, int len);

/*! \brief Secure the flash memory.
*/
void flash_secure(void);

#endif
