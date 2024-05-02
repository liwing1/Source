/*******************************************************************************
 *  emeter-lcd.h -
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

#if !defined(_EMETER_LCD_H_)
#define _EMETER_LCD_H_

typedef uint16_t lcd_starburst_cell_t;
typedef uint8_t lcd_7segmnent_cell_t;

void lcd_init(void);
void lcd_sleep(void);
void lcd_awaken(void);

void lcd_clear(void);

void lcd_cell(uint8_t pattern, int pos);

void lcd_icon(int code, int on);

void lcd_char(char c, int field, int pos);
void lcd_string(const char *s, int field);
void lcd_string_only(const char *s, int field, int pos);

void lcd_decu16_sub_field(uint16_t value, int field, int after, int start, int width);

void lcd_dec16(int16_t value, int field, int after);
void lcd_dec32(int32_t value, int field, int after);
void lcd_dec64(int64_t value, int field, int after);
void lcd_decu16(uint16_t value, int field, int after);
void lcd_decu32(uint32_t value, int field, int after);
void lcd_decu64(uint64_t value, int field, int after);

#endif
