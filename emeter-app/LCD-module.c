/*******************************************************************************
 *  LCD-module.c -
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

#if SUPPORT_LCD_MODULE
//These routines support an external LCD module attached to a
//port on an MSP430. This is not guaranteed to work for any
//particular module. However, most LCD modules have a lot in
//common, so this code is unlikely to require major changes to
//support any specific module.

//LCD command codes, preshifted for efficiency
#define READ        (6 << 5)
#define WRITE       (5 << 5)
#define RDMODIFYWR  (5 << 5)
#define WORKMODE    (4 << 5)

#define LCD_SYS_DIS     0x00    /* Turn off system oscillator and LCD bias generator */ 
#define LCD_SYS_EN      0x01    /* Turn on system osciallator */
#define LCD_LCD_OFF     0x02    /* Turn off LCD bias generator */
#define LCD_LCD_ON      0x03    /* Turn on LCD bias generator */
#define LCD_TIMER_DIS   0x04    /* Disable time base output */
#define LCD_WDT_DIS     0x05    /* Disable WDT flag output */
#define LCD_TIMER_EN    0x06    /* Enable time base output */
#define LCD_WDT_EN      0x07    /* Enable WDT flag output */
#define LCD_TONE_OFF    0x08    /* Turn off tone */
#define LCD_TONE_ON     0x09    /* Turn on tone */
#define LCD_CLR_TIMER   0x0C    /* Clear contents of time base generator */
#define LCD_CLR_WDT     0x0E    /* Clear contents of WDT */ 
#define LCD_XTAL_32K    0x14    /* System clock from crystal */
#define LCD_RC_256K     0x18    /* System clock from internal R/C */
#define LCD_EXT_256K    0x1C    /* System clock from external source */
#define LCD_BIAS_1_2    0x20
#define LCD_BIAS_1_3    0x21    
#define LCD_TONE_4K     0x40    /* Tone frequency 4kHz */
#define LCD_TONE_2K     0x60    /* Tone frequency 2kHz */
#define LCD_IRQ_DIS     0x80    /* Disable IRQ output */

static __inline__ void delay_a(void)
{
    register int i;

#if defined(__GNUC__)  &&  defined(__MSP430__)
    i = 10;
    __asm__ __volatile__ (
        "1: \n"
        " dec   %[i] \n"
        " jge   1b \n"
        : [i] "+r"(i));
#else
    for (i = 10;  i > 0;  i--)
        __no_operation();
#endif
}

static __inline__ void delay_b(void)
{
    register int i;
    
#if defined(__GNUC__)  &&  defined(__MSP430__)
    i = 90;
    __asm__ __volatile__ (
        "1: \n"
        " dec   %[i] \n"
        " jge   1b \n"
        : [i] "+r"(i));
#else
    for (i = 90;  i > 0;  i--)
        __no_operation();
#endif
}

static void LCDsend(uint8_t x, int len)
{
    for (  ;  len > 0;  len--)
    {
        delay_a();
        LCD_PORT_OUT &= ~LCD_WR_BIT;
        if (x & 0x80)
            LCD_PORT_OUT |= LCD_DATA_BIT;
        else 
            LCD_PORT_OUT &= ~LCD_DATA_BIT;
        delay_b();
        x <<= 1;
        LCD_PORT_OUT |= LCD_WR_BIT;
   }
}

static uint8_t LCDget(int len)
{
    uint8_t x;

    x = 0;
    LCD_PORT_OUT |= LCD_DATA_BIT;
    LCD_PORT_DIR &= (~LCD_DATA_BIT);
    for (  ;  len > 0;  len--)
    {
        delay_a();/**/
        delay_a();/**/
        delay_a();/**/
        delay_a();
        LCD_PORT_OUT &= ~LCD_RD_BIT;
        delay_a();
        delay_a();/**/
        delay_a();/**/
        delay_a();/**/
        delay_a();/**/
        delay_a();/**/
        x <<= 1;
        if ((LCD_PORT_IN & LCD_DATA_BIT))
            x |= 0x10;
        delay_a();/**/
        delay_a();/**/
        LCD_PORT_OUT |= LCD_RD_BIT;
        delay_a();/**/
        delay_a();/**/
    }
    LCD_PORT_DIR |= LCD_DATA_BIT;
    return  x;
}

static void LCDsend_mode(uint8_t x)
{
    LCD_PORT_OUT |= (LCD_CS_BIT | LCD_DATA_BIT | LCD_WR_BIT | LCD_RD_BIT);
    LCD_PORT_OUT &= ~LCD_CS_BIT;
    LCDsend(x, 3);
}

static void set_lcd_working_mode(uint8_t mode)
{
    LCDsend_mode(WORKMODE);
    LCDsend(mode, 8);
    delay_a();
    LCD_PORT_OUT &= ~(LCD_DATA_BIT | LCD_WR_BIT);
    delay_b();
    LCD_PORT_OUT |= (LCD_CS_BIT | LCD_WR_BIT);
}

//Initialise the LCD display, and set it to initially display
//all segments.
void LCDinit(void)
{
    int i;

    /* Configure the LCD driver communication port */
    LCD_PORT_DIR |= (LCD_CS_BIT | LCD_WR_BIT | LCD_RD_BIT | LCD_DATA_BIT);

    /* Turn on system oscillator */
    set_lcd_working_mode(LCD_SYS_EN);
    /* Turn on LCD bias generator */
    set_lcd_working_mode(LCD_LCD_ON);
    /* System clock source, on-chip RC oscillator RC 256K */
    set_lcd_working_mode(LCD_RC_256K);
    /* LCD 1/2 bias option 4 commons */
    set_lcd_working_mode(LCD_BIAS_1_2 + 8);
    for (i = 1;  i <= lcd_cells;  i++)
        LCDchar(CHAR_ALL, i);
}

void LCDsleep(void)
{
    //TODO:
}

void LCDawaken(void)
{
    //TODO:
}

//Change a specified group of LCD cells.
void LCDchars(const uint8_t *s, int pos, int len)
{
    pos--;
    for (  ;  len > 0;  --len)
    {
        LCDsend_mode(WRITE);
        //Send the address
        LCDsend(pos++ << 3, 6);
        //Send the data
        LCDsend(*s++, 8);
        //Deselect the LCD
        LCD_PORT_OUT |= LCD_CS_BIT;
    }
}

//Change a specified LCD cell.
void LCDchar(uint16_t ch, int pos)
{
    LCDsend_mode(WRITE);
    //Send the address
    LCDsend((pos - 1) << 3, 6);
    //Send the data
    LCDsend(ch, 8);
    //Deselect the LCD
    LCD_PORT_OUT |= LCD_CS_BIT;
}

//Note: This only modifies the top nibble, but that is usually all
//you want to control SEG_h
void LCDmodify_char(uint16_t ch, int pos, int on)
{
    uint8_t x;

    LCDsend_mode(RDMODIFYWR);
    //Send the address
    LCDsend((pos - 1) << 3, 6);
    //Get the current data
    x = LCDget(4);
    //Modify it
    if (on)
        x |= ch;
    else
        x &= ~ch;
    //Send the new data back
    LCDsend(x, 4);
    //Deselect the LCD
    LCD_PORT_OUT |= LCD_CS_BIT;
}

#endif
