/*******************************************************************************
 *  emeter-lcd.c -
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
#include <stdlib.h>
#include <stdbool.h>
#if defined(__MSP430__)
#include <msp430.h>
#endif

#include "emeter-template.h"

#include <emeter-toolkit.h>

#include "emeter-app.h"
#include "emeter-lcd.h"

#if defined(USE_7SEGMENT)
#undef SEG_a
#undef SEG_b
#undef SEG_c
#undef SEG_d
#undef SEG_e
#undef SEG_f
#undef SEG_g
#undef SEG_h

#define SEG_a   LCD_7SEGMENT_SEG_a
#define SEG_b   LCD_7SEGMENT_SEG_b
#define SEG_c   LCD_7SEGMENT_SEG_c
#define SEG_d   LCD_7SEGMENT_SEG_d
#define SEG_e   LCD_7SEGMENT_SEG_e
#define SEG_f   LCD_7SEGMENT_SEG_f
#define SEG_g   LCD_7SEGMENT_SEG_g
#define SEG_h   LCD_7SEGMENT_SEG_h

#include <lcd-7segment-segments.h>

/* Full scale ASCII is definitely only for starburst displays. Many of the
   characters in this table look unrecognisable, but making a full ASCII table
   keeps development simple. */
static const uint8_t lcd_7segment_ascii_table[] =
{
    CHAR_7SEGMENT_SPACE,
    CHAR_7SEGMENT_SPACE,            /* exclamation */
    CHAR_7SEGMENT_DOUBLEQUOTE,
    CHAR_7SEGMENT_SPACE,            /* hash */
    CHAR_7SEGMENT_DOLLAR,
    CHAR_7SEGMENT_SPACE,            /* percent */
    CHAR_7SEGMENT_SPACE,            /* ampersand */
    CHAR_7SEGMENT_QUOTE,
    CHAR_7SEGMENT_LEFTBRACKET,
    CHAR_7SEGMENT_RIGHTBRACKET,
    CHAR_7SEGMENT_SPACE,            /* asterisk */
    CHAR_7SEGMENT_SPACE,            /* plus */
    CHAR_7SEGMENT_SPACE,            /* comma */
    CHAR_7SEGMENT_MINUS,
    CHAR_7SEGMENT_SPACE,            /* full stop */
    CHAR_7SEGMENT_SLASH,
    CHAR_7SEGMENT_0,
    CHAR_7SEGMENT_1,
    CHAR_7SEGMENT_2,
    CHAR_7SEGMENT_3,
    CHAR_7SEGMENT_4,
    CHAR_7SEGMENT_5,
    CHAR_7SEGMENT_6,
    CHAR_7SEGMENT_7,
    CHAR_7SEGMENT_8,
    CHAR_7SEGMENT_9,
    CHAR_7SEGMENT_SPACE,            /* colon */
    CHAR_7SEGMENT_SPACE,            /* semi-colon */
    CHAR_7SEGMENT_SPACE,            /* less than */
    CHAR_7SEGMENT_EQUALS,
    CHAR_7SEGMENT_SPACE,            /* greater than */
    CHAR_7SEGMENT_QUESTION,
    CHAR_7SEGMENT_SPACE,            /* at sign */
    CHAR_7SEGMENT_A,
    CHAR_7SEGMENT_B,
    CHAR_7SEGMENT_C,
    CHAR_7SEGMENT_D,
    CHAR_7SEGMENT_E,
    CHAR_7SEGMENT_F,
    CHAR_7SEGMENT_G,
    CHAR_7SEGMENT_H,
    CHAR_7SEGMENT_I,
    CHAR_7SEGMENT_J,
    CHAR_7SEGMENT_K,
    CHAR_7SEGMENT_L,
    CHAR_7SEGMENT_M,
    CHAR_7SEGMENT_N,
    CHAR_7SEGMENT_O,
    CHAR_7SEGMENT_P,
    CHAR_7SEGMENT_Q,
    CHAR_7SEGMENT_R,
    CHAR_7SEGMENT_S,
    CHAR_7SEGMENT_T,
    CHAR_7SEGMENT_U,
    CHAR_7SEGMENT_V,
    CHAR_7SEGMENT_W,
    CHAR_7SEGMENT_X,
    CHAR_7SEGMENT_Y,
    CHAR_7SEGMENT_Z,
    CHAR_7SEGMENT_LEFTBRACKET,
    CHAR_7SEGMENT_BACKSLASH,
    CHAR_7SEGMENT_RIGHTBRACKET,
    CHAR_7SEGMENT_CARET,
    CHAR_7SEGMENT_UNDERSCORE,
    CHAR_7SEGMENT_BACKQUOTE,
    CHAR_7SEGMENT_a,
    CHAR_7SEGMENT_b,
    CHAR_7SEGMENT_c,
    CHAR_7SEGMENT_d,
    CHAR_7SEGMENT_e,
    CHAR_7SEGMENT_f,
    CHAR_7SEGMENT_g,
    CHAR_7SEGMENT_h,
    CHAR_7SEGMENT_i,
    CHAR_7SEGMENT_j,
    CHAR_7SEGMENT_k,
    CHAR_7SEGMENT_l,
    CHAR_7SEGMENT_m,
    CHAR_7SEGMENT_n,
    CHAR_7SEGMENT_o,
    CHAR_7SEGMENT_p,
    CHAR_7SEGMENT_q,
    CHAR_7SEGMENT_r,
    CHAR_7SEGMENT_s,
    CHAR_7SEGMENT_t,
    CHAR_7SEGMENT_u,
    CHAR_7SEGMENT_v,
    CHAR_7SEGMENT_w,
    CHAR_7SEGMENT_x,
    CHAR_7SEGMENT_y,
    CHAR_7SEGMENT_z,
    CHAR_7SEGMENT_LEFTBRACKET,
    CHAR_7SEGMENT_VERTICALBAR,
    CHAR_7SEGMENT_RIGHTBRACKET,
    CHAR_7SEGMENT_SPACE,            /* squiggle */
    CHAR_7SEGMENT_SPACE             /* delete */
};

#undef SEG_a
#undef SEG_b
#undef SEG_c
#undef SEG_d
#undef SEG_e
#undef SEG_f
#undef SEG_g
#undef SEG_h
#undef SEG_i
#endif

#if defined(USE_STARBURST)
#undef SEG_12
#undef SEG_1
#undef SEG_3
#undef SEG_5
#undef SEG_6
#undef SEG_7
#undef SEG_9
#undef SEG_11

#define SEG_a   LCD_STARBURST_SEG_a
#define SEG_b   LCD_STARBURST_SEG_b
#define SEG_c   LCD_STARBURST_SEG_c
#define SEG_d   LCD_STARBURST_SEG_d
#define SEG_e   LCD_STARBURST_SEG_e
#define SEG_f   LCD_STARBURST_SEG_f
#define SEG_g   LCD_STARBURST_SEG_g
#define SEG_h   LCD_STARBURST_SEG_h
#define SEG_i   LCD_STARBURST_SEG_i

#define SEG_12  LCD_STARBURST_SEG_12
#define SEG_1   LCD_STARBURST_SEG_1
#define SEG_3   LCD_STARBURST_SEG_3
#define SEG_5   LCD_STARBURST_SEG_5
#define SEG_6   LCD_STARBURST_SEG_6
#define SEG_7   LCD_STARBURST_SEG_7
#define SEG_9   LCD_STARBURST_SEG_9
#define SEG_11  LCD_STARBURST_SEG_11

#include <lcd-starburst-segments.h>

/* Full scale ASCII is definitely only for starburst displays */
static const uint16_t lcd_starburst_ascii_table[] =
{
    CHAR_STARBURST_SPACE,
    CHAR_STARBURST_SPACE,         /* exclamation */
    CHAR_STARBURST_DOUBLEQUOTE,
    CHAR_STARBURST_SPACE,         /* hash */
    CHAR_STARBURST_DOLLAR,
    CHAR_STARBURST_SPACE,         /* percent */
    CHAR_STARBURST_SPACE,         /* ampersand */
    CHAR_STARBURST_QUOTE,
    CHAR_STARBURST_LEFTBRACKET,
    CHAR_STARBURST_RIGHTBRACKET,
    CHAR_STARBURST_ASTERISK,
    CHAR_STARBURST_PLUS,
    CHAR_STARBURST_SPACE,         /* comma */
    CHAR_STARBURST_MINUS,
    CHAR_STARBURST_SPACE,         /* full stop */
    CHAR_STARBURST_SLASH,
    CHAR_STARBURST_0,
    CHAR_STARBURST_1,
    CHAR_STARBURST_2,
    CHAR_STARBURST_3,
    CHAR_STARBURST_4,
    CHAR_STARBURST_5,
    CHAR_STARBURST_6,
    CHAR_STARBURST_7,
    CHAR_STARBURST_8,
    CHAR_STARBURST_9,
    CHAR_STARBURST_SPACE,         /* colon */
    CHAR_STARBURST_SPACE,         /* semi-colon */
    CHAR_STARBURST_LT,
    CHAR_STARBURST_EQUALS,
    CHAR_STARBURST_GT,
    CHAR_STARBURST_QUESTION,
    CHAR_STARBURST_SPACE,         /* at sign */
    CHAR_STARBURST_A,
    CHAR_STARBURST_B,
    CHAR_STARBURST_C,
    CHAR_STARBURST_D,
    CHAR_STARBURST_E,
    CHAR_STARBURST_F,
    CHAR_STARBURST_G,
    CHAR_STARBURST_H,
    CHAR_STARBURST_I,
    CHAR_STARBURST_J,
    CHAR_STARBURST_K,
    CHAR_STARBURST_L,
    CHAR_STARBURST_M,
    CHAR_STARBURST_N,
    CHAR_STARBURST_O,
    CHAR_STARBURST_P,
    CHAR_STARBURST_Q,
    CHAR_STARBURST_R,
    CHAR_STARBURST_S,
    CHAR_STARBURST_T,
    CHAR_STARBURST_U,
    CHAR_STARBURST_V,
    CHAR_STARBURST_W,
    CHAR_STARBURST_X,
    CHAR_STARBURST_Y,
    CHAR_STARBURST_Z,
    CHAR_STARBURST_LEFTBRACKET,
    CHAR_STARBURST_BACKSLASH,
    CHAR_STARBURST_RIGHTBRACKET,
    CHAR_STARBURST_CARET,
    CHAR_STARBURST_UNDERSCORE,
    CHAR_STARBURST_BACKQUOTE,
    CHAR_STARBURST_a,
    CHAR_STARBURST_b,
    CHAR_STARBURST_C,
    CHAR_STARBURST_d,
    CHAR_STARBURST_e,
    CHAR_STARBURST_f,
    CHAR_STARBURST_g,
    CHAR_STARBURST_h,
    CHAR_STARBURST_i,
    CHAR_STARBURST_j,
    CHAR_STARBURST_k,
    CHAR_STARBURST_l,
    CHAR_STARBURST_m,
    CHAR_STARBURST_n,
    CHAR_STARBURST_o,
    CHAR_STARBURST_p,
    CHAR_STARBURST_q,
    CHAR_STARBURST_r,
    CHAR_STARBURST_s,
    CHAR_STARBURST_t,
    CHAR_STARBURST_u,
    CHAR_STARBURST_v,
    CHAR_STARBURST_w,
    CHAR_STARBURST_x,
    CHAR_STARBURST_y,
    CHAR_STARBURST_z,
    CHAR_STARBURST_LEFTBRACKET,
    CHAR_STARBURST_VERTICALBAR,
    CHAR_STARBURST_RIGHTBRACKET,
    CHAR_STARBURST_SPACE,         /* squiggle */
    CHAR_STARBURST_SPACE          /* delete */
};

#undef SEG_a
#undef SEG_b
#undef SEG_c
#undef SEG_d
#undef SEG_e
#undef SEG_f
#undef SEG_g
#undef SEG_h
#undef SEG_i

#undef SEG_12
#undef SEG_1
#undef SEG_3
#undef SEG_5
#undef SEG_6
#undef SEG_7
#undef SEG_9
#undef SEG_11
#endif

enum
{
    LCD_MODE_7SEGMENT = 0,
    LCD_MODE_STARBURST = 1
};

struct lcd_field_s
{
    int8_t start_location;  /* First location of the field in LCD memory */
    int8_t step;            /* The way we step through the field in the LCD memory (+ or -) */
    int8_t cells;           /* The number of character cells in the field */
    int8_t mode;            /* The mode in which the field works */
    uint16_t segment_mask;
    const uint8_t *minus_icons;
    const uint8_t *dp_icons;
};

DISPLAY_MINUS_SIGNS

DISPLAY_DECIMAL_POINTS

static const struct lcd_field_s lcd_fields[] =
{
    /* The following display fields table should be defined in the hardware specific
       header file. */
    DISPLAY_FIELDS
};

/* Initialise the LCD display, and set it to initially display all segments. */
void lcd_init(void)
{
    int i;

    for (i = 0;  i < 20;  i++)
        LCDMEM[i] = 0;
#if defined(__MSP430_HAS_LCD__)  ||  defined(__MSP430_HAS_LCD4__)
    LCDCTL = LCD_INIT;
    #if defined(LCDPCTL0_INIT)
    LCDPCTL0 = LCDPCTL0_INIT;
    #endif
    #if defined(LCDPCTL1_INIT)
    LCDPCTL1 = LCDPCTL1_INIT;
    #endif

#elif defined(__MSP430_HAS_LCD_A__)
    /* LCD-A controller setup */
    LCDACTL = LCDACTL_INIT;
    LCDAPCTL0 = LCDAPCTL0_INIT;
    LCDAPCTL1 = LCDAPCTL1_INIT;
    LCDAVCTL0 = LCDAVCTL0_INIT;
    LCDAVCTL1 = LCDAVCTL1_INIT;
#elif defined(__MSP430_HAS_LCD_C__)
    /* LCD-C controller setup */
    LCDCCTL0 = LCDCCTL0_INIT;
    LCDCCTL1 = LCDCCTL1_INIT;
    LCDCBLKCTL = LCDCBLKCTL_INIT;
    LCDCMEMCTL = LCDCMEMCTL_INIT;
    LCDCVCTL = LCDCVCTL_INIT;
    LCDCPCTL0 = LCDCPCTL0_INIT;
    LCDCPCTL1 = LCDCPCTL1_INIT;
    LCDCPCTL2 = LCDCPCTL2_INIT;
    LCDCCPCTL = LCDCCPCTL_INIT;
#endif
    for (i = 0;  i < 20;  i++)
        LCDMEM[i] = 0xFF;
}

void lcd_sleep(void)
{
#if defined(__MSP430_HAS_LCD__)  ||  defined(__MSP430_HAS_LCD4__)
    LCDCTL &= ~LCDON;
#elif defined(__MSP430_HAS_LCD_A__)
    LCDACTL &= ~(LCDON);
#elif defined(__MSP430_HAS_LCD_C__)
    LCDCCTL0 &= ~LCDON;
#endif
}

void lcd_awaken(void)
{
#if defined(__MSP430_HAS_LCD_A__)
    LCDACTL |= LCDON;
#elif defined(__MSP430_HAS_LCD_C__)
    LCDCCTL0 |= LCDON;
#else
    LCDCTL |= LCDON;
#endif
}

void lcd_clear(void)
{
    int i;

    for (i = 0;  i < 20;  i++)
        LCDMEM[i] = 0;
}

/* Change a specified LCD cell. */
static void lcd_field_cell(uint8_t ch, int field, int pos)
{
#if defined(USE_STARBURST)
    uint16_t chx;
#endif

    if (lcd_fields[field].step < 0)
        pos = -pos;
#if defined(USE_STARBURST)
    if (lcd_fields[field].mode == LCD_MODE_STARBURST)
    {
        pos = lcd_fields[field].start_location - 1 + (pos << 1);
        chx = lcd_starburst_ascii_table[ch - ' '];
        LCDMEM[pos] &= lcd_fields[field].segment_mask;
        LCDMEM[pos] |= chx & 0xFF;
        pos += (lcd_fields[field].step >> 1);
    #if defined(MSP430F6726_EVM_V1_LCD_FUDGE)
        if (pos == 11)
        {
            uint8_t xx;

            xx = ((LCDMEM[19] & 0x0F) << 4) | ((LCDMEM[19] & 0xF0) >> 4);
            xx &= (lcd_fields[field].segment_mask >> 8);
            xx |= chx >> 8;
            LCDMEM[19] = ((xx & 0x0F) << 4) | ((xx & 0xF0) >> 4);
        }
        else
    #endif
        {
            LCDMEM[pos] &= (lcd_fields[field].segment_mask >> 8);
            LCDMEM[pos] |= chx >> 8;
        }
    }
    else
#endif
    {
        pos = lcd_fields[field].start_location - 1 + pos;
        LCDMEM[pos] = (LCDMEM[pos] & lcd_fields[field].segment_mask) | lcd_7segment_ascii_table[ch - ' '];
    }
}

void lcd_icon(int code, int on)
{
    if (code)
    {
        if (on)
            LCDMEM[(code & 0x1F) - 1] |= (1 << (code >> 5));
        else
            LCDMEM[(code & 0x1F) - 1] &= ~(1 << (code >> 5));
    }
}

void lcd_cell(uint8_t pattern, int pos)
{
    LCDMEM[pos] = pattern;
}

static void set_decimal_point(int field, int after)
{
    int i;

    /* Clear them all, then set the one we want */
    for (i = 0;  i < lcd_fields[field].cells;  i++)
        lcd_icon(lcd_fields[field].dp_icons[i], false);
    if (after)
        lcd_icon(lcd_fields[field].dp_icons[after], true);
}

void lcd_char(char c, int field, int pos)
{
    lcd_field_cell(c, field - 1, pos - 1);
}

void lcd_string(const char *s, int field)
{
    int pos;

    set_decimal_point(--field, 0);
    lcd_icon(lcd_fields[field].minus_icons[0], false);
    for (pos = 0;  *s  &&  pos < lcd_fields[field].cells;  pos++)
        lcd_field_cell(*s++, field, pos);
    for (  ;  pos < lcd_fields[field].cells;  pos++)
        lcd_field_cell(' ', field, pos);
}

void lcd_string_only(const char *s, int field, int pos)
{
    set_decimal_point(--field, 0);
    lcd_icon(lcd_fields[field].minus_icons[0], false);
    for (pos--;  *s  &&  pos < lcd_fields[field].cells;  pos++)
        lcd_field_cell(*s++, field, pos);
}

static int auto_range(int digits, int after, int field, int cells)
{
    int shift;

    shift = 0;
    if (after < 0)
    {
        /* Autorange */
        after = -after;
        if (digits > cells)
        {
            if (digits - after > cells)
            {
                /* This value is over-range */
                return -1;
            }
            shift = digits - cells;
            if (shift > after)
                shift = after;
        }
        /* Make sure things display OK if the field width won't hold all the fractional
           digits we have. */
        if (after - shift >= cells)
            shift = after - cells + 1;
        /* Some displays do not have a decimal point for every digit, so we might need
           to shift down a little, to get things to display properly. */
        while (after > shift  &&  lcd_fields[field].dp_icons[after - shift] == 0)
            shift++;
    }
    else
    {
        if (digits > cells)
        {
            /* This value is over-range */
            return -1;
        }
    }
    return shift;
}

/* Display a small field within an LCD field. This is suitable for displaying
   things like time and date, where several pieces need to be contatenated. */
void lcd_decu16_sub_field(uint16_t value, int field, int after, int start, int width)
{
    int i;
    int flag;
    int pos;
    unsigned int digit;
    uint8_t bcd[3];

    pos = start - 1;
    field--;
    flag = 1;
    bin2bcd16(bcd, value);
    lcd_icon(lcd_fields[field].minus_icons[0], false);
    for (i = width;  i > 6;  i--)
        lcd_field_cell(' ', field, pos++);
    for (i = 6 - i;  i < 6;  i++)
    {
        digit = bcd[i >> 1];
        if ((i & 1) == 0)
            digit >>= 4;
        digit &= 0x0F;
        if (digit  ||  i >= (5 - after))
            flag = 0;
        lcd_field_cell((flag == 0)  ?  (digit + '0')  :  ' ', field, pos++);
    }
}   

/* Display an unsigned 16 bit integer, with leading zero suppression. "after" is the number of
   digits which are after the decimal point. The leading zero suppression allows for this. */
void lcd_decu16(uint16_t value, int field, int after)
{
    after = abs(after);
    lcd_decu16_sub_field(value, field, after, 1, lcd_fields[field - 1].cells);
    set_decimal_point(field - 1, after);
}   

/* Display a signed 16 bit integer, with leading zero suppression. "after" is the number of
   digits which are after the decimal point. The leading zero suppression allows for this. */
void lcd_dec16(int16_t value, int field, int after)
{
    int i;
    int flag;
    int pos;
    unsigned int digit;
    uint8_t bcd[3];

    after = abs(after);
    pos = 0;
    field--;
    i = lcd_fields[field].cells;
    if (value < 0)
    {
        flag = 2;
        value = -value;
        if (lcd_fields[field].minus_icons[0] == 0)
        {
            /* Make space for the minus, as we have no special icon for when it
               is in the leading position of the field. */
            lcd_field_cell(' ', field, pos++);
            i--;
        }
    }
    else
    {
        flag = 1;
    }
    bin2bcd16(bcd, value);
    lcd_icon(lcd_fields[field].minus_icons[0], false);
    for (  ;  i > 6;  i--)
        lcd_field_cell(' ', field, pos++);
    for (i = 6 - i;  i < 6;  i++)
    {
        digit = bcd[i >> 1];
        if ((i & 1) == 0)
            digit >>= 4;
        digit &= 0x0F;
        if (digit  ||  i >= (5 - after))
        {
            if (flag == 2)
                lcd_icon(lcd_fields[field].minus_icons[pos], true);
            flag = 0;
        }
        lcd_field_cell((flag == 0)  ?  (digit + '0')  :  ' ', field, pos++);
    }
    set_decimal_point(field, after);
}   

static void lcd_dec32_common(uint32_t value, int field, int after, int flag, int reserve_first_cell)
{
    uint8_t bcd[5];
    unsigned int digit;
    int shift;
    int i;
    int pos;
    int cells;

    pos = 0;
    cells = lcd_fields[field].cells;
    if (reserve_first_cell)
    {
        pos = 1;
        cells--;
    }

    bin2bcd32(bcd, value);
    for (i = 0;  i < 20;  i++)
    {
        digit = bcd[i >> 1];
        if ((i & 1) == 0)
            digit >>= 4;
        digit &= 0x0F;
        if (digit != 0)
            break;
    }

    shift = auto_range(10 - i, after, field, cells);
    if (shift < 0)
    {
        /* This value is over-range */
        lcd_string("High", field + 1);
        return;
    }
    after = abs(after);
    lcd_icon(lcd_fields[field].minus_icons[0], false);
    for (i = 10 - cells - shift;  i < 10 - shift;  i++)
    {
        digit = bcd[i >> 1];
        if ((i & 1) == 0)
            digit >>= 4;
        digit &= 0x0F;
        if (digit  ||  i >= (9 - after))
        {
            if (flag == 2)
                lcd_icon(lcd_fields[field].minus_icons[pos], true);
            flag = 0;
        }
        lcd_field_cell((flag == 0)  ?  (digit + '0')  :  ' ', field, pos++);
    }
    set_decimal_point(field, after - shift);
}

/* Display an unsigned 32 bit integer, with leading zero suppression. "after" is the number of
   digits which are after the decimal point. The leading zero suppression allows for this. If
   after is negative, its modulus is the maximum number of digits after the decimal point, but
   the decimal places will automatically be dropped to fit a large number. If the number will not
   fit at all, "High" is displayed. */
void lcd_decu32(uint32_t value, int field, int after)
{
    field--;
    lcd_dec32_common(value, field, after, 1, false);
}

/* Display a signed 32 bit integer, with leading zero suppression. "after" is the number of
   digits which are after the decimal point. The leading zero suppression allows for this. If
   after is negative, its modulus is the maximum number of digits after the decimal point, but
   the decimal places will automatically be dropped to fit a large number. If the number will not
   fit at all, "High" is displayed. */
void lcd_dec32(int32_t value, int field, int after)
{
    uint32_t mod_value;
    int flag;
    int reserve_first_cell;

    field--;

    reserve_first_cell = false;
    if (value < 0)
    {
        mod_value = -value;
        if (lcd_fields[field].minus_icons[0] == 0)
        {
            /* Make space for the minus, as we have no special icon for when it
               is in the leading position of the field. */
            lcd_field_cell(' ', field, 0);
            reserve_first_cell = true;
        }
        flag = 2;
    }
    else
    {
        mod_value = value;
        flag = 1;
    }
    lcd_dec32_common(mod_value, field, after, flag, reserve_first_cell);
}

static void lcd_dec64_common(uint64_t value, int field, int after, int flag, int reserve_first_cell)
{
    uint8_t bcd[10];
    unsigned int digit;
    int shift;
    int i;
    int pos;
    int cells;

    pos = 0;
    cells = lcd_fields[field].cells;
    if (reserve_first_cell)
    {
        pos = 1;
        cells--;
    }

    bin2bcd64(bcd, value);
    for (i = 0;  i < 20;  i++)
    {
        digit = bcd[i >> 1];
        if ((i & 1) == 0)
            digit >>= 4;
        digit &= 0x0F;
        if (digit != 0)
            break;
    }

    shift = auto_range(20 - i, after, field, cells);
    if (shift < 0)
    {
        /* This value is over-range */
        lcd_string("High", field + 1);
        return;
    }
    after = abs(after);
    lcd_icon(lcd_fields[field].minus_icons[0], false);
    for (i = 20 - cells - shift;  i < 20 - shift;  i++)
    {
        digit = bcd[i >> 1];
        if ((i & 1) == 0)
            digit >>= 4;
        digit &= 0x0F;
        if (digit  ||  i >= (19 - after))
        {
            if (flag == 2)
                lcd_icon(lcd_fields[field].minus_icons[pos], true);
            flag = 0;
        }
        lcd_field_cell((flag == 0)  ?  (digit + '0')  :  ' ', field, pos++);
    }
    set_decimal_point(field, after - shift);
}

/* Display an unsigned 64 bit integer, with leading zero suppression. "after" is the number of
   digits which are after the decimal point. The leading zero suppression allows for this. If
   after is negative, its modulus is the maximum number of digits after the decimal point, but
   the decimal places will automatically be dropped to fit a large number. If the number will not
   fit at all, "High" is displayed. */
void lcd_decu64(uint64_t value, int field, int after)
{
    field--;
    lcd_dec64_common(value, field, after, 1, false);
}

/* Display a signed 64 bit integer, with leading zero suppression. "after" is the number of
   digits which are after the decimal point. The leading zero suppression allows for this. If
   after is negative, its modulus is the maximum number of digits after the decimal point, but
   the decimal places will automatically be dropped to fit a large number. If the number will not
   fit at all, "High" is displayed. */
void lcd_dec64(int64_t value, int field, int after)
{
    uint64_t mod_value;
    int flag;
    int reserve_first_cell;

    field--;

    reserve_first_cell = false;
    if (value < 0)
    {
        mod_value = -value;
        if (lcd_fields[field].minus_icons[0] == 0)
        {
            /* Make space for the minus, as we have no special icon for when it
               is in the leading position of the field. */
            lcd_field_cell(' ', field, 0);
            reserve_first_cell = true;
        }
        flag = 2;
    }
    else
    {
        mod_value = value;
        flag = 1;
    }
    lcd_dec64_common(mod_value, field, after, flag, reserve_first_cell);
}
