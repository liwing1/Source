/*******************************************************************************
 *  lcd-starburst-segments.h -
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

#if !defined(_LCD_STARBURST_SEGMENTS_H_)
#define _LCD_STARBURST_SEGMENTS_H_

/* SEG_a to SEG_h and SEG_1 to SEG_12 should have been defined to match the segments of the LCD in use before
   this file is included. Otherwise.... */

#if !defined(SEG_a)  ||  !defined(SEG_b)  ||  !defined(SEG_c)  ||  !defined(SEG_d)  ||  !defined(SEG_e)  ||  !defined(SEG_f)  ||  !defined(SEG_g)  ||  !defined(SEG_h)
#error The LCD segments have not been defined
#endif

#if !defined(SEG_1)  ||  !defined(SEG_3)  ||  !defined(SEG_5)  ||  !defined(SEG_6)  ||  !defined(SEG_7)  ||  !defined(SEG_9)  ||  !defined(SEG_11)  ||  !defined(SEG_12)
#error The LCD segments have not been defined
#endif

#define CHAR_STARBURST_SPACE            0
#define CHAR_STARBURST_ALL              (uint16_t) (SEG_a|SEG_b|SEG_c|SEG_d|SEG_e|SEG_f|SEG_1|SEG_3|SEG_5|SEG_6|SEG_7|SEG_9|SEG_11|SEG_12)

#define CHAR_STARBURST_0                (uint16_t) (SEG_a|SEG_b|SEG_c|SEG_d|SEG_e|SEG_f)
#define CHAR_STARBURST_1                (uint16_t) (SEG_b|SEG_c)
#define CHAR_STARBURST_2                (uint16_t) (SEG_a|SEG_d|SEG_e|SEG_1|SEG_9)
#define CHAR_STARBURST_3                (uint16_t) (SEG_a|SEG_b|SEG_c|SEG_d|SEG_3)
#define CHAR_STARBURST_4                (uint16_t) (SEG_b|SEG_c|SEG_f|SEG_3|SEG_9)
#define CHAR_STARBURST_5                (uint16_t) (SEG_a|SEG_c|SEG_d|SEG_f|SEG_3|SEG_9)
#define CHAR_STARBURST_6                (uint16_t) (SEG_a|SEG_c|SEG_d|SEG_e|SEG_f|SEG_3|SEG_9)
#define CHAR_STARBURST_7                (uint16_t) (SEG_a|SEG_b|SEG_c)
#define CHAR_STARBURST_8                (uint16_t) (SEG_a|SEG_b|SEG_c|SEG_d|SEG_e|SEG_f|SEG_3|SEG_9)
#define CHAR_STARBURST_9                (uint16_t) (SEG_a|SEG_b|SEG_c|SEG_d|SEG_f|SEG_3|SEG_9)

#define CHAR_STARBURST_DOUBLEQUOTE      (uint16_t) (SEG_f|SEG_12)
#define CHAR_STARBURST_DOLLAR           (uint16_t) (SEG_a|SEG_c|SEG_d|SEG_f|SEG_3|SEG_6|SEG_9|SEG_12)
#define CHAR_STARBURST_QUOTE            (uint16_t) (SEG_f)
#define CHAR_STARBURST_ASTERISK         (uint16_t) (SEG_1|SEG_3|SEG_5|SEG_6|SEG_7|SEG_9|SEG_11|SEG_12)
#define CHAR_STARBURST_PLUS             (uint16_t) (SEG_3|SEG_6|SEG_9|SEG_12)
#define CHAR_STARBURST_MINUS            (uint16_t) (SEG_3|SEG_9)
#define CHAR_STARBURST_SLASH            (uint16_t) (SEG_1|SEG_7)
#define CHAR_STARBURST_LT               (uint16_t) (SEG_1|SEG_5)
#define CHAR_STARBURST_EQUALS           (uint16_t) (SEG_d|SEG_3|SEG_9)
#define CHAR_STARBURST_GT               (uint16_t) (SEG_7|SEG_11)
#define CHAR_STARBURST_QUESTION         (uint16_t) (SEG_a|SEG_f|SEG_1|SEG_6)
#define CHAR_STARBURST_LEFTBRACKET      (uint16_t) (SEG_a|SEG_d|SEG_e|SEG_f)
#define CHAR_STARBURST_BACKSLASH        (uint16_t) (SEG_5|SEG_11)
#define CHAR_STARBURST_RIGHTBRACKET     (uint16_t) (SEG_a|SEG_b|SEG_c|SEG_d)
#define CHAR_STARBURST_CARET            (uint16_t) (SEG_5|SEG_7)
#define CHAR_STARBURST_UNDERSCORE       (uint16_t) (SEG_d)
#define CHAR_STARBURST_BACKQUOTE        (uint16_t) (SEG_b)
#define CHAR_STARBURST_VERTICALBAR      (uint16_t) (SEG_6|SEG_12)
#define CHAR_STARBURST_A                (uint16_t) (SEG_a|SEG_b|SEG_c|SEG_e|SEG_f|SEG_3|SEG_9)
#define CHAR_STARBURST_a                (uint16_t) (SEG_a|SEG_b|SEG_c|SEG_d|SEG_e|SEG_3|SEG_9)
#define CHAR_STARBURST_B                (uint16_t) (SEG_a|SEG_d|SEG_e|SEG_f|SEG_9|SEG_1|SEG_5)
#define CHAR_STARBURST_b                (uint16_t) (SEG_c|SEG_d|SEG_e|SEG_f|SEG_3|SEG_9)
#define CHAR_STARBURST_C                (uint16_t) (SEG_a|SEG_d|SEG_e|SEG_f)
#define CHAR_STARBURST_c                (uint16_t) (SEG_d|SEG_e|SEG_3|SEG_9)
#define CHAR_STARBURST_D                (uint16_t) (SEG_a|SEG_b|SEG_c|SEG_d|SEG_6|SEG_12)
#define CHAR_STARBURST_d                (uint16_t) (SEG_b|SEG_c|SEG_d|SEG_e|SEG_3|SEG_9)
#define CHAR_STARBURST_E                (uint16_t) (SEG_a|SEG_d|SEG_e|SEG_f|SEG_9)
#define CHAR_STARBURST_e                (uint16_t) (SEG_a|SEG_b|SEG_d|SEG_e|SEG_f|SEG_3|SEG_9)
#define CHAR_STARBURST_F                (uint16_t) (SEG_a|SEG_e|SEG_f|SEG_9)
#define CHAR_STARBURST_f                (uint16_t) (SEG_a|SEG_e|SEG_f|SEG_9)               /* same as F */
#define CHAR_STARBURST_G                (uint16_t) (SEG_a|SEG_c|SEG_d|SEG_e|SEG_f|SEG_3)
#define CHAR_STARBURST_g                (uint16_t) (SEG_a|SEG_b|SEG_c|SEG_d|SEG_f|SEG_3|SEG_9)
#define CHAR_STARBURST_H                (uint16_t) (SEG_b|SEG_c|SEG_e|SEG_f|SEG_3|SEG_9)
#define CHAR_STARBURST_h                (uint16_t) (SEG_c|SEG_e|SEG_f|SEG_3|SEG_9)
#define CHAR_STARBURST_I                (uint16_t) (SEG_a|SEG_d|SEG_6|SEG_12)
#define CHAR_STARBURST_i                (uint16_t) (SEG_c)
#define CHAR_STARBURST_J                (uint16_t) (SEG_a|SEG_e|SEG_7|SEG_12)
#define CHAR_STARBURST_j                (uint16_t) (SEG_e|SEG_7|SEG_12)
#define CHAR_STARBURST_K                (uint16_t) (SEG_e|SEG_f|SEG_1|SEG_5|SEG_9)
#define CHAR_STARBURST_k                (uint16_t) (SEG_e|SEG_f|SEG_1|SEG_5|SEG_9)         /* same as K */
#define CHAR_STARBURST_L                (uint16_t) (SEG_d|SEG_e|SEG_f)
#define CHAR_STARBURST_l                (uint16_t) (SEG_b|SEG_c)
#define CHAR_STARBURST_M                (uint16_t) (SEG_b|SEG_c|SEG_e|SEG_f|SEG_1|SEG_11)
#define CHAR_STARBURST_m                (uint16_t) (SEG_c|SEG_e|SEG_3|SEG_6|SEG_9)
#define CHAR_STARBURST_N                (uint16_t) (SEG_b|SEG_c|SEG_e|SEG_f|SEG_5|SEG_11)
#define CHAR_STARBURST_n                (uint16_t) (SEG_c|SEG_e|SEG_3|SEG_9)
#define CHAR_STARBURST_O                (uint16_t) (SEG_a|SEG_b|SEG_c|SEG_d|SEG_e|SEG_f)
#define CHAR_STARBURST_o                (uint16_t) (SEG_c|SEG_d|SEG_e|SEG_3|SEG_9)
#define CHAR_STARBURST_P                (uint16_t) (SEG_a|SEG_b|SEG_e|SEG_f|SEG_3|SEG_9)
#define CHAR_STARBURST_p                (uint16_t) (SEG_a|SEG_e|SEG_f|SEG_1|SEG_9)
#define CHAR_STARBURST_Q                (uint16_t) (SEG_a|SEG_b|SEG_c|SEG_d|SEG_e|SEG_f|SEG_5)
#define CHAR_STARBURST_q                (uint16_t) (SEG_a|SEG_b|SEG_c|SEG_f|SEG_3|SEG_9)
#define CHAR_STARBURST_R                (uint16_t) (SEG_a|SEG_b|SEG_e|SEG_f|SEG_3|SEG_5|SEG_9)
#define CHAR_STARBURST_r                (uint16_t) (SEG_e|SEG_3|SEG_9)
#define CHAR_STARBURST_S                (uint16_t) (SEG_a|SEG_c|SEG_d|SEG_f|SEG_3|SEG_9)
#define CHAR_STARBURST_s                (uint16_t) (SEG_d|SEG_5|SEG_3)
#define CHAR_STARBURST_T                (uint16_t) (SEG_a|SEG_6|SEG_12)
#define CHAR_STARBURST_t                (uint16_t) (SEG_d|SEG_e|SEG_f|SEG_9)
#define CHAR_STARBURST_U                (uint16_t) (SEG_b|SEG_c|SEG_d|SEG_e|SEG_f)
#define CHAR_STARBURST_u                (uint16_t) (SEG_c|SEG_d|SEG_e)
#define CHAR_STARBURST_V                (uint16_t) (SEG_e|SEG_f|SEG_1|SEG_7)
#define CHAR_STARBURST_v                (uint16_t) (SEG_e|SEG_7)
#define CHAR_STARBURST_W                (uint16_t) (SEG_b|SEG_c|SEG_e|SEG_f|SEG_5|SEG_7)
#define CHAR_STARBURST_w                (uint16_t) (SEG_c|SEG_d|SEG_e|SEG_6)
#define CHAR_STARBURST_X                (uint16_t) (SEG_1|SEG_5|SEG_7|SEG_11)
#define CHAR_STARBURST_x                (uint16_t) (SEG_1|SEG_5|SEG_7|SEG_11)              /* same as X */
#define CHAR_STARBURST_Y                (uint16_t) (SEG_1|SEG_6|SEG_11)
#define CHAR_STARBURST_y                (uint16_t) (SEG_b|SEG_c|SEG_d|SEG_f|SEG_3|SEG_9)
#define CHAR_STARBURST_Z                (uint16_t) (SEG_a|SEG_d|SEG_1|SEG_7)
#define CHAR_STARBURST_z                (uint16_t) (SEG_d|SEG_9|SEG_7)

#endif
