/*******************************************************************************
 *  lcd-7segment-segments.h -
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

#if !defined(_LCD_7SEGMENT_SEGMENTS_H_)
#define _LCD_7SEGMENT_SEGMENTS_H_

/* SEG_a to SEG_h should have been defined to match the segments of the LCD in use before
   this file is included. Otherwise.... */

#if !defined(SEG_a)  ||  !defined(SEG_b)  ||  !defined(SEG_c)  ||  !defined(SEG_d)  ||  !defined(SEG_e)  ||  !defined(SEG_f)  ||  !defined(SEG_g)  ||  !defined(SEG_h)
#error The LCD segments have not been defined
#endif

#define CHAR_7SEGMENT_SPACE             0
#define CHAR_7SEGMENT_ALL               (uint8_t) (SEG_a|SEG_b|SEG_c|SEG_d|SEG_e|SEG_f|SEG_g)

#define CHAR_7SEGMENT_0                 (uint8_t) (SEG_a|SEG_b|SEG_c|SEG_d|SEG_e|SEG_f)
#define CHAR_7SEGMENT_1                 (uint8_t) (SEG_b|SEG_c)
#define CHAR_7SEGMENT_2                 (uint8_t) (SEG_a|SEG_b|SEG_d|SEG_e|SEG_g)
#define CHAR_7SEGMENT_3                 (uint8_t) (SEG_a|SEG_b|SEG_c|SEG_d|SEG_g)
#define CHAR_7SEGMENT_4                 (uint8_t) (SEG_b|SEG_c|SEG_f|SEG_g)
#define CHAR_7SEGMENT_5                 (uint8_t) (SEG_a|SEG_c|SEG_d|SEG_f|SEG_g)
#define CHAR_7SEGMENT_6                 (uint8_t) (SEG_a|SEG_c|SEG_d|SEG_e|SEG_f|SEG_g)
#define CHAR_7SEGMENT_7                 (uint8_t) (SEG_a|SEG_b|SEG_c)
#define CHAR_7SEGMENT_8                 (uint8_t) (SEG_a|SEG_b|SEG_c|SEG_d|SEG_e|SEG_f|SEG_g)
#define CHAR_7SEGMENT_9                 (uint8_t) (SEG_a|SEG_b|SEG_c|SEG_d|SEG_f|SEG_g)

#define CHAR_7SEGMENT_DOUBLEQUOTE       (uint8_t) (SEG_a|SEG_f)
#define CHAR_7SEGMENT_DOLLAR            (uint8_t) (SEG_a|SEG_c|SEG_d|SEG_f|SEG_g)
#define CHAR_7SEGMENT_QUOTE             (uint8_t) (SEG_a)
#define CHAR_7SEGMENT_MINUS             (uint8_t) (SEG_g)
#define CHAR_7SEGMENT_SLASH             (uint8_t) (SEG_b|SEG_c)
#define CHAR_7SEGMENT_EQUALS            (uint8_t) (SEG_d|SEG_g)
#define CHAR_7SEGMENT_QUESTION          (uint8_t) (SEG_a|SEG_b|SEG_e|SEG_g)
#define CHAR_7SEGMENT_LEFTBRACKET       (uint8_t) (SEG_a|SEG_d|SEG_e|SEG_f)
#define CHAR_7SEGMENT_BACKSLASH         (uint8_t) (SEG_e|SEG_f)
#define CHAR_7SEGMENT_RIGHTBRACKET      (uint8_t) (SEG_a|SEG_b|SEG_c|SEG_d)
#define CHAR_7SEGMENT_CARET             (uint8_t) (SEG_a)
#define CHAR_7SEGMENT_UNDERSCORE        (uint8_t) (SEG_d)
#define CHAR_7SEGMENT_BACKQUOTE         (uint8_t) (SEG_b)
#define CHAR_7SEGMENT_VERTICALBAR       (uint8_t) (SEG_e|SEG_f)

/* Some of these look good, and some do not. Some also look the same as a digit or each other. Beware! */
#define CHAR_7SEGMENT_A                 (uint8_t) (SEG_a|SEG_b|SEG_c|SEG_e|SEG_f|SEG_g)
#define CHAR_7SEGMENT_a                 (uint8_t) (SEG_a|SEG_b|SEG_c|SEG_d|SEG_e|SEG_g)
#define CHAR_7SEGMENT_B                 (uint8_t) (SEG_c|SEG_d|SEG_e|SEG_f|SEG_g)
#define CHAR_7SEGMENT_b                 (uint8_t) (SEG_c|SEG_d|SEG_e|SEG_f|SEG_g)
#define CHAR_7SEGMENT_C                 (uint8_t) (SEG_a|SEG_d|SEG_e|SEG_f)
#define CHAR_7SEGMENT_c                 (uint8_t) (SEG_d|SEG_e|SEG_g)
#define CHAR_7SEGMENT_D                 (uint8_t) (SEG_b|SEG_c|SEG_d|SEG_e|SEG_g)
#define CHAR_7SEGMENT_d                 (uint8_t) (SEG_b|SEG_c|SEG_d|SEG_e|SEG_g)
#define CHAR_7SEGMENT_E                 (uint8_t) (SEG_a|SEG_d|SEG_e|SEG_f|SEG_g)
#define CHAR_7SEGMENT_e                 (uint8_t) (SEG_a|SEG_b|SEG_d|SEG_e|SEG_f|SEG_g)
#define CHAR_7SEGMENT_F                 (uint8_t) (SEG_a|SEG_e|SEG_f|SEG_g)
#define CHAR_7SEGMENT_f                 (uint8_t) (SEG_a|SEG_e|SEG_f|SEG_g)
#define CHAR_7SEGMENT_G                 (uint8_t) (SEG_a|SEG_c|SEG_d|SEG_e|SEG_f)
#define CHAR_7SEGMENT_g                 (uint8_t) (SEG_a|SEG_b|SEG_c|SEG_d|SEG_f|SEG_g)
#define CHAR_7SEGMENT_H                 (uint8_t) (SEG_b|SEG_c|SEG_e|SEG_f|SEG_g)
#define CHAR_7SEGMENT_h                 (uint8_t) (SEG_c|SEG_e|SEG_f|SEG_g)
#define CHAR_7SEGMENT_I                 (uint8_t) (SEG_b|SEG_c)
#define CHAR_7SEGMENT_i                 (uint8_t) (SEG_c)
#define CHAR_7SEGMENT_J                 (uint8_t) (SEG_a|SEG_b|SEG_c|SEG_d|SEG_e)
#define CHAR_7SEGMENT_j                 (uint8_t) (SEG_a|SEG_b|SEG_c|SEG_d|SEG_e)
#define CHAR_7SEGMENT_K                 (uint8_t) (SEG_c|SEG_d|SEG_e|SEG_f|SEG_g)
#define CHAR_7SEGMENT_k                 (uint8_t) (SEG_c|SEG_d|SEG_e|SEG_f|SEG_g)
#define CHAR_7SEGMENT_L                 (uint8_t) (SEG_d|SEG_e|SEG_f)
#define CHAR_7SEGMENT_l                 (uint8_t) (SEG_b|SEG_c)
#define CHAR_7SEGMENT_M                 (uint8_t) (SEG_b|SEG_c|SEG_d|SEG_e|SEG_f)
#define CHAR_7SEGMENT_m                 (uint8_t) (SEG_c|SEG_e|SEG_g)
#define CHAR_7SEGMENT_N                 (uint8_t) (SEG_b|SEG_c|SEG_d|SEG_e|SEG_f)
#define CHAR_7SEGMENT_n                 (uint8_t) (SEG_c|SEG_e|SEG_g)
#define CHAR_7SEGMENT_O                 (uint8_t) (SEG_a|SEG_b|SEG_c|SEG_d|SEG_e|SEG_f)
#define CHAR_7SEGMENT_o                 (uint8_t) (SEG_c|SEG_d|SEG_e|SEG_g)
#define CHAR_7SEGMENT_P                 (uint8_t) (SEG_a|SEG_b|SEG_e|SEG_f|SEG_g)
#define CHAR_7SEGMENT_p                 (uint8_t) (SEG_a|SEG_b|SEG_e|SEG_f|SEG_g)
#define CHAR_7SEGMENT_Q                 (uint8_t) (SEG_a|SEG_b|SEG_c|SEG_f|SEG_g)
#define CHAR_7SEGMENT_q                 (uint8_t) (SEG_a|SEG_b|SEG_c|SEG_f|SEG_g)
#define CHAR_7SEGMENT_R                 (uint8_t) (SEG_a|SEG_b|SEG_c|SEG_e|SEG_f|SEG_g)
#define CHAR_7SEGMENT_r                 (uint8_t) (SEG_e|SEG_g)
#define CHAR_7SEGMENT_S                 (uint8_t) (SEG_a|SEG_c|SEG_d|SEG_f|SEG_g)
#define CHAR_7SEGMENT_s                 (uint8_t) (SEG_a|SEG_c|SEG_d|SEG_f|SEG_g)
#define CHAR_7SEGMENT_T                 (uint8_t) (SEG_d|SEG_e|SEG_f|SEG_g)
#define CHAR_7SEGMENT_t                 (uint8_t) (SEG_d|SEG_e|SEG_f|SEG_g)
#define CHAR_7SEGMENT_u                 (uint8_t) (SEG_c|SEG_d|SEG_e)
#define CHAR_7SEGMENT_U                 (uint8_t) (SEG_b|SEG_c|SEG_d|SEG_e|SEG_f)
#define CHAR_7SEGMENT_v                 (uint8_t) (SEG_c|SEG_d|SEG_e)
#define CHAR_7SEGMENT_V                 (uint8_t) (SEG_b|SEG_c|SEG_d|SEG_e|SEG_f)
#define CHAR_7SEGMENT_w                 (uint8_t) (SEG_c|SEG_d|SEG_e)
#define CHAR_7SEGMENT_W                 (uint8_t) (SEG_b|SEG_c|SEG_d|SEG_e|SEG_f)
#define CHAR_7SEGMENT_x                 (uint8_t) (SEG_b|SEG_c|SEG_e|SEG_f|SEG_g)
#define CHAR_7SEGMENT_X                 (uint8_t) (SEG_b|SEG_c|SEG_e|SEG_f|SEG_g)
#define CHAR_7SEGMENT_y                 (uint8_t) (SEG_b|SEG_c|SEG_d|SEG_f|SEG_g)
#define CHAR_7SEGMENT_Y                 (uint8_t) (SEG_b|SEG_c|SEG_d|SEG_f|SEG_g)
#define CHAR_7SEGMENT_z                 (uint8_t) (SEG_a|SEG_b|SEG_d|SEG_e|SEG_g)
#define CHAR_7SEGMENT_Z                 (uint8_t) (SEG_a|SEG_b|SEG_d|SEG_e|SEG_g)
#define CHAR_7SEGMENT_MINUS             (uint8_t) (SEG_g)
#define CHAR_7SEGMENT_EQUALS            (uint8_t) (SEG_d|SEG_g)

#endif
