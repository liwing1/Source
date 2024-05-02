/*******************************************************************************
 *  lierda_lcd.h - LCD definitions for the Lierda LCD panel with 2 rows of 8
 *                 7-segment digits, and various icons for metrology
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

/* LCD definitions for the panel used by Lierda which has
   2 lines of 8 digits
   various indicators for 3-phase e-meters
   160 total segments
   
   These definitions assume a particular mapping of the COM and SEG
   lines of the LCD to pins on the MCU.
 */

/* Each display item is defined by a 16 bit number, where
        Upper 8 bits is the LCD controller memory location
        Lower 8 bits is a mask for the LCD controller location
 */

/* Segment mapping for the digits. All digits are consistent on this display */
#define LCD_7SEGMENT_SEG_a_BIT                  0
#define LCD_7SEGMENT_SEG_b_BIT                  1
#define LCD_7SEGMENT_SEG_c_BIT                  2
#define LCD_7SEGMENT_SEG_d_BIT                  7
#define LCD_7SEGMENT_SEG_e_BIT                  6
#define LCD_7SEGMENT_SEG_f_BIT                  4
#define LCD_7SEGMENT_SEG_g_BIT                  5
#define LCD_7SEGMENT_SEG_h_BIT                  3

#define LCD_7SEGMENT_SEG_a                      (1 << LCD_7SEGMENT_SEG_a_BIT)
#define LCD_7SEGMENT_SEG_b                      (1 << LCD_7SEGMENT_SEG_b_BIT)
#define LCD_7SEGMENT_SEG_c                      (1 << LCD_7SEGMENT_SEG_c_BIT)
#define LCD_7SEGMENT_SEG_d                      (1 << LCD_7SEGMENT_SEG_d_BIT)
#define LCD_7SEGMENT_SEG_e                      (1 << LCD_7SEGMENT_SEG_e_BIT)
#define LCD_7SEGMENT_SEG_f                      (1 << LCD_7SEGMENT_SEG_f_BIT)
#define LCD_7SEGMENT_SEG_g                      (1 << LCD_7SEGMENT_SEG_g_BIT)
#define LCD_7SEGMENT_SEG_h                      (1 << LCD_7SEGMENT_SEG_h_BIT)

#define LCD_7SEGMENT_MASK           LCD_7SEGMENT_SEG_h

#define icon_loc(cell,bit)          ((bit << 5) | cell)

/* bottom row 4 quadrant indicator, quadrant 1 */
#define LCD_BOTTOM_QUADRANT_1       icon_loc(9, LCD_7SEGMENT_SEG_a_BIT)
/* bottom row 4 quadrant indicator, quadrant 2 */
#define LCD_BOTTOM_QUADRANT_2       icon_loc(9, LCD_7SEGMENT_SEG_b_BIT)
/* bottom row 4 quadrant indicator, quadrant 3 */
#define LCD_BOTTOM_QUADRANT_3       icon_loc(9, LCD_7SEGMENT_SEG_c_BIT)
/* bottom row low battery */
#define LCD_BOTTOM_LOW_BATTERY      icon_loc(9, LCD_7SEGMENT_SEG_d_BIT)
/* bottom row phase C */
#define LCD_BOTTOM_PHASE_C_ICON     icon_loc(9, LCD_7SEGMENT_SEG_e_BIT)
/* bottom row phase A */
#define LCD_BOTTOM_PHASE_A_ICON     icon_loc(9, LCD_7SEGMENT_SEG_f_BIT)
/* bottom row phase B */
#define LCD_BOTTOM_PHASE_B_ICON     icon_loc(9, LCD_7SEGMENT_SEG_g_BIT)
/* bottom row 4 quadrant indicator, quadrant 4 */
#define LCD_BOTTOM_QUADRANT_4       icon_loc(9, LCD_7SEGMENT_SEG_h_BIT)

/* bottom row jim */
#define LCD_1_5                     icon_loc(10, LCD_7SEGMENT_SEG_a_BIT)
/* bottom row san */
#define LCD_1_6                     icon_loc(10, LCD_7SEGMENT_SEG_b_BIT)
/* bottom row ping */
#define LCD_1_7                     icon_loc(10, LCD_7SEGMENT_SEG_c_BIT)
/* Hz */
#define LCD_LOWER_HERTZ             icon_loc(10, LCD_7SEGMENT_SEG_d_BIT)
/* bottom row bin ching */
#define LCD_1_3                     icon_loc(10, LCD_7SEGMENT_SEG_e_BIT)
/* bottom row ??? so */
#define LCD_1_1                     icon_loc(10, LCD_7SEGMENT_SEG_f_BIT)
/* bottom row ??? wai */
#define LCD_1_2                     icon_loc(10, LCD_7SEGMENT_SEG_g_BIT)
/* bottom row yung */
#define LCD_1_8                     icon_loc(10, LCD_7SEGMENT_SEG_h_BIT)

/* left side "current" */
#define LCD_LOWER_CURRENT           icon_loc(11, LCD_7SEGMENT_SEG_a_BIT)
/* top row "time" */
#define LCD_TOP_TIME                icon_loc(11, LCD_7SEGMENT_SEG_b_BIT)
/* top row sui leung */
#define LCD_20_7                    icon_loc(11, LCD_7SEGMENT_SEG_c_BIT)
/* top row yung */
#define LCD_20_4                    icon_loc(11, LCD_7SEGMENT_SEG_d_BIT)
/* top row ping */
#define LCD_20_3                    icon_loc(11, LCD_7SEGMENT_SEG_e_BIT)
/* top row jim */
#define LCD_20_1                    icon_loc(11, LCD_7SEGMENT_SEG_f_BIT)
/* top row san */
#define LCD_20_2                    icon_loc(11, LCD_7SEGMENT_SEG_g_BIT)
/* top row din leung */
#define LCD_20_8                    icon_loc(11, LCD_7SEGMENT_SEG_h_BIT)

/* top row "voltage" */
#define LCD_UPPER_VOLTAGE           icon_loc(20, LCD_7SEGMENT_SEG_a_BIT)
/* top row ???? */
#define LCD_11_6                    icon_loc(20, LCD_7SEGMENT_SEG_b_BIT)
/* top row "last month" */
#define LCD_TOP_LAST_MONTH          icon_loc(20, LCD_7SEGMENT_SEG_c_BIT)
/* top row "reverse" */
#define LCD_TOP_REVERSE             icon_loc(20, LCD_7SEGMENT_SEG_d_BIT)
/* top row "forwards" */
#define LCD_TOP_FORWARDS            icon_loc(20, LCD_7SEGMENT_SEG_e_BIT)
/* lower row "reactive power" */
#define LCD_LOWER_REACTIVE_POWER    icon_loc(20, LCD_7SEGMENT_SEG_f_BIT)
/* top row "active power" */
#define LCD_UPPER_ACTIVE_POWER      icon_loc(20, LCD_7SEGMENT_SEG_g_BIT)
/* top row "previous" */
#define LCD_TOP_11_8                icon_loc(20, LCD_7SEGMENT_SEG_h_BIT)

/* upper row h (add hours to kW) */
#define LCD_UPPER_HOUR              icon_loc(19, LCD_7SEGMENT_SEG_h_BIT)

/* upper row decimal point for digit 2 */
#define LCD_UPPER_DP_2              icon_loc(18, LCD_7SEGMENT_SEG_h_BIT)

/* upper row kW */
#define LCD_UPPER_KW                icon_loc(17, LCD_7SEGMENT_SEG_h_BIT)

/* upper row decimal point for digit 4 */
#define LCD_UPPER_DP_4              icon_loc(16, LCD_7SEGMENT_SEG_h_BIT)

/* upper row decimal point for digit 5 */
#define LCD_UPPER_DP_5              icon_loc(15, LCD_7SEGMENT_SEG_h_BIT)

/* upper row decimal point for digit 6 */
#define LCD_UPPER_DP_6              icon_loc(14, LCD_7SEGMENT_SEG_h_BIT)

/* upper row decimal point for digit 7 */
#define LCD_UPPER_DP_7              icon_loc(13, LCD_7SEGMENT_SEG_h_BIT)

/* upper row V (volts) */
#define LCD_UPPER_VOLTS             icon_loc(12, LCD_7SEGMENT_SEG_h_BIT)

/* lower row h (add hours to kvar) */
#define LCD_LOWER_HOUR              icon_loc(8, LCD_7SEGMENT_SEG_h_BIT)

/* lower row decimal point for digit 2 */
#define LCD_LOWER_DP_2              icon_loc(7, LCD_7SEGMENT_SEG_h_BIT)

/* lower row kvar */
#define LCD_LOWER_KVAR              icon_loc(6, LCD_7SEGMENT_SEG_h_BIT)

/* lower row decimal point for digit 4 */
#define LCD_LOWER_DP_4              icon_loc(5, LCD_7SEGMENT_SEG_h_BIT)

/* lower row decimal point for digit 5 */
#define LCD_LOWER_DP_5              icon_loc(4, LCD_7SEGMENT_SEG_h_BIT)

/* lower row decimal point for digit 6 */
#define LCD_LOWER_DP_6              icon_loc(3, LCD_7SEGMENT_SEG_h_BIT)

/* lower row decimal point for digit 7 */
#define LCD_LOWER_DP_7              icon_loc(2, LCD_7SEGMENT_SEG_h_BIT)

/* lower row A (amps) */
#define LCD_LOWER_AMPS              icon_loc(1, LCD_7SEGMENT_SEG_h_BIT)

#define UPPER_NUMBER_FIRST_DIGIT    19
#define UPPER_NUMBER_DIGITS         8
#define UPPER_NUMBER_STEP           -1

#define LOWER_NUMBER_FIRST_DIGIT    8
#define LOWER_NUMBER_DIGITS         8
#define LOWER_NUMBER_STEP           -1

#define LCD_UPPER_MINUS_2           icon_loc(19, LCD_7SEGMENT_SEG_g_BIT)
#define LCD_UPPER_MINUS_3           icon_loc(18, LCD_7SEGMENT_SEG_g_BIT)
#define LCD_UPPER_MINUS_4           icon_loc(17, LCD_7SEGMENT_SEG_g_BIT)
#define LCD_UPPER_MINUS_5           icon_loc(16, LCD_7SEGMENT_SEG_g_BIT)
#define LCD_UPPER_MINUS_6           icon_loc(15, LCD_7SEGMENT_SEG_g_BIT)
#define LCD_UPPER_MINUS_7           icon_loc(14, LCD_7SEGMENT_SEG_g_BIT)
#define LCD_UPPER_MINUS_8           icon_loc(13, LCD_7SEGMENT_SEG_g_BIT)

#define LCD_LOWER_MINUS_2           icon_loc(8, LCD_7SEGMENT_SEG_g_BIT)
#define LCD_LOWER_MINUS_3           icon_loc(7, LCD_7SEGMENT_SEG_g_BIT)
#define LCD_LOWER_MINUS_4           icon_loc(6, LCD_7SEGMENT_SEG_g_BIT)
#define LCD_LOWER_MINUS_5           icon_loc(5, LCD_7SEGMENT_SEG_g_BIT)
#define LCD_LOWER_MINUS_6           icon_loc(4, LCD_7SEGMENT_SEG_g_BIT)
#define LCD_LOWER_MINUS_7           icon_loc(3, LCD_7SEGMENT_SEG_g_BIT)
#define LCD_LOWER_MINUS_8           icon_loc(2, LCD_7SEGMENT_SEG_g_BIT)

