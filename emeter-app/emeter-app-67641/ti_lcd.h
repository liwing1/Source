/*******************************************************************************
 *  ti_lcd.h - LCD definitions for the TI LCD panel with a 6 digit starburst
 *             and a small 4 digit 7-segment field
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

/* LCD definitions for the TI panel which has
   1 line of 4 7-segment digits
   1 line of 6 starburst digits
   various indicators
   160 total segments
   
   These definitions assume a particular mapping of the COM and SEG
   lines of the LCD to pins on the MCU.
 */

/* Each display item is defined by a 16 bit number, where
        Upper 8 bits is the LCD controller memory location
        Lower 8 bits is a mask for the LCD controller location
 */

/* Segment mapping for the digits. All digits are consistent on this display */

/* 7-segment area */
#define LCD_7SEGMENT_a_BIT          3
#define LCD_7SEGMENT_b_BIT          2
#define LCD_7SEGMENT_c_BIT          1
#define LCD_7SEGMENT_d_BIT          0
#define LCD_7SEGMENT_e_BIT          7
#define LCD_7SEGMENT_f_BIT          6
#define LCD_7SEGMENT_g_BIT          5
#define LCD_7SEGMENT_h_BIT          4

#define LCD_7SEGMENT_SEG_a          (1 << LCD_7SEGMENT_a_BIT)
#define LCD_7SEGMENT_SEG_b          (1 << LCD_7SEGMENT_b_BIT)
#define LCD_7SEGMENT_SEG_c          (1 << LCD_7SEGMENT_c_BIT)
#define LCD_7SEGMENT_SEG_d          (1 << LCD_7SEGMENT_d_BIT)
#define LCD_7SEGMENT_SEG_e          (1 << LCD_7SEGMENT_e_BIT)
#define LCD_7SEGMENT_SEG_f          (1 << LCD_7SEGMENT_f_BIT)
#define LCD_7SEGMENT_SEG_g          (1 << LCD_7SEGMENT_g_BIT)
#define LCD_7SEGMENT_SEG_h          (1 << LCD_7SEGMENT_h_BIT)

/* Starburst area */
#define LCD_STARBURST_a_BIT         7
#define LCD_STARBURST_b_BIT         6
#define LCD_STARBURST_c_BIT         5
#define LCD_STARBURST_d_BIT         4
#define LCD_STARBURST_e_BIT         3
#define LCD_STARBURST_f_BIT         2
/* SEG_g isn't actually one segment */
#define LCD_STARBURST_h_BIT         8
#define LCD_STARBURST_i_BIT         10

#define LCD_STARBURST_12_BIT        14
#define LCD_STARBURST_1_BIT         13
#define LCD_STARBURST_3_BIT         0
#define LCD_STARBURST_5_BIT         9
#define LCD_STARBURST_6_BIT         12
#define LCD_STARBURST_7_BIT         11
#define LCD_STARBURST_9_BIT         1
#define LCD_STARBURST_11_BIT        15

#define LCD_STARBURST_SEG_a         (1 << LCD_STARBURST_a_BIT)
#define LCD_STARBURST_SEG_b         (1 << LCD_STARBURST_b_BIT)
#define LCD_STARBURST_SEG_c         (1 << LCD_STARBURST_c_BIT)
#define LCD_STARBURST_SEG_d         (1 << LCD_STARBURST_d_BIT)
#define LCD_STARBURST_SEG_e         (1 << LCD_STARBURST_e_BIT)
#define LCD_STARBURST_SEG_f         (1 << LCD_STARBURST_f_BIT)
#define LCD_STARBURST_SEG_g         ((1 << LCD_STARBURST_3_BIT) | (1 << LCD_STARBURST_9_BIT))
#define LCD_STARBURST_SEG_h         (1 << LCD_STARBURST_h_BIT)
#define LCD_STARBURST_SEG_i         (1 << LCD_STARBURST_i_BIT)

#define LCD_STARBURST_SEG_12        (1 << LCD_STARBURST_12_BIT)
#define LCD_STARBURST_SEG_1         (1 << LCD_STARBURST_1_BIT)
#define LCD_STARBURST_SEG_3         (1 << LCD_STARBURST_3_BIT)
#define LCD_STARBURST_SEG_5         (1 << LCD_STARBURST_5_BIT)
#define LCD_STARBURST_SEG_6         (1 << LCD_STARBURST_6_BIT)
#define LCD_STARBURST_SEG_7         (1 << LCD_STARBURST_7_BIT)
#define LCD_STARBURST_SEG_9         (1 << LCD_STARBURST_9_BIT)
#define LCD_STARBURST_SEG_11        (1 << LCD_STARBURST_11_BIT)

#define icon_loc(cell,bit)          ((bit << 5) | cell)

#define NUMBER_FIRST_DIGIT          14
#define NUMBER_DIGITS               4

#define STARBURST_FIRST_DIGIT       1
#define STARBURST_DIGITS            6

/* A 6-segment bar graph symbol, with end bracket and a pip that can look like a battery */
#define BARGRAPH_DIGIT              13
#define BARGRAPH_SEG_1              (1 << 5)
#define BARGRAPH_SEG_2              (1 << 1)
#define BARGRAPH_SEG_3              (1 << 6)
#define BARGRAPH_SEG_4              (1 << 2)
#define BARGRAPH_SEG_5              (1 << 7)
#define BARGRAPH_SEG_6              (1 << 3)
#define BARGRAPH_BRACKETS           (1 << 4)            /* Brackets around the bar-graph. Note that is displays both brackets */
#define BARGRAPH_BATTERY_PIP        (1 << 0)            /* This is a small pip, to the right of the closing bracket, to make a battery shape */

/* An 8 segment pie chart symbol */
#define PIE_CHART_DIGIT             19
#define PIE_CHART_SEG_1             (1 << 0)            /* 12 o'clock */
#define PIE_CHART_SEG_2             (1 << 4)
#define PIE_CHART_SEG_3             (1 << 1)            /* 3 o'clock */
#define PIE_CHART_SEG_4             (1 << 5)
#define PIE_CHART_SEG_5             (1 << 2)            /* 6 o'clock */
#define PIE_CHART_SEG_6             (1 << 6)
#define PIE_CHART_SEG_7             (1 << 3)            /* 9 o'clock */
#define PIE_CHART_SEG_8             (1 << 7)

#define LCD_PIE_CHART_SEG_1         icon_loc(19, 0)     /* 12 o'clock */
#define LCD_PIE_CHART_SEG_2         icon_loc(19, 4)
#define LCD_PIE_CHART_SEG_3         icon_loc(19, 1)     /* 3 o'clock */
#define LCD_PIE_CHART_SEG_4         icon_loc(19, 5)
#define LCD_PIE_CHART_SEG_5         icon_loc(19, 2)     /* 6 o'clock */
#define LCD_PIE_CHART_SEG_6         icon_loc(19, 6)
#define LCD_PIE_CHART_SEG_7         icon_loc(19, 3)     /* 9 o'clock */
#define LCD_PIE_CHART_SEG_8         icon_loc(19, 7)

#define LCD_COMMA                   icon_loc(20, 4)     /* Not sure what this is */
#define LCD_RING                    icon_loc(20, 5)     /* Not sure what this is */
#define LCD_HEART                   icon_loc(20, 6)
#define LCD_EXCLAMATION_MARK        icon_loc(20, 7)
#define LCD_MAST                    icon_loc(6, 2)      /* Transmitter mast symbol */
#define LCD_MAST_RX                 icon_loc(12, 0)     /* "RX" besides the transmitter mast symbol */
#define LCD_MAST_TX                 icon_loc(12, 2)     /* "TX" besides the transmitter mast symbol */
#define LCD_TX_SMALL_RIGHT          icon_loc(20, 0)
#define LCD_TX_LARGE_RIGHT          icon_loc(20, 1)
#define LCD_TX_SMALL_LEFT           icon_loc(20, 2)
#define LCD_TX_LARGE_LEFT           icon_loc(20, 3)
#define LCD_BOXED_R                 icon_loc(18, 1)     /* An R in a box */

#define LCD_PIE_CHART_PIP_1         icon_loc(18, 4)     /* A pip outside the pie chart at 12 o'clock */
#define LCD_PIE_CHART_PIP_2         icon_loc(18, 5)     /* A pip outside the pie chart at 3 o'clock */
#define LCD_PIE_CHART_PIP_3         icon_loc(18, 6)     /* A pip outside the pie chart at 6 o'clock */
#define LCD_PIE_CHART_PIP_4         icon_loc(18, 7)     /* A pip outside the pie chart at 9 o'clock */

#define LCD_7SEGMENT_MASK           0x10

#define LCD_7SEGMENT_COLON          icon_loc(17, 4)     /* 7-segment XX:XX */
#define LCD_7SEGMENT_PM             icon_loc(18, 2)     /* A PM indicator beside the 7-segment field */
#define LCD_7SEGMENT_BELL           icon_loc(18, 3)     /* A bell symbol beside the 7-segment field */

#define LCD_7SEGMENT_MINUS_1        icon_loc(18, 0)     /* 7-segment -XXXX */
#define LCD_7SEGMENT_MINUS_2        icon_loc(17, 5)     /* 7-segment  -XXX */
#define LCD_7SEGMENT_MINUS_3        icon_loc(16, 5)     /* 7-segment   -XX */
#define LCD_7SEGMENT_MINUS_4        icon_loc(15, 5)     /* 7-segment    -X */

#define LCD_7SEGMENT_DP_1           icon_loc(16, 4)     /* 7-segment X.XXX */
#define LCD_7SEGMENT_DP_2           icon_loc(15, 4)     /* 7-segment XX.XX */
#define LCD_7SEGMENT_DP_3           icon_loc(14, 4)     /* 7-segment XXX.X */

#define LCD_STARBURST_MASK          0x0500

#define LCD_STARBURST_DP_1          icon_loc(2, 0)      /* Starburst X.XXXXX */
#define LCD_STARBURST_DP_2          icon_loc(4, 0)      /* Starburst XX.XXXX */
#define LCD_STARBURST_DP_3          icon_loc(6, 0)      /* Starburst XXX.XXX */
#define LCD_STARBURST_DP_4          icon_loc(8, 0)      /* Starburst XXXX.XX */
#define LCD_STARBURST_DP_5          icon_loc(10, 0)     /* Starburst XXXXX.X */

#define LCD_STARBURST_MINUS_1       icon_loc(2, 2)      /* Starburst -XXXXXX */
#define LCD_STARBURST_MINUS_2       icon_loc(1, 0)      /* Starburst  -XXXXX */
#define LCD_STARBURST_MINUS_3       icon_loc(3, 0)      /* Starburst   -XXXX */
#define LCD_STARBURST_MINUS_4       icon_loc(5, 0)      /* Starburst    -XXX */
#define LCD_STARBURST_MINUS_5       icon_loc(7, 0)      /* Starburst     -XX */
#define LCD_STARBURST_MINUS_6       icon_loc(9, 0)      /* Starburst      -X */

#define LCD_STARBURST_COLON1        icon_loc(4, 2)      /* Starburst XX:XXXX */
#define LCD_STARBURST_COLON2        icon_loc(8, 2)      /* Starburst XXXX:XX */
#define LCD_STARBURST_DEGREE        icon_loc(10, 2)     /* Starburst XXXXXoX */
