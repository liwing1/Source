/*******************************************************************************
 *  emeter-template.h - MSP430F67641 3-phase distribution version
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


/*! This switch, in combination with the calibrator switch, enables calibration
    with the meter cooperating with an external reference, through a UART port. */
#define SERIAL_CALIBRATION_SUPPORT
#define SERIAL_CALIBRATION_PASSWORD_1               0x1234
#define SERIAL_CALIBRATION_PASSWORD_2               0x5678
#define SERIAL_CALIBRATION_PASSWORD_3               0x9ABC
#define SERIAL_CALIBRATION_PASSWORD_4               0xDEF0

/*! This switch enables the sending of the current readings, through a UART port,
    for use in cooperative calibration with other meters. */
#define SERIAL_CALIBRATION_REF_SUPPORT

/*! This selects real time clock support. This is implemented in software on
    the MSP430. */
#define RTC_SUPPORT

/*! Corrected RTC support enables temperature and basic error compensation for
    the MSP430's 32kHz crystal oscillator, so it makes for a higher quality RTC
    source, even using low accuracy (eg 20ppm) crystals. */
#define CORRECTED_RTC_SUPPORT

/* This switch enables support of an IR receiver and/or transmitter for
   programming and/or reading the meter. */
#define SERIAL_COMMS_SUPPORT
#define DLT645_SUPPORT

/*! This defines the speed of eUSCI 0 */
#define UART_0_SUPPORT
#define UART_0_BAUD_RATE                            9600
#define UART_0_DLT645_SUPPORT

/*! This defines the speed of eUSCI 1 */
#define UART_1_SUPPORT
#define UART_1_BAUD_RATE                            19200/3
#undef UART_1_DLT645_SUPPORT

// Define this macro so can send active power readings to the IHD430 via CC2530
#define IHD430_SUPPORT



/* Pull in definitions for the LCD */
#include "ti_lcd.h"

#define custom_set_consumption(x,y)                 /**/

#undef  LCD_DISPLAY_SUPPORT
#define  OLED_DISPLAY_SUPPORT

#define USE_STARBURST
#define USE_7SEGMENT

#define STARBURST_FIELD                             1
#define NUMERIC_FIELD                               2

#define ICON_PHASE_A                                LCD_PIE_CHART_SEG_1
#define ICON_PHASE_B                                LCD_PIE_CHART_SEG_4
#define ICON_PHASE_C                                LCD_PIE_CHART_SEG_6

#define INFO_POSITION                               STARBURST_FIRST_DIGIT
#define INFO_CHARS                                  STARBURST_DIGITS

#define TIME_DISPLAY_FIELD                          STARBURST_FIELD
#define TIME_TAG                                    "Time"
#define DATE_DISPLAY_FIELD                          STARBURST_FIELD
#define DATE_TAG                                    "Date"

#define HOUR_DISPLAY_POSITION                       1
#define ICON_TIME_COLON_1                           LCD_STARBURST_COLON1
#define MINUTE_DISPLAY_POSITION                     3
#define ICON_TIME_COLON_2                           LCD_STARBURST_COLON2
#define SECONDS_DISPLAY_POSITION                    5

#define YEAR_DISPLAY_POSITION                       1
#define ICON_DATE_COLON_1                           LCD_STARBURST_COLON1
#define MONTH_DISPLAY_POSITION                      3
#define ICON_DATE_COLON_2                           LCD_STARBURST_COLON2
#define DAY_DISPLAY_POSITION                        5

/* We have no icons for V, kW, etc, so we need to display a character to show the information type
   on display */
#define DISPLAY_TYPE_FIELD                          2
#define DISPLAY_TYPE_POSITION                       1

#define ACTIVE_POWER_DISPLAY_FIELD                  STARBURST_FIELD
#define ACTIVE_POWER_TAG                            "AcPo"

#define REACTIVE_POWER_DISPLAY_FIELD                STARBURST_FIELD
#define REACTIVE_POWER_TAG                          "rePo"

#define APPARENT_POWER_DISPLAY_FIELD                STARBURST_FIELD
#define APPARENT_POWER_TAG                          "ApPo"

#define ACTIVE_ENERGY_DISPLAY_FIELD                 STARBURST_FIELD
#define ACTIVE_ENERGY_DISPLAY_IN_KWH
#define ACTIVE_ENERGY_TAG                           "AcEn"

#define REACTIVE_ENERGY_DISPLAY_FIELD               STARBURST_FIELD
#define REACTIVE_ENERGY_DISPLAY_IN_KVARH
#define REACTIVE_ENERGY_TAG                         "reEn"

#define POWER_FACTOR_DISPLAY_FIELD                  STARBURST_FIELD
#define POWER_FACTOR_TAG                            "PF"

#define VOLTAGE_DISPLAY_FIELD                       STARBURST_FIELD
#define VOLTAGE_TAG                                 "Vrms"

#define CURRENT_DISPLAY_FIELD                       STARBURST_FIELD
#define CURRENT_TAG                                 "Irms"

#define FREQUENCY_DISPLAY_FIELD                     STARBURST_FIELD
#define FREQUENCY_TAG                               "Freq"

#define TEMPERATURE_DISPLAY_FIELD                   STARBURST_FIELD
#define TEMPERATURE_DISPLAY_RESOLUTION              1
#define TEMPERATURE_TAG                             "Temp"

/* Definition of the minus sign segments and decimal point segments for the display fields */
#define DISPLAY_MINUS_SIGNS \
 \
static const uint8_t field1_minus_icons[] = \
{ \
    LCD_STARBURST_MINUS_1, \
    LCD_STARBURST_MINUS_2, \
    LCD_STARBURST_MINUS_3, \
    LCD_STARBURST_MINUS_4, \
    LCD_STARBURST_MINUS_5, \
    LCD_STARBURST_MINUS_6 \
}; \
 \
static const uint8_t field2_minus_icons[] = \
{ \
    LCD_7SEGMENT_MINUS_1, \
    LCD_7SEGMENT_MINUS_2, \
    LCD_7SEGMENT_MINUS_3, \
    LCD_7SEGMENT_MINUS_4 \
};

#define DISPLAY_DECIMAL_POINTS \
static const uint8_t field1_dp_icons[] = \
{ \
    0, \
    LCD_STARBURST_DP_5, \
    LCD_STARBURST_DP_4, \
    LCD_STARBURST_DP_3, \
    LCD_STARBURST_DP_2, \
    LCD_STARBURST_DP_1 \
}; \
 \
static const uint8_t field2_dp_icons[] = \
{ \
    0, \
    LCD_7SEGMENT_DP_3, \
    LCD_7SEGMENT_DP_2, \
    LCD_7SEGMENT_DP_1 \
};

/* LCD display fields
   The main field is a 6 character starburst field.
   The supplementary field is a 4 character 7-segment field */
#define DISPLAY_FIELDS \
    {1, 2, 6, LCD_MODE_STARBURST, LCD_STARBURST_MASK, field1_minus_icons, field1_dp_icons}, \
    {17, -1, 4, LCD_MODE_7SEGMENT, LCD_7SEGMENT_MASK, field2_minus_icons, field2_dp_icons}

#define NUM_DISPLAY_FIELDS 2

/* LCD display sequence table */
#define DISPLAY_STEP_SEQUENCE \
    DISPLAY_ITEM_SELECT_PHASE_1, \
        DISPLAY_ITEM_TEMPERATURE, \
    DISPLAY_ITEM_SELECT_PHASE_1, \
        DISPLAY_ITEM_REACTIVE_POWER, \
    DISPLAY_ITEM_SELECT_PHASE_1, \
        DISPLAY_ITEM_APPARENT_POWER, \
    DISPLAY_ITEM_SELECT_PHASE_1, \
        DISPLAY_ITEM_POWER_FACTOR, \
    DISPLAY_ITEM_SELECT_PHASE_1, \
        DISPLAY_ITEM_VOLTAGE, \
    DISPLAY_ITEM_SELECT_PHASE_1, \
        DISPLAY_ITEM_CURRENT, \
    DISPLAY_ITEM_SELECT_PHASE_1, \
        DISPLAY_ITEM_MAINS_FREQUENCY, \
    DISPLAY_ITEM_SELECT_PHASE_1, \
        DISPLAY_ITEM_ACTIVE_ENERGY, \
    DISPLAY_ITEM_SELECT_PHASE_1, \
        DISPLAY_ITEM_REACTIVE_ENERGY, \
    DISPLAY_ITEM_SELECT_PHASE_2, \
        DISPLAY_ITEM_ACTIVE_POWER, \
    DISPLAY_ITEM_SELECT_PHASE_2, \
        DISPLAY_ITEM_REACTIVE_POWER, \
    DISPLAY_ITEM_SELECT_PHASE_2, \
        DISPLAY_ITEM_APPARENT_POWER, \
    DISPLAY_ITEM_SELECT_PHASE_2, \
        DISPLAY_ITEM_POWER_FACTOR, \
    DISPLAY_ITEM_SELECT_PHASE_2, \
        DISPLAY_ITEM_VOLTAGE, \
    DISPLAY_ITEM_SELECT_PHASE_2, \
        DISPLAY_ITEM_CURRENT, \
    DISPLAY_ITEM_SELECT_PHASE_2, \
        DISPLAY_ITEM_MAINS_FREQUENCY, \
    DISPLAY_ITEM_SELECT_PHASE_2, \
        DISPLAY_ITEM_ACTIVE_ENERGY, \
    DISPLAY_ITEM_SELECT_PHASE_2, \
        DISPLAY_ITEM_REACTIVE_ENERGY, \
    DISPLAY_ITEM_SELECT_PHASE_3, \
        DISPLAY_ITEM_ACTIVE_POWER, \
    DISPLAY_ITEM_SELECT_PHASE_3, \
        DISPLAY_ITEM_REACTIVE_POWER, \
    DISPLAY_ITEM_SELECT_PHASE_3, \
        DISPLAY_ITEM_APPARENT_POWER, \
    DISPLAY_ITEM_SELECT_PHASE_3, \
        DISPLAY_ITEM_POWER_FACTOR, \
    DISPLAY_ITEM_SELECT_PHASE_3, \
        DISPLAY_ITEM_VOLTAGE, \
    DISPLAY_ITEM_SELECT_PHASE_3, \
        DISPLAY_ITEM_CURRENT, \
    DISPLAY_ITEM_SELECT_PHASE_3, \
        DISPLAY_ITEM_MAINS_FREQUENCY, \
    DISPLAY_ITEM_SELECT_PHASE_3, \
        DISPLAY_ITEM_ACTIVE_ENERGY, \
    DISPLAY_ITEM_SELECT_PHASE_3, \
        DISPLAY_ITEM_REACTIVE_ENERGY, \
    DISPLAY_ITEM_SELECT_TOTAL, \
        DISPLAY_ITEM_ACTIVE_POWER, \
    DISPLAY_ITEM_SELECT_TOTAL, \
        DISPLAY_ITEM_REACTIVE_POWER, \
    DISPLAY_ITEM_SELECT_TOTAL, \
        DISPLAY_ITEM_TIME, \
    DISPLAY_ITEM_SELECT_TOTAL, \
       DISPLAY_ITEM_DATE, \
    DISPLAY_ITEM_SELECT_RESTART

#define custom_lcd_clear_periphery() \
    lcd_cell(0, BARGRAPH_DIGIT - 1); \
    lcd_cell(0, PIE_CHART_DIGIT - 1); \
    lcd_icon(LCD_7SEGMENT_PM, false); \
    lcd_icon(LCD_7SEGMENT_BELL, false); \
    lcd_icon(LCD_7SEGMENT_MINUS_1, false); \
    lcd_icon(LCD_7SEGMENT_COLON, false); \
    lcd_icon(LCD_7SEGMENT_DP_1, false); \
    lcd_icon(LCD_7SEGMENT_DP_2, false); \
    lcd_icon(LCD_7SEGMENT_DP_3, false); \
    lcd_icon(LCD_STARBURST_MINUS_1, false); \
    lcd_icon(LCD_STARBURST_COLON1, false); \
    lcd_icon(LCD_STARBURST_COLON2, false); \
    lcd_icon(LCD_STARBURST_DEGREE, false); \
    lcd_icon(LCD_STARBURST_DP_1, false); \
    lcd_icon(LCD_STARBURST_DP_2, false); \
    lcd_icon(LCD_STARBURST_DP_3, false); \
    lcd_icon(LCD_STARBURST_DP_4, false); \
    lcd_icon(LCD_STARBURST_DP_5, false); \
    lcd_icon(LCD_COMMA, false); \
    lcd_icon(LCD_RING, false); \
    lcd_icon(LCD_HEART, false); \
    lcd_icon(LCD_EXCLAMATION_MARK, false); \
    lcd_icon(LCD_MAST, false); \
    lcd_icon(LCD_MAST_RX, false); \
    lcd_icon(LCD_MAST_TX, false); \
    lcd_icon(LCD_TX_SMALL_RIGHT, false); \
    lcd_icon(LCD_TX_LARGE_RIGHT, false); \
    lcd_icon(LCD_TX_SMALL_LEFT, false); \
    lcd_icon(LCD_TX_LARGE_LEFT, false); \
    lcd_icon(LCD_BOXED_R, false); \
    lcd_icon(LCD_PIE_CHART_PIP_1, false); \
    lcd_icon(LCD_PIE_CHART_PIP_2, false); \
    lcd_icon(LCD_PIE_CHART_PIP_3, false); \
    lcd_icon(LCD_PIE_CHART_PIP_4, false);

#define custom_lcd_clear_line_1_tags()
#define custom_lcd_clear_line_2_tags()

#undef USE_WATCHDOG

#define custom_active_energy_pulse_start(ph) \
{ \
  switch(ph) \
  {           \
    \
    case FAKE_PHASE_TOTAL: \
         P3OUT &= ~BIT2; \
         break; \
    case 0: \
         P8OUT &= ~BIT6; \
         break; \
    case 1: \
         P8OUT &= ~BIT4; \
         break; \
    case 2: \
         P8OUT &= ~BIT5; \
         break; \
  } \
}

#define custom_active_energy_pulse_end(ph) \
{ \
  switch(ph) \
  {           \
    \
    case FAKE_PHASE_TOTAL: \
         P3OUT |= BIT2; \
         break; \
    case 0: \
         P8OUT |= BIT6; \
         break; \
    case 1: \
         P8OUT |= BIT4; \
         break; \
    case 2: \
         P8OUT |= BIT5; \
         break; \
  } \
}

#define custom_reactive_energy_pulse_start(ph) \
{ \
    if (ph == FAKE_PHASE_TOTAL) \
        P3OUT &= ~BIT3; \
}

#define custom_reactive_energy_pulse_end(ph) \
{ \
    if (ph == FAKE_PHASE_TOTAL) \
        P3OUT |= BIT3; \
}

/* This custom function must check the conditions, such as passswords and shorting link tests,
   appropriate to the meter's "enable calibration" condition. */
#define custom_is_calibration_enabled() true

/*
    P1.0 = Voltage sense 1
    P1.1 = Voltage sense 2
    P1.2 = UART 0 Rx
    P1.3 = UART 0 Tx
    P1.4 = UART 1 Rx IrDA
    P1.5 = UART 1 Tx IrDA
    P1.6 = Button
    P1.7 = Button/RF_VREG
 */
#define P1DIR_INIT                                  (BIT5 | BIT3)
#define P1SEL_INIT                                  (BIT5 | BIT4 | BIT3 | BIT2 | BIT1 | BIT0)
#define P1OUT_INIT                                  (0)
#define P1REN_INIT                                  (0)

/*
    P2.0 = SCL for EEPROM
    P2.1 = SDA for EEPROM
    P2.2 = A2SOMI
    P2.3 = A2SIMO
    P2.4 = A2CS
    P2.5 = A2CLK
    P2.6 = GPIO 
    P2.7 = GPIO
 */

#define P2DIR_INIT                                  (BIT3)
#define P2SEL_INIT                                  (BIT3 | BIT2| BIT1 | BIT0)
#define P2OUT_INIT                                  (0) //(BIT1 | BIT0)
#define P2REN_INIT                                  (0) //(BIT1 | BIT0)

/*
    P3.0 = GPIO
    P3.1 = GPIO
    P3.2 = ACT
    P3.3 = REACT
    P3.4 = LCD segment line 39
    P3.5 = LCD segment line 38
    P3.6 = LCD segment line 37
    P3.7 = LCD segment line 36
 */
#define P3DIR_INIT                                  (BIT3 | BIT2 | BIT1 | BIT0)
#define P3SEL_INIT                                  (0)
#define P3OUT_INIT                                  (BIT3 | BIT2| BIT0)
#define P3REN_INIT                                  (0)

/*
    P4.0 = LCD segment line 35
    P4.1 = LCD segment line 34
    P4.2 = LCD segment line 33
    P4.3 = LCD segment line 32
    P4.4 = LCD segment line 31
    P4.5 = LCD segment line 30
    P4.6 = LCD segment line 29
    P4.7 = LCD segment line 28
 */
#define P4DIR_INIT                                  (BIT0)
#define P4SEL_INIT                                  (0)
#define P4OUT_INIT                                  (0)
#define P4REN_INIT                                  (0)

/*
    P5.0 = LCD segment line 27
    P5.1 = LCD segment line 26
    P5.2 = LCD segment line 25
    P5.3 = LCD segment line 24
    P5.4 = LCD segment line 23
    P5.5 = LCD segment line 22
    P5.6 = LCD segment line 21
    P5.7 = LCD segment line 20
*/
#define P5DIR_INIT                                  (0)
#define P5SEL_INIT                                  (0)
#define P5OUT_INIT                                  (0)
#define P5REN_INIT                                  (0)

/*
    P6.0 = LCD segment line 19
    P6.1 = LCD segment line 18
    P6.2 = LCD segment line 17
    P6.3 = LCD segment line 16
    P6.4 = LCD segment line 15
    P6.5 = LCD segment line 14
    P6.6 = LCD segment line 13
    P6.7 = LCD segment line 12
*/
#define P6DIR_INIT                                  (0)
#define P6SEL_INIT                                  (0)
#define P6OUT_INIT                                  (0)
#define P6REN_INIT                                  (0)

/*
    P7.0 = LCD segment line 11
    P7.1 = LCD segment line 10
    P7.2 = LCD segment line 9
    P7.3 = LCD segment line 8
    P7.4 = LCD segment line 7
    P7.5 = LCD segment line 6
    P7.6 = LCD segment line 5
    P7.7 = LCD segment line 4
 */
#define P7DIR_INIT                                  (0)
#define P7SEL_INIT                                  (0)
#define P7OUT_INIT                                  (0)
#define P7REN_INIT                                  (0)

/*
    P8.0 = LCD segment line 3
    P8.1 = LCD segment line 2
    P8.2 = LCD segment line 1
    P8.3 = LCD segment line 0
    P8.4 = Phase 2
    P8.5 = Phase 3
    P8.6 = Phase 1
    P8.7 = GPIO
 */
#define P8DIR_INIT                                  (BIT6 | BIT5 | BIT4)
#define P8SEL_INIT                                  (0)
#define P8OUT_INIT                                  (BIT6 | BIT5 | BIT4)
#define P8REN_INIT                                  (0)

/*
    P9.0 =
    P9.1 = A5/voltage phase 1
    P9.2 = A4/voltage phase 2
    P9.3 = A3/voltage phase 3
    P9.4 = 
    P9.5 = 
    P9.6 = 
    P9.7 = 
 */
#define P9DIR_INIT                                  (0)
#define P9SEL_INIT                                  (BIT3 | BIT2 | BIT1)
#define P9OUT_INIT                                  (0)
#define P9REN_INIT                                  (0)

/*
    PJ.0 =  SMCLK
    PJ.1 =  MCLK
    PJ.2 =  ADC10CLK
    PJ.3 =  ACLK
 */
#define PJDIR_INIT                                  (0)
#define PJSEL_INIT                                  (BIT3 | BIT1 | BIT0)
#define PJOUT_INIT                                  (0)
#define PJOUT_REN                                   (0)

#ifdef OUTPUT_RTCCLK
/* Remap the RTCCLK bit */
#define CUSTOM_PORT_MAP \
    PMAPKEYID = PMAPKEY; \
    PMAPCTL = PMAPRECFG; \
    P3MAP2 = UINT8_C(PM_RTCCLK); \
    P3SEL |= BIT2; \
    PMAPKEYID = UINT8_C(0)
#endif

/*
    The LCD is a full 160 segment display, with 1 line of 6 starburst digits,
    1 line of 4 7-seg digits, plus various symbols.
 */
#define LCDCCTL0_INIT                               (LCDDIV_31 | LCDPRE_1 | LCD4MUX | LCDON)
#define LCDCCTL1_INIT                               (0)
#define LCDCBLKCTL_INIT                             (0)
#define LCDCMEMCTL_INIT                             (0)
#define LCDCVCTL_INIT                               (LCDCPEN | VLCD_2_60)
#define LCDCPCTL0_INIT                              (0xFFFF)  /* Segment lines 0 to 39 */
#define LCDCPCTL1_INIT                              (0xFFFF)
#define LCDCPCTL2_INIT                              (0xFF)
#define LCDCCPCTL_INIT                              (0)

/*  The below features are either not applicable to this EVM or have just been disabled.*/
      
    #undef OUTPUT_RTCCLK      
          
    /*! Related definitions to get special action routines to be called at various
        intervals. User supplied callback functions must be provided. Note these
        callback routines are called from within the per second timer interrupt
        service routine. Don't do anything too complex within them. If a long
        activity is required, set a flag within a simple routine, and do the main
        work in the main non-interrupt loop. */
    #undef PER_SECOND_ACTIVITY_SUPPORT
    #undef PER_MINUTE_ACTIVITY_SUPPORT
    #undef PER_HOUR_ACTIVITY_SUPPORT
    #undef PER_DAY_ACTIVITY_SUPPORT
    #undef PER_MONTH_ACTIVITY_SUPPORT
    #undef PER_YEAR_ACTIVITY_SUPPORT 
          
          /* This switch enables power down to battery backup status on loss of line
       power. */
    #undef POWER_DOWN_SUPPORT

    /* These switches select a method of detecting when power is restored, if
       power down mode is supported. */
    /* This method assumes the voltage waveform is being turned into simple
       digital pulses into an I/O pin. If this feature is used, POWER_UP_VOLTAGE_PULSE
       must define the way a voltage pulse is sensed. */
    #undef POWER_UP_BY_VOLTAGE_PULSES

    /* This method assumes the pre-regulator power supply voltage is being
       monitored by Comparator A. This method is suitable for meters which
       support a live/neutral only mode, for tamper resistance. */
    #undef POWER_UP_BY_SUPPLY_SENSING
          
          /* Many data logging requirements can be met by using only the MSP430's
       info memory. If an external serial EEPROM is needed for more
       complex requirements, this switch will enable an interface to
       I2C type serial EEPROMs. Basic routines to driver these EEPROMs are
       included in the toolkit. Routines to actually store and retrieve
       information are left to the meter designer. */
    #undef EXTERNAL_EEPROM_SUPPORT
          
          /*! This switch enables multi-rate tariff features */
    #undef MULTI_RATE_SUPPORT

    /*! This switch selects where a backup battery's condition is monitored. */
    #undef BATTERY_MONITOR_SUPPORT

