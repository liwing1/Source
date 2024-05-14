/*******************************************************************************
 *  metrology-instantiate-adcs.h -
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

/* Create definitions to abstract the ADCs */

#if defined(__MSP430_HAS_SD24_B__)  &&  defined(__MSP430F6736__)  &&  !defined(__MSP430_HAS_SD24_B3__)
#define __MSP430_HAS_SD24_B3__
#endif
   
#if defined(__MSP430_HAS_SD24_B__)  &&  defined(__MSP430F67641__)  &&  !defined(__MSP430_HAS_SD24_B3__)
#define __MSP430_HAS_SD24_B3__
#endif

#if defined(__MSP430_HAS_SD24_B__)  &&  defined(__MSP430F6749__)  &&  !defined(__MSP430_HAS_SD24_B4__)
#define __MSP430_HAS_SD24_B4__
#endif

#if defined(__MSP430_HAS_SD24_B__)  &&  defined(__MSP430F6769__)  &&  !defined(__MSP430_HAS_SD24_B6__)
#define __MSP430_HAS_SD24_B6__
#endif

#if defined(__MSP430_HAS_SD24_B__)  &&  defined(__MSP430F6779__)  &&  !defined(__MSP430_HAS_SD24_B7__)
#define __MSP430_HAS_SD24_B7__
#endif

#include <sigma_delta_24bit_access.h>
#include <sigma_delta_16bit_access.h>
#include <sar_12bit_access.h>
#include <sar_10bit_access.h>

#if defined(__HAS_SD_ADC__)
    #if defined(TWENTYFOUR_BIT)
/*! The minimum values which the ADCs can produce. */
#define V_ADC_MIN                       -31000
#define I_ADC_MIN                       (-31000L*256L)
/*! The maximum values which the ADCs can produce. */
#define V_ADC_MAX                       31000
#define I_ADC_MAX                       (31000L*256L)
    #else
/*! The minimum values which the ADCs can produce. */
#define V_ADC_MIN                       -31000
#define I_ADC_MIN                       -31000
/*! The maximum values which the ADCs can produce. */
#define V_ADC_MAX                       31000
#define I_ADC_MAX                       31000
    #endif
#endif

/*! Set up meaningful names for the ADC registers, to match the way we use them. */

#if defined(__MSP430_HAS_SD24_B3__)  &&  NUM_PHASES > 1
/* 67641 3-phase */
#define ADC_VOLTAGE(x)          dma_adc_buffer[15 - x];
#define ADC_VOLTAGE_PENDING(x)    (SD24BTRGCTL & SD24TRGIFG)
#else
#define ADC_VOLTAGE_(x)         ADC16_ ## x()
#define ADC_VOLTAGE(x)          ADC_VOLTAGE_(x)
#define ADC_VOLTAGE_PENDING_(x) ADC16_ ## x ## _PENDING()
#define ADC_VOLTAGE_PENDING(x)  ADC_VOLTAGE_PENDING_(x)
#endif

#if defined(TWENTYFOUR_BIT)
#define ADC_CURRENT_(x)         ADC32_ ## x()
#define ADC_CURRENT(x)          ADC_CURRENT_(x)
#define ADC_CURRENT_PENDING_(x) ADC32_ ## x ## _PENDING()
#define ADC_CURRENT_PENDING(x)  ADC_CURRENT_PENDING_(x)
#else
#define ADC_CURRENT_(x)         ADC16_ ## x()
#define ADC_CURRENT(x)          ADC_CURRENT_(x)
#define ADC_CURRENT_PENDING_(x) ADC16_ ## x ## _PENDING()
#define ADC_CURRENT_PENDING(x)  ADC_CURRENT_PENDING_(x)
#endif

#define sd_xxxx_reg_(b,a)       b ## a
#define sd_xxxx_reg(b,a)        sd_xxxx_reg_(b,a)

#if defined(__MSP430_HAS_SD16_2__)

#define SD_CCTL_                SD16CCTL
#define SD_INCTL_               SD16INCTL
#define SD_PRE_                 SD16PRE

#define SD_CTL                  SD16CTL
#define SD_CCTL_TRIGGER         SD16CCTL1

#define SD_INCH_VOLTAGE         SD16INCH_0
#define SD_INCH_CURRENT         SD16INCH_0
#define SD_INCH_TEMPERATURE     SD16INCH_6

#define SD_SC                   SD16SC
#define SD_OSR_32               SD16OSR_32
#define SD_OSR_256              SD16OSR_256
#define SD_DF                   SD16DF
#define SD_GRP                  SD16GRP
#define SD_SNGL                 SD16SNGL
#define SD_REFON                SD16REFON
#define SD_LP                   SD16LP
#define SD_IE                   SD16IE
#define SD_SSEL_1               SD16SSEL_1
#define SD_DIV_2                SD16DIV_2
#define SD_DIV_3                SD16DIV_3
#define SD_GAIN_1               SD16GAIN_1
#define SD_GAIN_2               SD16GAIN_2
#define SD_GAIN_4               SD16GAIN_4
#define SD_GAIN_8               SD16GAIN_
#define SD_GAIN_16              SD16GAIN_16
#define SD_GAIN_TEMPERATURE     SD16GAIN_1

#elif defined(__MSP430_HAS_SD16_3__)

#define SD_CCTL_                SD16CCTL
#define SD_INCTL_               SD16INCTL
#define SD_PRE_                 SD16PRE

#define SD_CTL                  SD16CTL
#define SD_CCTL_TRIGGER         SD16CCTL2

#define SD_INCH_VOLTAGE         SD16INCH_0
#define SD_INCH_CURRENT         SD16INCH_0
#define SD_INCH_TEMPERATURE     SD16INCH_6
#define SD_SC                   SD16SC
#define SD_OSR_32               SD16OSR_32
#define SD_OSR_256              SD16OSR_256
#define SD_DF                   SD16DF
#define SD_GRP                  SD16GRP
#define SD_SNGL                 SD16SNGL
#define SD_REFON                SD16REFON
#define SD_LP                   SD16LP
#define SD_IE                   SD16IE
#define SD_SSEL_1               SD16SSEL_1
#define SD_DIV_2                SD16DIV_2
#define SD_DIV_3                SD16DIV_3
#define SD_GAIN_1               SD16GAIN_1
#define SD_GAIN_2               SD16GAIN_2
#define SD_GAIN_4               SD16GAIN_4
#define SD_GAIN_8               SD16GAIN_8
#define SD_GAIN_16              SD16GAIN_16
#define SD_GAIN_TEMPERATURE     SD16GAIN_1

#elif defined(__MSP430_HAS_SD24__)  ||  defined(__MSP430_HAS_SD24_2__)  ||  defined(__MSP430_HAS_SD24_3__)

#define SD_CCTL_                SD24CCTL
#define SD_INCTL_               SD24INCTL
#define SD_PRE_                 SD24PRE

#define SD_CTL                  SD24CTL
#if defined(__MSP430_HAS_SD24_2__)
#define SD_CCTL_TRIGGER         SD24CCTL1
#elif defined(__MSP430_HAS_SD24_3__)
#define SD_CCTL_TRIGGER         SD24CCTL2
#elif defined(__MSP430_HAS_SD24_4__)
#define SD_CCTL_TRIGGER         SD24CCTL3
#endif

#define SD_INCH_VOLTAGE         SD24INCH_0
#define SD_INCH_CURRENT         SD24INCH_0
#define SD_INCH_TEMPERATURE     SD24INCH_6
#define SD_SC                   SD24SC
#define SD_OSR_32               SD24OSR_32
#define SD_OSR_256              SD24OSR_256
#define SD_DF                   SD24DF
#define SD_GRP                  SD24GRP
#define SD_SNGL                 SD24SNGL
#define SD_REFON                SD24REFON
#define SD_LP                   SD24LP
#define SD_IE                   SD24IE
#define SD_DIV_2                SD24DIV_2
#define SD_DIV_3                SD24DIV_3
#define SD_GAIN_1               SD24GAIN_1
#define SD_GAIN_2               SD24GAIN_2
#define SD_GAIN_4               SD24GAIN_4
#define SD_GAIN_8               SD24GAIN_8
#define SD_GAIN_16              SD24GAIN_16
#define SD_GAIN_TEMPERATURE     SD24GAIN_1

#elif defined(__MSP430_HAS_SD24_A2__)

#define SD_CCTL_                SD24CCTL
#define SD_INCTL_               SD24INCTL
#define SD_PRE_                 SD24PRE

#define SD_CTL                  SD24CTL
#define SD_CCTL_TRIGGER         SD24CCTL1

#define SD_INCH_VOLTAGE         SD24INCH_0
#define SD_INCH_CURRENT         SD24INCH_0
#define SD_INCH_TEMPERATURE     SD24INCH_6
#define SD_SC                   SD24SC
#define SD_OSR_32               SD24OSR_32
#define SD_OSR_256              SD24OSR_256
#define SD_DF                   SD24DF
#define SD_GRP                  SD24GRP
#define SD_SNGL                 SD24SNGL
#define SD_REFON                SD24REFON
#define SD_LP                   SD24LP
#define SD_IE                   SD24IE
#define SD_SSEL_1               SD24SSEL_1
#define SD_DIV_2                SD24DIV_2
#define SD_DIV_3                SD24DIV_3
#define SD_XDIV_1               SD24XDIV_1
#define SD_XDIV_2               SD24XDIV_2
#define SD_GAIN_1               SD24GAIN_1
#define SD_GAIN_2               SD24GAIN_2
#define SD_GAIN_4               SD24GAIN_4
#define SD_GAIN_8               SD24GAIN_8
#define SD_GAIN_16              SD24GAIN_16
#define SD_GAIN_TEMPERATURE     SD24GAIN_1

#elif defined(__MSP430_HAS_SD24_A3__)

#define SD_CCTL_                SD24CCTL
#define SD_INCTL_               SD24INCTL
#define SD_PRE_                 SD24PRE

#define SD_CTL                  SD24CTL
#define SD_CCTL_TRIGGER         SD24CCTL2

#define SD_INCH_VOLTAGE         SD24INCH_0
#define SD_INCH_CURRENT         SD24INCH_0
#define SD_INCH_TEMPERATURE     SD24INCH_6
#define SD_SC                   SD24SC
#define SD_OSR_32               SD24OSR_32
#define SD_OSR_256              SD24OSR_256
#define SD_DF                   SD24DF
#define SD_GRP                  SD24GRP
#define SD_SNGL                 SD24SNGL
#define SD_REFON                SD24REFON
#define SD_LP                   SD24LP
#define SD_IE                   SD24IE
#define SD_SSEL_1               SD24SSEL_1
#define SD_DIV_2                SD24DIV_2
#define SD_DIV_3                SD24DIV_3
#define SD_XDIV_1               SD24XDIV_1
#define SD_XDIV_2               SD24XDIV_2
#define SD_GAIN_1               SD24GAIN_1
#define SD_GAIN_2               SD24GAIN_2
#define SD_GAIN_4               SD24GAIN_4
#define SD_GAIN_8               SD24GAIN_8
#define SD_GAIN_16              SD24GAIN_16
#define SD_GAIN_TEMPERATURE     SD24GAIN_1

#elif defined(__MSP430_HAS_SD24_B__)

#define SD_CCTL_                SD24BCCTL
#define SD_INCTL_               SD24BINCTL
#define SD_OSR_                 SD24BOSR
#define SD_PRE_                 SD24BPRE

#define SD_CTL0                 SD24BCTL0
#define SD_CTL1                 SD24BCTL1
    #if defined(__MSP430_HAS_SD24_B2__)
#define SD_CCTL_TRIGGER         SD24BCCTL1
#define SD24BIE_MASK            0x03
    #elif defined(__MSP430_HAS_SD24_B3__)
#define SD_CCTL_TRIGGER         SD24BCCTL2
#define SD24BIE_MASK            0x07
    #elif defined(__MSP430_HAS_SD24_B4__)
#define SD_CCTL_TRIGGER         SD24BCCTL3
#define SD24BIE_MASK            0x0F
    #elif defined(__MSP430_HAS_SD24_B6__)
#define SD_CCTL_TRIGGER         SD24BCCTL5
#define SD24BIE_MASK            0x3F
    #elif defined(__MSP430_HAS_SD24_B7__)
#define SD_CCTL_TRIGGER         SD24BCCTL6
#define SD24BIE_MASK            0x7F
    #endif

#define SD_SC                   SD24GRP0SC
#define SD_OSR_32               SD24OSR_32
#define SD_OSR_256              SD24OSR_256
#define SD_DF_1                 SD24DF_1
#define SD_GRP                  SD24SCS_4
#define SD_SNGL                 SD24SNGL
#define SD_REFON                SD24REFON
#define SD_LP                   SD24LP

#elif defined(__MSP430_HAS_SD16_A3__)  ||  defined(__MSP430_HAS_SD16_A4__)  ||  defined(__MSP430_HAS_SD16_A6__)  ||  defined(__MSP430_HAS_SD16_A7__)

#define SD_CCTL_                SD16CCTL
#define SD_INCTL_               SD16INCTL
#define SD_PRE_                 SD16PRE

    #if defined(__MSP430_HAS_SD16_A7__)
#define SD_CCTL_TRIGGER         SD16CCTL6
    #elif defined(__MSP430_HAS_SD16_A6__)
#define SD_CCTL_TRIGGER         SD16CCTL5
    #elif defined(__MSP430_HAS_SD16_A5__)
#define SD_CCTL_TRIGGER         SD16CCTL4
    #elif defined(__MSP430_HAS_SD16_A4__)
#define SD_CCTL_TRIGGER         SD16CCTL3
    #elif defined(__MSP430_HAS_SD16_A3__)
#define SD_CCTL_TRIGGER         SD16CCTL2
    #elif defined(__MSP430_HAS_SD16_A2__)
#define SD_CCTL_TRIGGER         SD16CCTL1
    #endif

#define SD_CTL                  SD16CTL

#define SD_INCH_VOLTAGE         SD16INCH_0
#define SD_INCH_CURRENT         SD16INCH_0
#define SD_INCH_TEMPERATURE     SD16INCH_6
#define SD_SC                   SD16SC
#define SD_OSR_32               SD16OSR_32
#define SD_OSR_256              SD16OSR_256
#define SD_DF                   SD16DF
#define SD_GRP                  SD16GRP
#define SD_SNGL                 SD16SNGL
#define SD_REFON                SD16REFON
#define SD_LP                   SD16LP
#define SD_IE                   SD16IE
#define SD_SSEL_1               SD16SSEL_1
#define SD_DIV_2                SD16DIV_2
#define SD_DIV_3                SD16DIV_3
#define SD_XDIV_1               SD16XDIV_1
#define SD_XDIV_2               SD16XDIV_2
#define SD_GAIN_1               SD16GAIN_1
#define SD_GAIN_2               SD16GAIN_2
#define SD_GAIN_4               SD16GAIN_4
#define SD_GAIN_8               SD16GAIN_8
#define SD_GAIN_16              SD16GAIN_16
#define SD_GAIN_TEMPERATURE     SD16GAIN_1

#else
#error No known ADC available.
#endif
