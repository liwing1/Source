/*******************************************************************************
 *  metrology-setup.c -
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
#if !defined(__MSP430__)
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#endif
#if defined(__GNUC__)
#include <signal.h>
#endif
#include <math.h>

#include <emeter-toolkit.h>

#include "emeter-metrology.h"

#include "emeter-metrology-internal.h"

#if defined(__MSP430_HAS_SD16_2__)  ||  defined(__MSP430_HAS_SD16_3__)
#define SD16CONF0_FUDGE     0x70
#define SD16CONF1_FUDGE     0x48    //0x38
#endif

#define sd_resolve_x(x) THAT_WAS_ ## x
#define sd_resolve(x) sd_resolve_x(x)

#if defined(__MSP430_HAS_SD24_B__)
#define sd_set_normal_live_current_mode_(a) \
    sd_xxxx_reg(SD_INCTL_, a) = SD_LIVE_CURRENT_GAIN; \
    sd_xxxx_reg(SD_CCTL_, a) = SD_DF_1 | SD24SCS_4; \
    sd_xxxx_reg(SD_OSR_, a) = 256 - 1; \
    sd_xxxx_reg(SD_PRE_, a) = 0;
#define sd_set_normal_live_current_mode(a) sd_set_normal_live_current_mode_(a)

#define sd_set_limp_live_current_mode_(a) \
    sd_xxxx_reg(SD_INCTL_, a) = SD_LIVE_CURRENT_GAIN; \
    sd_xxxx_reg(SD_CCTL_, a) = SD_DF_1 | SD_SNGL | SD_GRP; \
    sd_xxxx_reg(SD_OSR_, a) = 32 - 1; \
    sd_xxxx_reg(SD_PRE_, a) = 0;
#define sd_set_limp_live_current_mode(a) sd_set_limp_live_current_mode_(a)

#define sd_set_idle_live_current_mode_(a) \
    sd_xxxx_reg(SD_INCTL_, a) = 0; \
    sd_xxxx_reg(SD_CCTL_, a) = 0; \
    sd_xxxx_reg(SD_OSR_, a) = 0; \
    sd_xxxx_reg(SD_PRE_, a) = 0;
#define sd_set_idle_live_current_mode(a) sd_set_idle_live_current_mode_(a)

#define sd_set_normal_neutral_current_mode_(a) \
    sd_xxxx_reg(SD_INCTL_, a) = SD_NEUTRAL_CURRENT_GAIN; \
    sd_xxxx_reg(SD_CCTL_, a) = SD_DF_1 | SD24SCS_4; \
    sd_xxxx_reg(SD_OSR_, a) = 256 - 1; \
    sd_xxxx_reg(SD_PRE_, a) = 0;
#define sd_set_normal_neutral_current_mode(a) sd_set_normal_neutral_current_mode_(a)

#define sd_set_limp_neutral_current_mode_(a) \
    sd_xxxx_reg(SD_INCTL_, a) = SD_NEUTRAL_CURRENT_GAIN; \
    sd_xxxx_reg(SD_CCTL_, a) = SD_DF_1 | SD_SNGL | SD_GRP; \
    sd_xxxx_reg(SD_OSR_, a) = 32 - 1; \
    sd_xxxx_reg(SD_PRE_, a) = 0;
#define sd_set_limp_neutral_current_mode(a) sd_set_limp_neutral_current_mode_(a)

#define sd_set_idle_neutral_current_mode_(a) \
    sd_xxxx_reg(SD_INCTL_, a) = 0; \
    sd_xxxx_reg(SD_CCTL_, a) = 0; \
    sd_xxxx_reg(SD_OSR_, a) = 0; \
    sd_xxxx_reg(SD_PRE_, a) = 0;
#define sd_set_idle_neutral_current_mode(a) sd_set_idle_neutral_current_mode_(a)

#define sd_set_normal_voltage_mode_(a) \
    sd_xxxx_reg(SD_INCTL_, a) = SD_VOLTAGE_GAIN; \
    sd_xxxx_reg(SD_CCTL_, a) = SD_DF_1 | SD24ALGN | SD24SCS_4; \
    sd_xxxx_reg(SD_OSR_, a) = 256 - 1; \
    sd_xxxx_reg(SD_PRE_, a) = 0;
#define sd_set_normal_voltage_mode(a) sd_set_normal_voltage_mode_(a)

#define sd_set_limp_voltage_mode_(a) \
    sd_xxxx_reg(SD_INCTL_, a) = SD_VOLTAGE_GAIN; \
    sd_xxxx_reg(SD_CCTL_, a) = SD_DF_1 | SD_SNGL | SD_GRP; \
    sd_xxxx_reg(SD_OSR_, a) = 32 - 1; \
    sd_xxxx_reg(SD_PRE_, a) = 0;
#define sd_set_limp_voltage_mode(a) sd_set_limp_voltage_mode_(a)

#define sd_set_idle_voltage_mode_(a) \
    sd_xxxx_reg(SD_INCTL_, a) = 0; \
    sd_xxxx_reg(SD_CCTL_, a) = 0; \
    sd_xxxx_reg(SD_OSR_, a) = 0; \
    sd_xxxx_reg(SD_PRE_, a) = 0;
#define sd_set_idle_voltage_mode(a) sd_set_idle_voltage_mode_(a)

#else

#define sd_set_normal_live_current_mode_(a) \
    sd_xxxx_reg(SD_INCTL_, a) = SD_INCH_CURRENT | SD_LIVE_CURRENT_GAIN; \
    sd_xxxx_reg(SD_CCTL_, a) = SD_OSR_256 | SD_DF | SD_GRP | SD_IE; \
    sd_xxxx_reg(SD_PRE_, a) = 0;
#define sd_set_normal_live_current_mode(a) sd_set_normal_live_current_mode_(a)

#define sd_set_limp_live_current_mode_(a) \
    sd_xxxx_reg(SD_INCTL_, a) = SD_INCH_CURRENT | SD_LIVE_CURRENT_GAIN; \
    sd_xxxx_reg(SD_CCTL_, a) = SD_OSR_32 | SD_DF | SD_SNGL | SD_GRP | SD_IE; \
    sd_xxxx_reg(SD_PRE_, a) = 0;
#define sd_set_limp_live_current_mode(a) sd_set_limp_live_current_mode_(a)

#define sd_set_idle_live_current_mode_(a) \
    sd_xxxx_reg(SD_INCTL_, a) = 0; \
    sd_xxxx_reg(SD_CCTL_, a) = 0; \
    sd_xxxx_reg(SD_PRE_, a) = 0;
#define sd_set_idle_live_current_mode(a) sd_set_idle_live_current_mode_(a)

#define sd_set_normal_neutral_current_mode_(a) \
    sd_xxxx_reg(SD_INCTL_, a) = SD_INCH_CURRENT | SD_NEUTRAL_CURRENT_GAIN; \
    sd_xxxx_reg(SD_CCTL_, a) = SD_OSR_256 | SD_DF | SD_GRP | SD_IE; \
    sd_xxxx_reg(SD_PRE_, a) = 0;
#define sd_set_normal_neutral_current_mode(a) sd_set_normal_neutral_current_mode_(a)

#define sd_set_limp_neutral_current_mode_(a) \
    sd_xxxx_reg(SD_INCTL_, a) = SD_INCH_CURRENT | SD_NEUTRAL_CURRENT_GAIN; \
    sd_xxxx_reg(SD_CCTL_, a) = SD_OSR_32 | SD_DF | SD_SNGL | SD_GRP | SD_IE; \
    sd_xxxx_reg(SD_PRE_, a) = 0;
#define sd_set_limp_neutral_current_mode(a) sd_set_limp_neutral_current_mode_(a)

#define sd_set_idle_neutral_current_mode_(a) \
    sd_xxxx_reg(SD_INCTL_, a) = 0; \
    sd_xxxx_reg(SD_CCTL_, a) = 0; \
    sd_xxxx_reg(SD_PRE_, a) = 0;
#define sd_set_idle_neutral_current_mode(a) sd_set_idle_neutral_current_mode_(a)

#define sd_set_normal_voltage_mode_(a) \
    sd_xxxx_reg(SD_INCTL_, a) = SD_INCH_VOLTAGE | SD_VOLTAGE_GAIN; \
    sd_xxxx_reg(SD_CCTL_, a) = SD_OSR_256 | SD_DF | SD_GRP; \
    sd_xxxx_reg(SD_PRE_, a) = 0;
#define sd_set_normal_voltage_mode(a) sd_set_normal_voltage_mode_(a)

#define sd_set_limp_voltage_mode_(a) \
    sd_xxxx_reg(SD_INCTL_, a) = SD_INCH_VOLTAGE | SD_VOLTAGE_GAIN; \
    sd_xxxx_reg(SD_CCTL_, a) = SD_OSR_32 | SD_DF | SD_SNGL | SD_GRP | SD_IE; \
    sd_xxxx_reg(SD_PRE_, a) = 0;
#define sd_set_limp_voltage_mode(a) sd_set_limp_voltage_mode_(a)

#define sd_set_idle_voltage_mode_(a) \
    sd_xxxx_reg(SD_INCTL_, a) = 0; \
    sd_xxxx_reg(SD_CCTL_, a) = 0; \
    sd_xxxx_reg(SD_PRE_, a) = 0;
#define sd_set_idle_voltage_mode(a) sd_set_idle_voltage_mode_(a)

#endif

#if defined(__MSP430_HAS_SD24_B2__)
extern int32_t dma_sd24b_buffer[2];
#elif defined(__MSP430_HAS_SD24_B3__)
extern int32_t dma_sd24b_buffer[3];
#elif defined(__MSP430_HAS_SD24_B4__)
extern int32_t dma_sd24b_buffer[4];
#elif defined(__MSP430_HAS_SD24_B6__)
extern int32_t dma_sd24b_buffer[6];
#elif defined(__MSP430_HAS_SD24_B7__)
extern int32_t dma_sd24b_buffer[7];
#endif
#if NUM_PHASES > 1  &&  defined(__MSP430_HAS_SD24_B3__)  &&  defined(__MSP430_HAS_ADC10_A__)
extern int16_t dma_adc_buffer[16];
#endif

#if defined(SAG_POWER_DOWN_SUPPORT)
static void sag_power_down_control(void)
{
    int64_t xxx;
#if NUM_PHASES > 1
    struct phase_parms_s *phase;
    struct phase_calibration_data_s const *phase_cal;
    int ph;
#endif

#if NUM_PHASES > 1
    phase = working_data.phases;
    phase_cal = cal_info->phases;
    for (ph = 0;  ph < NUM_PHASES;  ph++, phase++, phase_cal++)
#endif
    {
        /* Find the current sag and swell thresholds, based on the current mains period */
        xxx = MAINS_NOMINAL_VOLTAGE*1414LL*65536LL*4LL;
        xxx /= cal_info->phases[ph].V_rms_scale_factor[normal_limp];
        xxx *= (((100 - SAG_POWER_DOWN_THRESHOLD)*256)/100);
        phase->metrology.sag_power_down_threshold = xxx >> 16;
    }
}
#endif

/*
 * Analog front-end initialization routine.
 *
 * Configures the sigma-delta ADC module as analog front-end for
 * a tamper-resistant meter using a current transformer and a
 * shunt as current sensors (see configuration of channel 0 and 1).
 */
void metrology_init_analog_front_end_normal_mode(void)
{
#if NUM_PHASES > 1
    int ph;
#endif

    __istate_t interrupt_state;
    interrupt_state = __get_interrupt_state();
    __disable_interrupt();

    /* The general configuration of the analog front-end,
       that applies to all channels: clock selection (SMCLK) and divider
       settings (depending on SMCLK frequency) and reference voltage
       selections. */
    
   __disable_interrupt();
#if defined(__MSP430_HAS_SD24_B__)
    /* Enable the voltage reference at 2.0V */
    REFCTL0 = REFMSTR | REFVSEL_1 | REFON;
    SD_CTL1 &= ~SD24GRP0SC;
#else
    sd_xxxx_reg(SD_CCTL_, PHASE_1_VOLTAGE_ADC_CHANNEL) &= ~SD_SC;
    sd_xxxx_reg(SD_CCTL_, PHASE_1_CURRENT_ADC_CHANNEL) &= ~SD_SC;
    #if !defined(VOLTAGE_SIGNAL_IS_COMMON)
        #if NUM_PHASES >= 2
    sd_xxxx_reg(SD_CCTL_, PHASE_2_VOLTAGE_ADC_CHANNEL) &= ~SD_SC;
    sd_xxxx_reg(SD_CCTL_, PHASE_2_CURRENT_ADC_CHANNEL) &= ~SD_SC;
        #endif
        #if NUM_PHASES >= 3
    sd_xxxx_reg(SD_CCTL_, PHASE_3_VOLTAGE_ADC_CHANNEL) &= ~SD_SC;
    sd_xxxx_reg(SD_CCTL_, PHASE_3_CURRENT_ADC_CHANNEL) &= ~SD_SC;
        #endif
    #endif
    #if defined(NEUTRAL_MONITOR_SUPPORT)
    sd_xxxx_reg(SD_CCTL_, NEUTRAL_CURRENT_ADC_CHANNEL) &= ~SD_SC;
    #endif
#endif

    /* Use the SMCLK. It will need to be divided to produce an ADC clock of 1.048576MHz. The ratio will depend on the
       frequency the SMCLK is set to. Use the internal reference. */
#if defined(__MSP430_HAS_SD24_B__)
    SD24BCTL0 = SD24SSEL__SMCLK     /* Clock is SMCLK */
        #if SD_CLOCK_DIVISION == 8
              | SD24PDIV_2          /* Divide by 8 (4*2) => ADC clock: 1.048576MHz */
              | SD24DIV0
        #elif SD_CLOCK_DIVISION == 16
              | SD24PDIV_2          /* Divide by 16 (4*4) => ADC clock: 1.048576MHz */
              | SD24DIV1
        #elif SD_CLOCK_DIVISION == 20
              | SD24PDIV_2          /* Divide by 20 (5*4) => ADC clock: 1.048576MHz */
              | SD24DIV2
        #elif SD_CLOCK_DIVISION == 24
              | SD24PDIV_2          /* Divide by 24 (6*4) => ADC clock: 1.048576MHz */
              | SD24DIV2 | SD24DIV0
        #endif
              | SD24REFS            /* Use internal reference */
              | SD24OV32;
    SD24BCTL1 = 0;
#else
    #if defined(__MSP430_HAS_SD24__)  ||  defined(__MSP430_HAS_SD24_2__)  ||  defined(__MSP430_HAS_SD24_3__)
    SD24CTL = SD24REFS;
    #elif SD_CLOCK_DIVISION == 4
    SD_CTL = SD_SSEL_1 | SD_DIV_2 | SD_REFON;
    #elif SD_CLOCK_DIVISION == 8
    SD_CTL = SD_SSEL_1 | SD_DIV_3 | SD_REFON;
    #elif SD_CLOCK_DIVISION == 12
    SD_CTL = SD_SSEL_1 | SD_XDIV_1 | SD_DIV_2 | SD_REFON;
    #elif SD_CLOCK_DIVISION == 16
    SD_CTL = SD_SSEL_1 | SD_XDIV_2 | SD_REFON;
    #endif
    #if defined(SD16CONF0_FUDGE)
    SD16CONF0 = SD16CONF0_FUDGE;
    #endif
    #if defined(SD16CONF1_FUDGE)
    SD16CONF1 = SD16CONF1_FUDGE;
    #endif
    #if defined(SD24CONF1_FUDGE)
    SD24CONF1 = SD24CONF1_FUDGE;
    #endif
#endif

    sd_set_normal_live_current_mode(PHASE_1_CURRENT_ADC_CHANNEL);
#if NUM_PHASES >= 2
    sd_set_normal_live_current_mode(PHASE_2_CURRENT_ADC_CHANNEL);
#endif
#if NUM_PHASES >= 3
    sd_set_normal_live_current_mode(PHASE_3_CURRENT_ADC_CHANNEL);
#endif
#if NUM_PHASES >= 4
    sd_set_normal_live_current_mode(PHASE_4_CURRENT_ADC_CHANNEL);
#endif
#if NUM_PHASES >= 5
    sd_set_normal_live_current_mode(PHASE_5_CURRENT_ADC_CHANNEL);
#endif
#if NUM_PHASES >= 6
    sd_set_normal_live_current_mode(PHASE_6_CURRENT_ADC_CHANNEL);
#endif
#if defined(NEUTRAL_MONITOR_SUPPORT)
    sd_set_normal_neutral_current_mode(NEUTRAL_CURRENT_ADC_CHANNEL);
#endif

#if 0 //defined(__MSP430_HAS_SD24_B__)
    /* Trigger DMA channel 0 from SD24IFG0 */
    DMACTL0 |= DMA0TSEL__SD24IFG;
    __data16_write_addr((uint16_t) &DMA0SA, (uint32_t) &SD24BMEML0);
    __data16_write_addr((uint16_t) &DMA0DA, (uint32_t) &dma_sd24b_buffer[0]);
    #if defined(__MSP430_HAS_SD24_B2__)
    DMA0SZ = 2*2;
    #elif defined(__MSP430_HAS_SD24_B3__)
    DMA0SZ = 2*3;
    #elif defined(__MSP430_HAS_SD24_B4__)
    DMA0SZ = 2*4;
    #elif defined(__MSP430_HAS_SD24_B6__)
    DMA0SZ = 2*6;
    #elif defined(__MSP430_HAS_SD24_B7__)
    DMA0SZ = 2*7;
    #endif
    /* Enable, source and destination addresses incremented, repeating single transfer. */
    DMA0CTL = DMADT_4 | DMASRCINCR_3 | DMADSTINCR_3 | DMAEN;
#endif

#if NUM_PHASES > 1  &&  defined(__MSP430_HAS_SD24_B3__)  &&  defined(__MSP430_HAS_ADC10_A__)
    /* The ADC10A is going to measure the three voltages. It will be triggered by the SD24B, and
       will deliver the three results by DMA */
    /* Trigger DMA channel 1 from ADCIFG0 */
    DMACTL0 |= DMA1TSEL__ADC10IFG0;
    __data16_write_addr((uint16_t) &DMA1SA, (uint32_t) &ADC10MEM0);
    __data16_write_addr((uint16_t) &DMA1DA, (uint32_t) &dma_adc_buffer[0]);
    /* Channels 15 to 0. We only want 5 to 3 really, but the sequencing engine in the ADC10A module
       only supports channel X to channel 0, and currently 5, 4, and 3 are our voltage channels */
    DMA1SZ = 16;
    /* Enable, destination address incremented, repeating single transfer. */
    DMA1CTL = DMADT_4 | DMADSTINCR_3 | DMAEN |DMAIE;
    ADC10CTL0 &= ~ADC10ENC;
    /* Clear pending interrupts to ensure trigger for DMA */
    ADC10IFG = 0;

    /* ADC on, ADC10 waits for trigger from the SD24, sampling time 2us (8xADCclk), auto next conv. */
    ADC10CTL0 = ADC10SHT0 | ADC10ON | ADC10MSC;
    #if SD_CLOCK_DIVISION == 8
    /* Triggered by the SD24, SMCLK/2 = 4MHz, Sequence of channels */
    ADC10CTL1 = ADC10SHP | ADC10SHS_3 | ADC10DIV_1 | ADC10SSEL_3 | ADC10CONSEQ_1;
    #elif SD_CLOCK_DIVISION == 16
    /* Triggered by the SD24, SMCLK/4 = 4MHz, Sequence of channels */
    ADC10CTL1 = ADC10SHP | ADC10SHS_3 | ADC10DIV_3 | ADC10SSEL_3 | ADC10CONSEQ_1;
    #elif SD_CLOCK_DIVISION == 20
    /* Triggered by the SD24, SMCLK/5 = 4MHz, Sequence of channels */
    ADC10CTL1 = ADC10SHP | ADC10SHS_3 | ADC10DIV_4 | ADC10SSEL_3 | ADC10CONSEQ_1;
    #elif SD_CLOCK_DIVISION == 24
    /* Triggered by the SD24, SMCLK/6 = 4MHz, Sequence of channels */
    ADC10CTL1 = ADC10SHP | ADC10SHS_3 | ADC10DIV_5 | ADC10SSEL_3 | ADC10CONSEQ_1;
    #endif
    /* Predivide by 1, 10bit resolution, results are scaled to 16bit signed numbers, so
       Vr- = 0x8000 and Vr+ = 0x7FC0. */
    ADC10CTL2 = ADC10RES | ADC10DF;
    /* Vr- = Vss, Vr+ = Vref+, The sequence of conversions will be ch15 to ch0. */
    ADC10MCTL0 = ADC10SREF_1 | ADC10INCH_15;
    /* Start ADC and wait for a trigger from the SD24 */
    ADC10CTL0 |= ADC10ENC;
     SD24BTRGPRE =0;
    SD24BIE = 0x07;
    SD24BTRGCTL |=SD24TRGIE;

#else
    #if defined(__MSP430_HAS_ADC10_A__)  &&  (defined(TEMPERATURE_SUPPORT)  ||  defined(VCC_MEASURE_SUPPORT))
    ADC10CTL0 &= ~ADC10ENC;
    /* Clear pending interrupts to ensure trigger for DMA */
    ADC10IFG = 0;

    /* ADC on, ADC10 waits for trigger from the SD24, sampling time 2us 8xADCclk, auto next conv. */
    ADC10CTL0 = ADC10SHT0 | ADC10ON;
        #if SD_CLOCK_DIVISION == 8
    /* Triggered by the SD24, SMCLK/2 = 4MHz, Sequence of channels */
    ADC10CTL1 = ADC10SHP | ADC10SHS_0 | ADC10DIV_1 | ADC10SSEL_3 | ADC10CONSEQ_0;
	    #elif SD_CLOCK_DIVISION == 16
    /* Triggered by the SD24, SMCLK/4 = 4MHz, Sequence of channels */
    ADC10CTL1 = ADC10SHP | ADC10SHS_0 | ADC10DIV_3 | ADC10SSEL_3 | ADC10CONSEQ_0;
	    #elif SD_CLOCK_DIVISION == 20
    /* Triggered by the SD24, SMCLK/5 = 4MHz, Sequence of channels */
    ADC10CTL1 = ADC10SHP | ADC10SHS_0 | ADC10DIV_4 | ADC10SSEL_3 | ADC10CONSEQ_0;
	    #elif SD_CLOCK_DIVISION == 24
    /* Triggered by the SD24, SMCLK/6 = 4MHz, Sequence of channels */
    ADC10CTL1 = ADC10SHP | ADC10SHS_0 | ADC10DIV_5 | ADC10SSEL_3 | ADC10CONSEQ_0;
        #endif
    /* Predivide by 1, 10bit resolution, results are scaled to 16bit signed numbers, so
       Vr- = 0x8000 and Vr+ = 0x7FC0. */
    ADC10CTL2 = ADC10RES | ADC10DF;
    /* Vr- = Vss, Vr+ = Vref+, The channel will be 0. */
    ADC10MCTL0 = ADC10SREF_1 | ADC10INCH_0;
    ADC10IE = ADC10IE0 | ADC10OVIE | ADC10TOVIE;
    /* Start ADC and wait for a software start conversion trigger */
    ADC10CTL0 |= ADC10ENC;
    #endif

    /* The three voltages are phase coincident. Only interrupt from one of them. */
    sd_set_normal_voltage_mode(PHASE_1_VOLTAGE_ADC_CHANNEL);
    #if !defined(VOLTAGE_SIGNAL_IS_COMMON)
        #if NUM_PHASES >= 2
    sd_set_normal_voltage_mode(PHASE_2_VOLTAGE_ADC_CHANNEL);
        #endif
        #if NUM_PHASES >= 3
    sd_set_normal_voltage_mode(PHASE_3_VOLTAGE_ADC_CHANNEL);
        #endif
    #endif
#endif

#if NUM_PHASES > 1
    for (ph = 0;  ph < NUM_PHASES;  ph++)
#endif
        working_data.phases[ph].metrology.current[0].in_phase_correction.sd_preloaded_offset = 0;
#if NUM_PHASES == 1  &&  defined(NEUTRAL_MONITOR_SUPPORT)
    working_data.phases[0].metrology.current[1].in_phase_correction.sd_preloaded_offset = 0;
#endif

#if defined(__MSP430_HAS_SD24_B__)
    #if NUM_PHASES > 1  &&  defined(__MSP430_HAS_SD24_B3__)  &&  defined(__MSP430_HAS_ADC10_A__)
    SD24BTRGCTL = SD24SCS__GROUP0 | SD24TRGIE;      /* Trigger generation group 0 SD24TRGIE */
    SD24BCTL1 |= SD24GRP0SC;            /* Start group 0 */
    #else
    SD24BIE = 0x0007;
    #endif
    SD_CTL1 |= SD24GRP0SC;
#else
    sd_xxxx_reg(SD_CCTL_, PHASE_1_VOLTAGE_ADC_CHANNEL) |= SD_IE;
    #if NUM_PHASES >= 2
    sd_xxxx_reg(SD_CCTL_, PHASE_1_CURRENT_ADC_CHANNEL) |= SD_IE;
    sd_xxxx_reg(SD_CCTL_, PHASE_2_CURRENT_ADC_CHANNEL) |= SD_IE;
    #endif
    #if NUM_PHASES >= 3
    sd_xxxx_reg(SD_CCTL_, PHASE_3_CURRENT_ADC_CHANNEL) |= SD_IE;
    #endif
    /* We want to start conversion on the last converter in the set */
    SD_CCTL_TRIGGER |= SD_SC;
#endif

#if defined(TEMPERATURE_SUPPORT)
    /* Initialise the temperature measurement to the intercept (i.e. 0C), so it doesn't take
       forever to settle. */
    //raw_temperature_from_adc = (int32_t) cal_info->temperature_sensor_intercept << 3;
#endif
    __set_interrupt_state(interrupt_state);
}

#if defined(LIMP_MODE_SUPPORT)
void metrology_init_analog_front_end_limp_mode(void)
{
    #if NUM_PHASES > 1
    int ph;
    #endif
    
     __istate_t interrupt_state;
    interrupt_state = __get_interrupt_state();
    __disable_interrupt();

    #if defined(__MSP430_HAS_SD24_B__)
    SD_CTL1 &= ~SD_SC;
    #else
    sd_xxxx_reg(SD_CCTL_, PHASE_1_VOLTAGE_ADC_CHANNEL) &= ~SD_SC;
    sd_xxxx_reg(SD_CCTL_, PHASE_1_CURRENT_ADC_CHANNEL) &= ~SD_SC;
        #if NUM_PHASES >= 2
    sd_xxxx_reg(SD_CCTL_, PHASE_2_VOLTAGE_ADC_CHANNEL) &= ~SD_SC;
    sd_xxxx_reg(SD_CCTL_, PHASE_2_CURRENT_ADC_CHANNEL) &= ~SD_SC;
        #endif
        #if NUM_PHASES >= 3
    sd_xxxx_reg(SD_CCTL_, PHASE_3_VOLTAGE_ADC_CHANNEL) &= ~SD_SC;
    sd_xxxx_reg(SD_CCTL_, PHASE_3_CURRENT_ADC_CHANNEL) &= ~SD_SC;
        #endif
        #if defined(NEUTRAL_MONITOR_SUPPORT)
    sd_xxxx_reg(SD_CCTL_, NEUTRAL_CURRENT_ADC_CHANNEL) &= ~SD_SC;
        #endif
    #endif

    #if defined(__MSP430_HAS_SD24_B__)
    SD24BCTL0 = SD24SSEL__SMCLK     /* Clock is SMCLK */
        #if SD_CLOCK_DIVISION == 8
              | SD24PDIV_2          /* Divide by 8 (4*2) => ADC clock: 1.048576MHz */
              | SD24DIV0
        #elif SD_CLOCK_DIVISION == 16
              | SD24PDIV_2          /* Divide by 16 (4*4) => ADC clock: 1.048576MHz */
              | SD24DIV1
        #elif SD_CLOCK_DIVISION == 20
              | SD24PDIV_2          /* Divide by 20 (5*4) => ADC clock: 1.048576MHz */
              | SD24DIV2
        #elif SD_CLOCK_DIVISION == 24
              | SD24PDIV_2          /* Divide by 24 (6*4) => ADC clock: 1.048576MHz */
              | SD24DIV2 | SD24DIV0
        #endif
              | SD24REFS            /* Use internal reference */
              | SD24OV32;
    #else
        #if defined(__MSP430_HAS_SD24__)  ||  defined(__MSP430_HAS_SD24_2__)
    SD24CTL = SD24REFS;
        #elif SD_CLOCK_DIVISION == 4
    SD_CTL = SD_SSEL_1 | SD_DIV_2 | SD_REFON;
        #elif SD_CLOCK_DIVISION == 8
    SD_CTL = SD_SSEL_1 | SD_DIV_3 | SD_REFON;
        #elif SD_CLOCK_DIVISION == 12
    SD_CTL = SD_SSEL_1 | SD_XDIV_1 | SD16DIV_2 | SD_REFON;
        #elif SD_CLOCK_DIVISION == 16
    SD_CTL = SD_SSEL_1 | SD_XDIV_2 | SD_REFON;
        #endif
        #if defined(SD16CONF0_FUDGE)
    SD16CONF0 = SD16CONF0_FUDGE;
        #endif
        #if defined(SD16CONF1_FUDGE)
    SD16CONF1 = SD16CONF1_FUDGE;
        #endif
        #if defined(SD24CONF1_FUDGE)
    SD24CONF1 = SD24CONF1_FUDGE;
        #endif
    #endif

    sd_set_limp_live_current_mode(PHASE_1_CURRENT_ADC_CHANNEL);
    #if NUM_PHASES >= 2
    sd_set_limp_live_current_mode(PHASE_2_CURRENT_ADC_CHANNEL);
    #endif
    #if NUM_PHASES >= 3
    sd_set_limp_live_current_mode(PHASE_3_CURRENT_ADC_CHANNEL);
    #endif
    #if defined(NEUTRAL_MONITOR_SUPPORT)
    sd_set_limp_neutral_current_mode(NEUTRAL_CURRENT_ADC_CHANNEL);
    #endif

    sd_set_limp_voltage_mode(PHASE_1_VOLTAGE_ADC_CHANNEL);
    #if NUM_PHASES >= 2
    sd_set_limp_voltage_mode(PHASE_2_VOLTAGE_ADC_CHANNEL);
    #endif
    #if NUM_PHASES >= 3
    sd_set_limp_voltage_mode(PHASE_3_VOLTAGE_ADC_CHANNEL);
    #endif

    /* The three voltages are phase coincident. Only interrupt from one of them. */
    #if defined(__MSP430_HAS_SD24_B__)
    SD24BIE = 0x0007;
    #else
        #if NUM_PHASES >= 2
    sd_xxxx_reg(SD_CCTL_, PHASE_1_CURRENT_ADC_CHANNEL) |= SD_IE;
    sd_xxxx_reg(SD_CCTL_, PHASE_2_CURRENT_ADC_CHANNEL) |= SD_IE;
        #endif
        #if NUM_PHASES >= 3
    sd_xxxx_reg(SD_CCTL_, PHASE_3_CURRENT_ADC_CHANNEL) |= SD_IE;
        #endif
    sd_xxxx_reg(SD_CCTL_, PHASE_1_VOLTAGE_ADC_CHANNEL) |= SD_IE;
    #endif

    #if NUM_PHASES > 1
    for (ph = 0;  ph < NUM_PHASES;  ph++)
    #endif
        working_data.phases[ph].metrology.current[0].in_phase_correction.sd_preloaded_offset = 0;
    #if NUM_PHASES == 1  &&  defined(NEUTRAL_MONITOR_SUPPORT)
    working_data.phases[ph].metrology.current[1].in_phase_correction.sd_preloaded_offset = 0;
    #endif
        __set_interrupt_state(interrupt_state);
}
#endif

void metrology_disable_analog_front_end(void)
{
#if NUM_PHASES > 1
    int ph;
#endif
    __istate_t interrupt_state;
    interrupt_state = __get_interrupt_state();
    __disable_interrupt();
    
#if NUM_PHASES > 1  &&  defined(__MSP430_HAS_SD24_B3__)  &&  defined(__MSP430_HAS_ADC10_A__)
   __disable_interrupt();
    ADC10CTL0 &= ~ADC10ENC;
    ADC10CTL1 = 0;
    ADC10CTL0 &= 0;
     DMA1CTL = 0;
#else
    sd_set_idle_voltage_mode(PHASE_1_VOLTAGE_ADC_CHANNEL);
    #if !defined(VOLTAGE_SIGNAL_IS_COMMON)
        #if NUM_PHASES >= 2
    sd_set_idle_voltage_mode(PHASE_2_VOLTAGE_ADC_CHANNEL);
        #endif
        #if NUM_PHASES >= 3
    sd_set_idle_voltage_mode(PHASE_3_VOLTAGE_ADC_CHANNEL);
        #endif
    #endif
#endif

    sd_set_idle_live_current_mode(PHASE_1_CURRENT_ADC_CHANNEL);
#if NUM_PHASES >= 2
    sd_set_idle_live_current_mode(PHASE_2_CURRENT_ADC_CHANNEL);
#endif
#if NUM_PHASES >= 3
    sd_set_idle_live_current_mode(PHASE_3_CURRENT_ADC_CHANNEL);
#endif
#if NUM_PHASES >= 4
    sd_set_idle_live_current_mode(PHASE_4_CURRENT_ADC_CHANNEL);
#endif
#if NUM_PHASES >= 5
    sd_set_idle_live_current_mode(PHASE_5_CURRENT_ADC_CHANNEL);
#endif
#if NUM_PHASES >= 6
    sd_set_idle_live_current_mode(PHASE_6_CURRENT_ADC_CHANNEL);
#endif
#if defined(NEUTRAL_MONITOR_SUPPORT)
    sd_set_idle_live_current_mode(NEUTRAL_CURRENT_ADC_CHANNEL);
#endif

#if defined(__MSP430_HAS_SD24_B__)
    SD24BCTL0 = 0;
#else
    SD_CTL = 0;
#endif

#if NUM_PHASES == 1
    working_data.phases[0].metrology.current[0].in_phase_correction.sd_preloaded_offset = 0;
    #if defined(NEUTRAL_MONITOR_SUPPORT)
    working_data.phases[0].metrology.current[1].in_phase_correction.sd_preloaded_offset = 0;
    #endif
#else
    for (ph = 0;  ph < NUM_PHASES;  ph++)
        working_data.phases[ph].metrology.current[0].in_phase_correction.sd_preloaded_offset = 0;
#endif
    
   __set_interrupt_state(interrupt_state);
}

int metrology_init_from_nv_data(void)
{
#if NUM_PHASES > 1
    int ph;
    struct phase_parms_s *phase;
    struct phase_calibration_data_s const *phase_cal;

    phase = working_data.phases;
    phase_cal = cal_info->phases;
    for (ph = 0;  ph < NUM_PHASES;  ph++, phase++, phase_cal++)
#endif
    {
        /* Prime the DC estimates for quick settling */
#if defined(LIMP_MODE_SUPPORT)
        dc_filter_current_init(phase->metrology.current[0].I_dc_estimate[0], phase_cal->current[0].initial_dc_estimate[0]);
        dc_filter_current_init(phase->metrology.current[0].I_dc_estimate[1], phase_cal->current[0].initial_dc_estimate[1]);
        phase->metrology.current[0].I_endstops = ENDSTOP_HITS_FOR_OVERLOAD;
    #if NUM_PHASES == 1  &&  defined(NEUTRAL_MONITOR_SUPPORT)
        dc_filter_current_init(phase->metrology.current[1].I_dc_estimate[0], phase_cal->current[1].initial_dc_estimate[0]);
        dc_filter_current_init(phase->metrology.current[1].I_dc_estimate[1], phase_cal->current[1].initial_dc_estimate[1]);
        phase->metrology.current[1].I_endstops = ENDSTOP_HITS_FOR_OVERLOAD;
    #endif
        dc_filter_voltage_init(phase->metrology.V_dc_estimate[0], phase_cal->initial_v_dc_estimate[0]);
        dc_filter_voltage_init(phase->metrology.V_dc_estimate[1], phase_cal->initial_v_dc_estimate[1]);
#else
        dc_filter_current_init(phase->metrology.current[0].I_dc_estimate[0], phase_cal->current[0].initial_dc_estimate[0]);
        phase->metrology.current[0].I_endstops = ENDSTOP_HITS_FOR_OVERLOAD;
    #if NUM_PHASES == 1  &&  defined(NEUTRAL_MONITOR_SUPPORT)
        dc_filter_current_init(phase->metrology.current[1].I_dc_estimate[0], phase_cal->current[1].initial_dc_estimate[0]);
        phase->metrology.current[1].I_endstops = ENDSTOP_HITS_FOR_OVERLOAD;
    #endif
        dc_filter_voltage_init(phase->metrology.V_dc_estimate[0], phase_cal->initial_v_dc_estimate[0]);
#endif
        phase->metrology.V_endstops = ENDSTOP_HITS_FOR_OVERLOAD;
#if defined(MAINS_FREQUENCY_SUPPORT)
    #if defined(LIMP_MODE_SUPPORT)
        #if NUM_PHASES == 1  &&  defined(NEUTRAL_MONITOR_SUPPORT)
        phase->metrology.current[1].period.period =
        #endif
        phase->metrology.current[0].period.period =
    #endif
        phase->metrology.voltage_period.period = ((SAMPLES_PER_10_SECONDS*6554)/MAINS_NOMINAL_FREQUENCY) << 8;
#endif
    }
#if NUM_PHASES > 1  &&  defined(NEUTRAL_MONITOR_SUPPORT)
    dc_filter_current_init(working_data.neutral.metrology.I_dc_estimate[0], cal_info->neutral.initial_dc_estimate[0]);
    working_data.neutral.metrology.I_endstops = ENDSTOP_HITS_FOR_OVERLOAD;
#endif
    return 0;
}

int metrology_align_with_nv_data(void)
{
#if NUM_PHASES > 1
    int ph;
    static struct phase_parms_s *phase;
    static struct phase_calibration_data_s const *phase_cal;
#endif

    metrology_disable_analog_front_end();
    metrology_init_analog_front_end_normal_mode();

#if NUM_PHASES > 1
    phase = working_data.phases;
    phase_cal = cal_info->phases;
    for (ph = 0;  ph < NUM_PHASES;  ph++, phase++, phase_cal++)
#endif
    {
        set_sd_phase_correction(&phase->metrology.current[0].in_phase_correction, 0, phase_cal->current[0].phase_correction);
#if NUM_PHASES == 1  &&  defined(NEUTRAL_MONITOR_SUPPORT)
        set_sd_phase_correction(&phase->metrology.current[1].in_phase_correction, 1, phase_cal->current[1].phase_correction);
#endif
    }
#if defined(SAG_POWER_DOWN_SUPPORT)
    sag_power_down_control();
#endif
    return 0;
}

void metrology_switch_to_normal_mode(void)
{
#if NUM_PHASES > 1
    #if !defined(DYNAMIC_CURRENT_RELATED_CORRECTION_SUPPORT)  &&  !defined(DYNAMIC_FREQUENCY_RELATED_CORRECTION_SUPPORT)
    int ph;
    struct phase_parms_s *phase;
    struct phase_calibration_data_s const *phase_cal;
    #endif
#endif

    /* Switch to full speed, full power mode */
    metrology_state |= METROLOGY_STATUS_PHASE_VOLTAGE_OK;

#if defined(__HAS_SD_ADC__)
    #if defined(__MSP430_HAS_TA3__)
    /* Disable the TIMER_A0 interrupt */
    TACTL = 0;
    TACCTL0 = 0;
    #endif
    metrology_init_analog_front_end_normal_mode();
#endif
    samples_per_second = SAMPLES_PER_10_SECONDS/10;

#if !defined(DYNAMIC_CURRENT_RELATED_CORRECTION_SUPPORT)  &&  !defined(DYNAMIC_FREQUENCY_RELATED_CORRECTION_SUPPORT)
    #if NUM_PHASES > 1
    phase = working_data.phases;
    phase_cal = cal_info->phases;
    for (ph = 0;  ph < NUM_PHASES;  ph++, phase++, phase_cal++)
    #endif
    {
    #if defined(__HAS_SD_ADC__)
        set_sd_phase_correction(&phase->metrology.current[0].in_phase_correction, ph, phase_cal->current[0].phase_correction);
        #if NUM_PHASES == 1  &&  defined(NEUTRAL_MONITOR_SUPPORT)
        set_sd_phase_correction(&phase->metrology.current[1].in_phase_correction, NUM_PHASES, phase_cal->current[1].phase_correction);
        #endif
    #else
        set_phase_correction(&phase->metrology.current[0].in_phase_correction, phase_cal->current[0].phase_correction);
        #if NUM_PHASES == 1  &&  defined(NEUTRAL_MONITOR_SUPPORT)
        set_phase_correction(&phase->metrology.current[1].in_phase_correction, phase_cal->current[1]phase_correction);
        #endif
    #endif
    }
    #if defined(__HAS_SD_ADC__)
        #if NUM_PHASES > 1  &&  defined(NEUTRAL_MONITOR_SUPPORT)
    set_sd_phase_correction(&working_data.neutral.metrology.in_phase_correction, NUM_PHASES, cal_info->neutral.phase_correction);
        #endif
    #endif
#endif
    switch_to_normal_mode();
    operating_mode = OPERATING_MODE_NORMAL;
}

#if defined(LIMP_MODE_SUPPORT)
void metrology_switch_to_limp_mode(void)
{
    /* Switch to minimum consumption, current measurement only mode */
    metrology_state &= ~(METROLOGY_STATUS_REVERSED | METROLOGY_STATUS_PHASE_VOLTAGE_OK);

    #if defined(__HAS_SD_ADC__)
        #if defined(__MSP430_HAS_TA3__)
    /* Enable the TIMER_A0 interrupt */
    TACTL = TACLR | MC_1 | TASSEL_1;
    TACCTL0 = CCIE;
        #endif
    metrology_init_analog_front_end_limp_mode();
    #endif

    samples_per_second = LIMP_SAMPLES_PER_10_SECONDS/10;

    switch_to_limp_mode();
    operating_mode = OPERATING_MODE_LIMP;
}
#endif

#if defined(POWER_DOWN_SUPPORT)
void metrology_switch_to_powerfail_mode(void)
{
    operating_mode = OPERATING_MODE_POWERFAIL;
    #if defined(LIMP_MODE_SUPPORT)
    normal_limp = 0;
    #endif

    /* Note that a power down occured */
    metrology_state |= POWER_DOWN;

    metrology_state &= ~(STATUS_REVERSED | STATUS_EARTHED | STATUS_PHASE_VOLTAGE_OK);
    /* Turn off all the LEDs. */
    #if defined(TOTAL_ACTIVE_ENERGY_SUPPORT)
    total_active_energy_pulse_end();
    #endif
    #if defined(TOTAL_REACTIVE_ENERGY_SUPPORT)
    total_reactive_energy_pulse_end();
    #endif

    /* Make the EEPROM signals inputs, and rely on pullups. */
    disable_eeprom_port();

    /* Shut down the LCD */
    custom_lcd_sleep_handler();

    #if defined(__MSP430_HAS_TA3__)  &&  defined(__MSP430_SD_ADC__)
    /* Disable the TIMER_A0 interrupt */
    TACTL = 0;
    TACCTL0 = 0;
    /* Disable the interrupt routine which re-enables the ADC */
    TACCTL2 = 0;
    #endif

    #if defined(__HAS_SD_ADC__)
    metrology_disable_analog_front_end();
    #endif
    #if defined(IEC1107_SUPPORT)  ||  defined(SERIAL_CALIBRATION_SUPPORT)  ||  defined(SERIAL_CALIBRATION_REF_SUPPORT)
    /* Disable the serial port. */
    U0ME &= ~(UTXE0 | URXE0);
        #if defined(IEC1107_SUPPORT)
    disable_ir_receiver();
        #endif
    #endif

    #if defined(BATTERY_MONITOR_SUPPORT)
    /* Battery sensing control pin */
    P3DIR &= ~(BIT1);
    P3OUT |= (BIT1);
    #endif

    #if defined(__MSP430_HAS_FLLPLUS__)  ||  defined(__MSP430_HAS_FLLPLUS_SMALL__)
    /* Slow the clock to 1MHz as quickly as possible. The FLL will not be active
       in LPM3, so switch it off now, and force the FLL's RC oscillator to
       about 1MHz. The exact frequency is not critical. */
    _BIS_SR(SCG0);                  /* switch off FLL locking */
    SCFI0 = FLLD_1;
    SCFQCTL = SCFI0_LOW | SCFQ_M;
    SCFI0 = 0x00;
    SCFI1 = 0x37;
    #endif
    #if defined(__MSP430_HAS_SVS__)
    /* At 1MHz it is safe to turn off the SVS, and rely on the brownout
       detector. Now the meter can survive on a very weak battery. */
    SVSCTL = 0;
    #endif

    /* ******************** LOW POWER STATE ************************** */
    /* Go to LPM3 mode and exit only when power comes back on. The timer
       interrupt that ticks every second should be checking for power
       restored while we sit here. When it sees the unregulated supply
       at a healthy voltage, it will wake us up. */
    _BIS_SR(LPM3_bits);

    /* Waking up from power down mode */
    #if defined(__MSP430_HAS_SVS__)
    /* Before we go to high speed we need to make sure the supply voltage is 
       adequate. If there is an SVS we can use that. There should be no wait
       at this point, since we should only have been woken up if the supply
       is healthy. However, it seems better to be cautious. */
    SVSCTL |= 0x60;
    /* Wait for adequate voltage to run at full speed */
    while (!(SVSCTL & SVSON))
        /* dummy loop */;
    while ((SVSCTL & SVSOP))
        /* dummy loop */;
    /* The voltage should now be OK to run the CPU at full speed. Now it should
       be OK to use the SVS as a reset source. */
    SVSCTL |= PORON;
    #endif

    #if defined(__MSP430_HAS_FLLPLUS__)  ||  defined(__MSP430_HAS_FLLPLUS_SMALL__)
    /* Speed up the clock to high speed. */
    SCFI0 = FN_3 | FLLD_4;
    SCFQCTL = SCFQCTL_HIGH;
    /* There seems no good reason to wait until the FLL has settled at this point. */
    #endif

    /* Take control of the EEPROM signals again. */
    enable_eeprom_port();

    /* Enable the serial port */
    #if defined(IEC1107_SUPPORT)  ||  defined(SERIAL_CALIBRATION_SUPPORT)  ||  defined(SERIAL_CALIBRATION_REF_SUPPORT)
        #if defined(SERIAL_CALIBRATION_REF_SUPPORT)
    U0ME |= (UTXE0 | URXE0);
        #elif defined(SERIAL_CALIBRATION_SUPPORT)
    U0ME |= URXE0;
        #else
    U0ME |= UTXE0;
        #endif
    #endif

    #if defined(__MSP430_HAS_TA3__)  &&  defined(__HAS_SD_ADC__)
    /* Enable the TIMER_A0 interrupt */
    TACTL = TACLR | MC_1 | TASSEL_1;
    TACCTL0 = CCIE;
    #endif

    kick_watchdog();
    #if defined(LOSE_FRACTIONAL_PULSE_AT_POWER_ON)
        #if defined(TOTAL_ACTIVE_ENERGY_PULSES_PER_KW_HOUR)
    totals.energy.active_energy_pulse.energy_integrator = 0;
        #endif
        #if defined(TOTAL_REACTIVE_ENERGY_PULSES_PER_KVAR_HOUR)
    totals.energy.reactive_energy_pulse.energy_integrator = 0;
        #endif
    #endif

    /* Come out of power down in limp mode, as we don't know
       if there is sufficent power available to driver the meter
       at full speed. We will soon switch to normal mode if a
       voltage signal is available. */
    /* Limp mode will fire up the ADC again. */
    #if defined(LIMP_MODE_SUPPORT)
    switch_to_limp_mode();
    #else
    switch_to_normal_mode();
    #endif
}
#endif

#if defined(__MSP430_HAS_TLV__)
uint8_t *tlv_find(int tag)
{
    uint8_t *ptr;
    int sum;

    /* First check if the TLV table is OK */
    sum = 0;
    for (ptr = (uint8_t *) (TLV_START + 2);  ptr <= (uint8_t *) TLV_END;  ptr++)
    {
        sum += *ptr;
    }
    if (*((uint16_t *) TLV_START) != -sum)
        return NULL;
    for (ptr = (uint8_t *) (TLV_START + 2);  ptr <= (uint8_t *) TLV_END;  ptr++)
    {
        if (*ptr == tag)
            return ptr;
        ptr++;
        ptr += (*ptr + 1);
    }
    return NULL;
}
#endif

int metrology_init(void)
{
#if defined(__MSP430_HAS_ESP430E__)
    /* Make this compatible with devices containing the ESP module, by just turning it off. */
    ESPCTL &= ~ESPEN;
#endif
    /* There should always be a power scaling factor for the first current sensor channel of the
       first phase, whatever type of metrology is being built, and that factor should not be
       0xFFFF. */
    if (cal_info->phases[0].current[0].P_scale_factor == 0xFFFF)
    {
        flash_memcpy((void *) cal_info, (const void *) &calibration_defaults, sizeof(calibration_defaults));
        flash_secure();
    }
    if(cfg_info->baud_rate == 0xFFFF)
    {
        flash_memcpy((void *) cfg_info, (const void *) &configuration_defaults, sizeof(configuration_defaults));
        flash_secure();
    }
#if defined(SAG_POWER_DOWN_SUPPORT)
    sag_power_down_control();
#endif
    metrology_state = 0;
    return 0;
}
