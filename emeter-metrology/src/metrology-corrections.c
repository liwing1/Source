/*******************************************************************************
 *  emeter-corrections.c -
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

/*! \file emeter-structs.h */
//
//  MSP430 e-meter dynamic phase correction parameters. These are modified during
//  calibration, so they must be on a separate page/pages of flash memory from
//  the program itself, so the program is not corrupted when the flash is erased.
//
#include <inttypes.h>
#include <msp430.h>

#include <emeter-toolkit.h>

#include "emeter-metrology.h"

#include "emeter-metrology-internal.h"

#if defined(DYNAMIC_FREQUENCY_RELATED_CORRECTION_SUPPORT)
    #if NUM_PHASES == 1
        #if defined(NEUTRAL_MONITOR_SUPPORT)
void dynamic_frequency_related_correction(int ch)
        #else
void dynamic_frequency_related_correction(void)
        #endif
    #else
void dynamic_frequency_related_correction(struct phase_parms_s *phase, struct phase_calibration_data_s const *phase_cal, int ph)
    #endif
{
    int32_t z;
    int16_t i;

    /* Adjust the phase shift to allow for the current mains frequency. This is just a linear
       approximation, but should be good enough over the frequency ranges we must cover. */
    i = phase_cal->current[ch].phase_correction;
    if (phase_cal->current[ch].frequency_phase_correction)
    {
        /* The frequency correction is phase-shift dependant, and is proportional to the square of
           the phase-shift. */
        z = (int32_t) i*i;
        z *= ((int) phase->readings.frequency - MAINS_NOMINAL_FREQUENCY*FREQUENCY_SCALING_FACTOR);
        z = z*phase_cal->current[ch].frequency_phase_correction;
        z >>= 16;
        /* Half bit round */
        z++;
        i -= z;
    }
    /* Update the whole group quickly, so the interrupt routine uses a consistent pair of
       step and beta values. */
    #if defined(__HAS_SD_ADC__)
        #if NUM_PHASES == 1
    set_sd_phase_correction(&phase->metrology.current[ch].in_phase_correction, ch, i);
        #else
    set_sd_phase_correction(&phase->metrology.current[ch].in_phase_correction, ph, i);
        #endif
    #endif
}
#endif

#if defined(DYNAMIC_CURRENT_RELATED_CORRECTION_SUPPORT)

#define CURRENT_CORRECTION_STEP_SIZE_1      100
#define CURRENT_CORRECTION_STEPS_1          50
#define CURRENT_CORRECTION_STEP_SIZE_2      1000
#define CURRENT_CORRECTION_STEPS_2          20
#define CURRENT_CORRECTION_STEP_SIZE_3      5000
#define CURRENT_CORRECTION_STEPS_3          6

#if NUM_PHASES == 1
    #if !defined(NEUTRAL_MONITOR_SUPPORT)
#define TAB_DEFS DEFAULT_PHASE_CORRECTION_1, DEFAULT_GAIN_CORRECTION_1
    #else
#define TAB_DEFS {DEFAULT_PHASE_CORRECTION_1, DEFAULT_GAIN_CORRECTION_1}, {DEFAULT_PHASE_CORRECTION_N2, DEFAULT_GAIN_CORRECTION_N}
    #endif
#elif NUM_PHASES == 2
    #if !defined(NEUTRAL_MONITOR_SUPPORT)
#define TAB_DEFS {DEFAULT_PHASE_CORRECTION_1, DEFAULT_GAIN_CORRECTION_1}, {DEFAULT_PHASE_CORRECTION_2, DEFAULT_GAIN_CORRECTION_2}, 
    #else
#define TAB_DEFS {DEFAULT_PHASE_CORRECTION_1, DEFAULT_GAIN_CORRECTION_1}, {DEFAULT_PHASE_CORRECTION_2, DEFAULT_GAIN_CORRECTION_2}, {DEFAULT_PHASE_CORRECTION_N, DEFAULT_GAIN_CORRECTION_N}
    #endif
#elif NUM_PHASES == 3
    #if !defined(NEUTRAL_MONITOR_SUPPORT)
#define TAB_DEFS {DEFAULT_PHASE_CORRECTION_1, DEFAULT_GAIN_CORRECTION_1}, {DEFAULT_PHASE_CORRECTION_2, DEFAULT_GAIN_CORRECTION_2}, {DEFAULT_PHASE_CORRECTION_3, DEFAULT_GAIN_CORRECTION_3}
    #else
#define TAB_DEFS {DEFAULT_PHASE_CORRECTION_1, DEFAULT_GAIN_CORRECTION_1}, {DEFAULT_PHASE_CORRECTION_2, DEFAULT_GAIN_CORRECTION_2}, {DEFAULT_PHASE_CORRECTION_3, DEFAULT_GAIN_CORRECTION_3}, {DEFAULT_PHASE_CORRECTION_N, DEFAULT_GAIN_CORRECTION_N}
    #endif
#endif

static const uint8_t steps[] =
{
#if defined(CURRENT_CORRECTION_STEP_SIZE_1)
    CURRENT_CORRECTION_STEPS_1,
#endif
#if defined(CURRENT_CORRECTION_STEP_SIZE_2)
    CURRENT_CORRECTION_STEPS_2,
#endif
#if defined(CURRENT_CORRECTION_STEP_SIZE_3)
    CURRENT_CORRECTION_STEPS_3,
#endif
#if defined(CURRENT_CORRECTION_STEP_SIZE_4)
    CURRENT_CORRECTION_STEPS_4,
#endif
#if defined(CURRENT_CORRECTION_STEP_SIZE_5)
    CURRENT_CORRECTION_STEPS_5,
#endif
    0xFF
};

static const int step_size[] =
{
#if defined(CURRENT_CORRECTION_STEP_SIZE_1)
    CURRENT_CORRECTION_STEP_SIZE_1,
#endif
#if defined(CURRENT_CORRECTION_STEP_SIZE_2)
    CURRENT_CORRECTION_STEP_SIZE_2,
#endif
#if defined(CURRENT_CORRECTION_STEP_SIZE_3)
    CURRENT_CORRECTION_STEP_SIZE_3,
#endif
#if defined(CURRENT_CORRECTION_STEP_SIZE_4)
    CURRENT_CORRECTION_STEP_SIZE_4,
#endif
#if defined(CURRENT_CORRECTION_STEP_SIZE_5)
    CURRENT_CORRECTION_STEP_SIZE_5,
#endif
    0xFF
};

static const current_t limits[] =
{
    0,
#if defined(CURRENT_CORRECTION_STEP_SIZE_1)
    CURRENT_CORRECTION_STEP_SIZE_1*CURRENT_CORRECTION_STEPS_1,
#endif
#if defined(CURRENT_CORRECTION_STEP_SIZE_2)
    CURRENT_CORRECTION_STEP_SIZE_1*CURRENT_CORRECTION_STEPS_1 + CURRENT_CORRECTION_STEP_SIZE_2*CURRENT_CORRECTION_STEPS_2,
#endif
#if defined(CURRENT_CORRECTION_STEP_SIZE_3)
    CURRENT_CORRECTION_STEP_SIZE_1*CURRENT_CORRECTION_STEPS_1 + CURRENT_CORRECTION_STEP_SIZE_2*CURRENT_CORRECTION_STEPS_2 + CURRENT_CORRECTION_STEP_SIZE_3*CURRENT_CORRECTION_STEPS_3,
#endif
#if defined(CURRENT_CORRECTION_STEP_SIZE_4)
    CURRENT_CORRECTION_STEP_SIZE_1*CURRENT_CORRECTION_STEPS_1 + CURRENT_CORRECTION_STEP_SIZE_2*CURRENT_CORRECTION_STEPS_2 + CURRENT_CORRECTION_STEP_SIZE_3*CURRENT_CORRECTION_STEPS_3 + CURRENT_CORRECTION_STEP_SIZE_4*CURRENT_CORRECTION_STEPS_4,
#endif
#if defined(CURRENT_CORRECTION_STEP_SIZE_5)
    CURRENT_CORRECTION_STEP_SIZE_1*CURRENT_CORRECTION_STEPS_1 + CURRENT_CORRECTION_STEP_SIZE_2*CURRENT_CORRECTION_STEPS_2 + CURRENT_CORRECTION_STEP_SIZE_3*CURRENT_CORRECTION_STEPS_3 + CURRENT_CORRECTION_STEP_SIZE_4*CURRENT_CORRECTION_STEPS_4 + CURRENT_CORRECTION_STEP_SIZE_5*CURRENT_CORRECTION_STEPS_5,
#endif
};

/*! This table, in a special section of flash memory, is constructed during the calibration of the
    meter. It defines the current dependant phase shift and gain characteristics of the CTs. The 77
    steps are based on a 60A meter, as follows:
    - 0.1A steps from 0.0A to 5.0A
    - 1A steps from 5.0A to 25.0A
    - 5A steps from 25.0A to 55.0A
    All currents above 55A use the 55A entry. This gives a total of 50 + 20 + 7 => 77 entries. Each entry
    consists of an 8 bit phase shift correction, and an 8 bit gain correction. */
#if defined(CALIBRATED_DYNAMIC_CURRENT_RELATED_CORRECTION_SUPPORT)
const int8_t __erasablemem__ phase_corrections[77][NUM_CURRENT_CHANNELS][2] =
#else
const int8_t phase_corrections[77][NUM_CURRENT_CHANNELS][2] =
#endif
{
    {TAB_DEFS},  // 0.00A
    {TAB_DEFS},  // 0.10A
    {TAB_DEFS},  // 0.20A
    {TAB_DEFS},  // 0.30A
    {TAB_DEFS},  // 0.40A
    {TAB_DEFS},  // 0.50A
    {TAB_DEFS},  // 0.60A
    {TAB_DEFS},  // 0.70A
    {TAB_DEFS},  // 0.80A
    {TAB_DEFS},  // 0.90A
    {TAB_DEFS},  // 1.00A
    {TAB_DEFS},  // 1.10A
    {TAB_DEFS},  // 1.20A
    {TAB_DEFS},  // 1.30A
    {TAB_DEFS},  // 1.40A
    {TAB_DEFS},  // 1.50A
    {TAB_DEFS},  // 1.60A
    {TAB_DEFS},  // 1.70A
    {TAB_DEFS},  // 1.80A
    {TAB_DEFS},  // 1.90A
    {TAB_DEFS},  // 2.00A
    {TAB_DEFS},  // 2.10A
    {TAB_DEFS},  // 2.20A
    {TAB_DEFS},  // 2.30A
    {TAB_DEFS},  // 2.40A
    {TAB_DEFS},  // 2.50A
    {TAB_DEFS},  // 2.60A
    {TAB_DEFS},  // 2.70A
    {TAB_DEFS},  // 2.80A
    {TAB_DEFS},  // 2.90A
    {TAB_DEFS},  // 3.00A
    {TAB_DEFS},  // 3.10A
    {TAB_DEFS},  // 3.20A
    {TAB_DEFS},  // 3.30A
    {TAB_DEFS},  // 3.40A
    {TAB_DEFS},  // 3.50A
    {TAB_DEFS},  // 3.60A
    {TAB_DEFS},  // 3.70A
    {TAB_DEFS},  // 3.80A
    {TAB_DEFS},  // 3.90A
    {TAB_DEFS},  // 4.00A
    {TAB_DEFS},  // 4.10A
    {TAB_DEFS},  // 4.20A
    {TAB_DEFS},  // 4.30A
    {TAB_DEFS},  // 4.40A
    {TAB_DEFS},  // 4.50A
    {TAB_DEFS},  // 4.60A
    {TAB_DEFS},  // 4.70A
    {TAB_DEFS},  // 4.80A
    {TAB_DEFS},  // 4.90A
    {TAB_DEFS},  // 5.00A    new step
    {TAB_DEFS},  // 6.00A
    {TAB_DEFS},  // 7.00A
    {TAB_DEFS},  // 8.00A
    {TAB_DEFS},  // 9.00A
    {TAB_DEFS},  // 10.00A
    {TAB_DEFS},  // 11.00A
    {TAB_DEFS},  // 12.00A
    {TAB_DEFS},  // 13.00A
    {TAB_DEFS},  // 14.00A
    {TAB_DEFS},  // 15.00A
    {TAB_DEFS},  // 16.00A
    {TAB_DEFS},  // 17.00A
    {TAB_DEFS},  // 18.00A
    {TAB_DEFS},  // 19.00A
    {TAB_DEFS},  // 20.00A
    {TAB_DEFS},  // 21.00A
    {TAB_DEFS},  // 22.00A
    {TAB_DEFS},  // 23.00A
    {TAB_DEFS},  // 24.00A
    {TAB_DEFS},  // 25.00A    new step
    {TAB_DEFS},  // 30.00A
    {TAB_DEFS},  // 35.00A
    {TAB_DEFS},  // 40.00A
    {TAB_DEFS},  // 45.00A
    {TAB_DEFS},  // 50.00A
    {TAB_DEFS}   // 55.00A
};

    #if NUM_PHASES == 1
        #if defined(NEUTRAL_MONITOR_SUPPORT)
void dynamic_current_related_phase_correction(int ch)
        #else
void dynamic_current_related_phase_correction(void)
        #endif
    #else
void dynamic_current_related_phase_correction(struct phase_parms_s *phase, struct phase_calibration_data_s const *phase_cal, int ph)
    #endif
{
    int16_t i;
    int16_t j;
    int16_t k;
    int32_t z;
    int16_t phase_corr;
    int16_t gain;
    rms_current_t x;

    #if NUM_PHASES == 1  &&  defined(NEUTRAL_MONITOR_SUPPORT)
    x = phase->metrology.current[ch].readings.I_rms;
    if ((phase->status & current_overrange_masks[ch]))
    #else
        #if NUM_PHASES > 1  &&  defined(NEUTRAL_MONITOR_SUPPORT)
    if (ph == FAKE_PHASE_NEUTRAL)
        x = working_data.neutral.readings.I_rms;
    else
        x = phase->readings.I_rms;
        #else
    x = phase->readings.I_rms;
        #endif
    if ((phase->status & PHASE_STATUS_I_OVERRANGE))
    #endif
    {
        phase_corr = phase_cal->current[0].phase_correction;
        gain = 0;
    }
    else
    {
        if (x < 0)
        {
            /* This should not happen, but just in case... */
            phase_corr = phase_cal->current[0].phase_correction;
            gain = 0;
        }
        else
        {
            /* Find the right section of the correction table */
            for (step = 0, i = 0;  steps[i] != 0xFF;  i++)
            {
                if (x < limits[i + 1])
                    break;
                step += steps[i];
            }
            if (steps[i] == 0xFF)
            {
                phase_corr = phase_cal->current[0].phase_correction + phase[step][0];
                gain = 0;
            }
            else
            {
                y = x - limits[i];
                j = y%step_size[i];
                k = y/step_size[i] + step;
                phase_corr = phase_cal->current[0].phase_correction + phase_corrections[k][ph + ch][0] + (phase_corrections[k + 1][ph + ch][0] - phase_corrections[k][ph + ch][0])*j/step_size[i];
                gain = phase_corrections[k][ph + ch][1] + (phase_corrections[k + 1][ph + ch][1] - phase_corrections[k][ph + ch][1])*j/step_size[i];
            }
        }
    }
    /* Adjust the phase shift to allow for the current mains frequency. This is just a linear
       approximation, but should be good enough over the frequency ranges we must cover. */
    if (phase_cal->current[ch].frequency_phase_correction)
    {
        /* The frequency correction is phase-shift dependant, and is proportional to the square of
           the phase-shift. */
        z = (int32_t) phase_corr*phase_corr;
        z *= ((int) phase->readings.frequency - MAINS_NOMINAL_FREQUENCY*100);
        z = z*phase_cal->current[ch].frequency_phase_correction;
        z >>= 16;
        /* Half bit round */
        z++;
        phase_corr -= z;
    }
    gain = 32767 - (gain << 2);
    /* Update the whole group quickly, so the interrupt routine uses a consistent pair of
       step and beta values. */
    #if defined(__HAS_SD_ADC__)
        #if NUM_PHASES == 1
    set_sd_phase_correction(&phase->metrology.current[ch].in_phase_correction, ch, phase_corr);
        #else
    set_sd_phase_correction(&phase->metrology.current[ch].in_phase_correction, ph, phase_corr);
        #endif
    #endif
}
#endif
