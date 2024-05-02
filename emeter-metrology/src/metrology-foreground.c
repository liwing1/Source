/*******************************************************************************
 *  metrology-foreground.c -
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

/*! \file emeter-metrology.h */

#include <inttypes.h>
#include <stdlib.h>
#include <stdbool.h>
#if !defined(__MSP430__)
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#endif
#if defined(__GNUC__)
#include <signal.h>
#endif
#include <string.h>

#include <emeter-toolkit.h>

#include "emeter-metrology.h"

#include "emeter-metrology-internal.h"

// TDTDTD
#define PHASE_REVERSED_THRESHOLD_POWER 1
#define PHASE_REVERSED_PERSISTENCE_CHECK 1

/* Meter status flag bits. */
uint16_t metrology_state;


/* Current operating mode - normal, limp, power down, etc. */
int8_t operating_mode;
#if defined(LIMP_MODE_SUPPORT)
int normal_limp;
#endif

#if defined(TEMPERATURE_SUPPORT)
int16_t temperature_in_celsius;
#endif

/* The main per-phase working parameter structure */
struct metrology_data_s working_data;

static __inline__ int32_t abs32(int32_t x)
{
    return (x < 0)  ?  -x  :  x;
}

void set_phase_correction(struct phase_correction_s *s, int correction)
{
    /* Add half a sample, as the zero shift point is half way through the phase shift table */
    correction += 128;
    s->step = I_HISTORY_STEPS + (correction >> 8);
#if PHASE_SHIFT_FIR_TABLE_ELEMENTS == 128
    correction = (PHASE_SHIFT_FIR_TABLE_ELEMENTS - 1) - ((correction & 0xFF) >> 1);
#elif PHASE_SHIFT_FIR_TABLE_ELEMENTS == 256
    correction = (PHASE_SHIFT_FIR_TABLE_ELEMENTS - 1) - (correction & 0xFF);
#else
#error This code needs changing for a FIR table which is not 128 or 256 entries long
#endif
    s->fir_beta = phase_shift_fir_coeffs[correction][0];
    s->fir_gain = phase_shift_fir_coeffs[correction][1];
}

#if defined(DYNAMIC_CURRENT_RELATED_CORRECTION_SUPPORT)  ||  defined(DYNAMIC_FREQUENCY_RELATED_CORRECTION_SUPPORT)
static void set_phase_gain_correction(struct phase_correction_s *s, int correction, int gain)
{
    /* Add half a sample, as the zero shift point is half way through the phase shift table */
    correction += 128;
    s->step = I_HISTORY_STEPS + (correction >> 8);
    #if PHASE_SHIFT_FIR_TABLE_ELEMENTS == 128
    correction = (PHASE_SHIFT_FIR_TABLE_ELEMENTS - 1) - ((correction & 0xFF) >> 1);
    #elif PHASE_SHIFT_FIR_TABLE_ELEMENTS == 256
    correction = (PHASE_SHIFT_FIR_TABLE_ELEMENTS - 1) - (correction & 0xFF);
    #else
#error This code needs changing for a FIR table which is not 128 or 256 entries long
    #endif
    s->fir_beta = phase_shift_fir_coeffs[correction][0];
    s->fir_gain = q1_15_mul(gain, phase_shift_fir_coeffs[correction][1]);
}
#endif

#if defined(__HAS_SD_ADC__)
void set_sd_phase_correction(struct phase_correction_sd_s *s, int phx, int correction)
{
    #if defined(__MSP430_HAS_SD24_B__)
    uint16_t bump;
        #if defined(__IAR_SYSTEMS_ICC__)
    static unsigned short int volatile * const sd_locations[NUM_CURRENT_CHANNELS] =
        #else
    static unsigned int volatile * const sd_locations[NUM_CURRENT_CHANNELS] =
        #endif
    #else
    uint8_t bump;
    static unsigned char volatile * const sd_locations[NUM_CURRENT_CHANNELS] =
    #endif
    {
        &sd_xxxx_reg(SD_PRE_, PHASE_1_CURRENT_ADC_CHANNEL),
    #if NUM_PHASES >= 2
        &sd_xxxx_reg(SD_PRE_, PHASE_2_CURRENT_ADC_CHANNEL),
    #endif
    #if NUM_PHASES >= 3
        &sd_xxxx_reg(SD_PRE_, PHASE_3_CURRENT_ADC_CHANNEL),
    #endif
    #if NUM_PHASES >= 4
        &sd_xxxx_reg(SD_PRE_, PHASE_4_CURRENT_ADC_CHANNEL),
    #endif
    #if NUM_PHASES >= 5
        &sd_xxxx_reg(SD_PRE_, PHASE_5_CURRENT_ADC_CHANNEL),
    #endif
    #if NUM_PHASES >= 6
        &sd_xxxx_reg(SD_PRE_, PHASE_6_CURRENT_ADC_CHANNEL),
    #endif
    #if defined(NEUTRAL_MONITOR_SUPPORT)
        &sd_xxxx_reg(SD_PRE_, NEUTRAL_CURRENT_ADC_CHANNEL),
    #endif
    };

    /* Only try to nudge the converter's timing when in normal operating mode. */
    if (operating_mode == OPERATING_MODE_NORMAL)
    {
        if ((bump = (s->sd_preloaded_offset - correction) & 0xFF))
            *sd_locations[phx] = bump;
    }
    /* Always store the required correction. */
    s->step = I_HISTORY_STEPS - (correction >> 8);
    s->sd_preloaded_offset = correction & 0xFF;
}
#endif

#if defined(MAINS_FREQUENCY_SUPPORT)
    #if NUM_PHASES == 1
static int16_t evaluate_mains_frequency(void)
    #else
static int16_t evaluate_mains_frequency(struct phase_parms_s *phase, struct phase_calibration_data_s const *phase_cal)
    #endif
{
    int32_t x;
    #if NUM_PHASES == 1  &&  defined(NEUTRAL_MONITOR_SUPPORT)  &&  defined(LIMP_MODE_SUPPORT)
    int ch;
    #endif

    /* Calculate the mains frequency in 1/100Hz increments, based on the mains
       period assessment from the background activity. */

    #if defined(LIMP_MODE_SUPPORT)
    if (operating_mode == OPERATING_MODE_LIMP)
    {
        /* In limp mode there is no voltage waveform, so we get the frequency from
           the current in the active lead. This may fail to measure frequency
           correctly for very low currents, and very distorted current waveforms. */
        #if NUM_PHASES == 1  &&  defined(NEUTRAL_MONITOR_SUPPORT)
            #if defined(ON_UNBALANCED_SELECT_HIGHER_READING)
        ch = (phase->status & PHASE_STATUS_CURRENT_FROM_NEUTRAL)  ?  1  :  0;
            #else
        ch = 0;
            #endif
        #endif

        x = phase->metrology.current[ch].period.period;
    }
    else
    #endif
    {
        /* Normally we get the mains frequency from the voltage. Voltage is always
           present, and is not subject to the same level of distortion as the current
           waveform with difficult loads. */
    #if defined(REACTIVE_POWER_BY_QUADRATURE_SUPPORT)
        /* We have a whole cycle period in the upper 16 bits, but we want the delay for 90 degrees, so we shift 2
           extra bits for that. */
        x = (phase->metrology.voltage_period.period >> (16 + 2));
        #if defined(__HAS_SD_ADC__)
        set_phase_correction(&phase->metrology.current[0].quadrature_correction, x - (phase_cal->current[0].phase_correction & 0xFF00));
            #if NUM_PHASES == 1  &&  defined(NEUTRAL_MONITOR_SUPPORT)
        set_phase_correction(&phase->metrology.current[1].quadrature_correction, x - (phase_cal->current[1].phase_correction & 0xFF00));
            #endif
        #else
        set_phase_correction(&phase->metrology.current[0].quadrature_correction, x + phase_cal->current[0].phase_correction);
            #if NUM_PHASES == 1  &&  defined(NEUTRAL_MONITOR_SUPPORT)
        set_phase_correction(&phase->metrology.current[1].quadrature_correction, x + phase_cal->current[1].phase_correction);
            #endif
        #endif
    #endif
        x = phase->metrology.voltage_period.period;
    }
    #if defined(FUNDAMENTAL_POWER_SUPPORT)
    phase->metrology.pure_phase_rate = (int64_t) 0x100000000000000LL/(int64_t) x;
    #endif
    x = (int32_t) SAMPLES_PER_10_SECONDS*256L*10L/(x >> 16);
    return x;
}
#endif

#if defined(VRMS_SUPPORT)
    #if NUM_PHASES == 1
static rms_voltage_t evaluate_rms_voltage(void)
    #else
static rms_voltage_t evaluate_rms_voltage(struct phase_parms_s *phase, struct phase_calibration_data_s const *phase_cal)
    #endif
{
    rms_voltage_t x;
    int32_t tmp;
    int dp;

    if ((phase->status & PHASE_STATUS_V_OVERRANGE))
        return RMS_VOLTAGE_OVERRANGE;

    dp = phase->metrology.dp_set;
    /* The accumulated voltage is 16bitsx16bits*(~4096). So its a 43/44 bit number.
       After dividing by (~4096) its a 31/32 bit number.
       After we take the square root of the 32 bit number its a 16.16 bit number. */
    tmp = div_ac_voltage(phase->metrology.dot_prod[dp].V_sq, phase->metrology.dot_prod[dp].sample_count);
    if (tmp < phase_cal->v_ac_offset)
        return 0;
  
    /* The ac_offset removes the effect of the AWGN from the ADC front end. AWGN is orthogonal to everything but a true copy
       of itself. This means means we need to subtract the ac_offset in a "Pythagoras" manner", while still squared. */
    x = isqrt32(tmp - phase_cal->v_ac_offset);
    /* If we multiply the 16.16 bit number by a 15 bit scaling factor we get a 31.16 bit number.
       Dropping the last 10 bits gives us a 21.6 bit number */
    x = mul48u_32_16(x, phase_cal->V_rms_scale_factor[normal_limp]) >> 10;
    #if defined(TEMPERATURE_CORRECTION_SUPPORT)
    x = mul48u_32_16(x, working_data.temperature_correction.amplitude_factor);
    #endif
    return x;
}
#endif

#if defined(FUNDAMENTAL_VRMS_SUPPORT)
    #if NUM_PHASES == 1
static rms_voltage_t evaluate_fundamental_rms_voltage(void)
    #else
static rms_voltage_t evaluate_fundamental_rms_voltage(struct phase_parms_s *phase, struct phase_calibration_data_s const *phase_cal)
    #endif
{
    int16_t i;
    rms_voltage_t x;
    int dp;

    if ((phase->status & PHASE_STATUS_V_OVERRANGE))
        return RMS_VOLTAGE_OVERRANGE;

    dp = phase->metrology.dp_set;
    /* Scale by the voltage gain */
    x = div_ac_voltage(phase->metrology.dot_prod[dp].V_fundamental, phase->metrology.dot_prod[dp].sample_count);
    /* A negative value indicates the voltage correlation is completely unsynced. */
    if (x < 0)
        return 0;

    /* Scale by 1/sqrt(2). Nudge the scaling factor up a little, to avoid losses due to rounding. */
    i = q1_15_mul(phase_cal->V_rms_scale_factor[normal_limp], 23171 + 3);
    /* Scale by the voltage calibration factor */
    x = mul48u_32_16(x, i) >> 8;
    #if defined(TEMPERATURE_CORRECTION_SUPPORT)
    x = mul48u_32_16(x, working_data.temperature_correction.amplitude_factor);
    #endif
    return x;
}
#endif

#if defined(VOLTAGE_THD_SUPPORT)
    #if NUM_PHASES == 1
static thd_t evaluate_voltage_thd(void)
    #else
static thd_t evaluate_voltage_thd(struct phase_parms_s *phase, struct phase_calibration_data_s const *phase_cal)
    #endif
{
    int64_t x;
    int64_t y;
    int64_t z;

#if defined(VOLTAGE_THD_MEASUREMENT_CUTOFF)
    /* Don't evaluate the voltage THD if the voltage is too small, as we can get some REALLY low accuracy answers
       from the weak signals. */
    if (phase->readings.V_rms < VOLTAGE_THD_MEASUREMENT_CUTOFF)
        return 0;
#endif

    /* Avoid silly results if the fundamental appears to exceed the total, either transiently or
       due to rounding issues. */
    if (phase->readings.fundamental_V_rms > phase->readings.V_rms)
        return 0;
    x = (int64_t) phase->readings.fundamental_V_rms*phase->readings.fundamental_V_rms;
    y = (int64_t) phase->readings.V_rms*phase->readings.V_rms;
    /* Prevent tiny errors in x and y from leading to tiny negative values for THD */
    if (x >= y)
        return 0;
    z = y - x;
#if defined(IEC_THD_SUPPORT)
    z = isqrt64(z);
    z /= phase->readings.fundamental_V_rms;
    z *= 10000;
    y = z >> 32;
#else
    z *= 10000;
    z /= x;
    y = z;
#endif
    if (y < 0)
        return 0;
    return y;
}
#endif

#if defined(IRMS_SUPPORT)
    #if NUM_PHASES == 1
static rms_current_t evaluate_rms_current(void)
    #else
static rms_current_t evaluate_rms_current(struct phase_parms_s *phase, struct phase_calibration_data_s const *phase_cal, int ph)
    #endif
{
    rms_current_t x[PER_PHASE_CURRENT_CHANNELS];
    #if defined(TWENTYFOUR_BIT)
    int64_t tmp;
    #else
    int32_t tmp;
    #endif
    #if NUM_PHASES == 1  &&  defined(NEUTRAL_MONITOR_SUPPORT)
    int ch;
    #endif
    #if defined(LIMP_MODE_SUPPORT)  &&  defined(PHASE_UNBALANCED_DETECTION_SUPPORT)
    static const int32_t thresholds[2] =
    {
        PHASE_UNBALANCED_CUTOFF_THRESHOLD_CURRENT,
        PHASE_UNBALANCED_TIGHT_THRESHOLD_CURRENT
    };
    #endif
    int dp;

    dp = phase->metrology.dp_set;
    /* Calculate the RMS current. Return RMS_CURRENT_OVERRANGE for overrange
       (i.e. ADC clip). A side effect of this routine is it updates the dynamic
       phase correction settings, based on the newly calculated current. */
    /* We always have to work out the properly scaled current from both leads, in
       order to work out the FIR coeffs for the next block. */
    #if NUM_PHASES == 1  &&  defined(NEUTRAL_MONITOR_SUPPORT)
    for (ch = 0;  ch < 2;  ch++)
    #endif
    {
    #if NUM_PHASES == 1  &&  defined(NEUTRAL_MONITOR_SUPPORT)
        if ((phase->status & current_overrange_masks[ch]))
    #else
        if ((phase->status & PHASE_STATUS_I_OVERRANGE))
    #endif
        {
            x[ch] = RMS_CURRENT_OVERRANGE;
        }
        else
        {
            tmp = div_ac_current(phase->metrology.current[ch].dot_prod[dp].I_sq, phase->metrology.current[ch].dot_prod[dp].sample_count);
            if (tmp < phase_cal->current[ch].ac_offset)
            {
                x[ch] = 0;
            }
            else
            {
                /* The ac_offset removes the effect of the AWGN from the ADC front end. AWGN is orthogonal to everything but a true copy
                   of itself. This means means we need to subtract the ac_offset in a "Pythagoras" manner", while still squared. */
    #if defined(TWENTYFOUR_BIT)
                x[ch] = isqrt64(tmp - phase_cal->current[ch].ac_offset) >> 26;
    #else
                x[ch] = isqrt32(tmp - phase_cal->current[ch].ac_offset) >> 2;
    #endif
                x[ch] = mul48u_32_16(x[ch], phase_cal->current[ch].I_rms_scale_factor[normal_limp]);
            }
        }
    #if defined(PER_SENSOR_PRECALCULATED_PARAMETER_SUPPORT)
        phase->metrology.current[ch].readings.I_rms = x[ch];
    #endif

    #if defined(DYNAMIC_CURRENT_RELATED_CORRECTION_SUPPORT)
        #if NUM_PHASES == 1
            #if defined(NEUTRAL_MONITOR_SUPPORT)
            dynamic_current_related_correction(ch);
            #else
            dynamic_current_related_correction();
            #endif
        #else
        dynamic_current_related_correction(phase, phase_cal, ph);
        #endif
    #elif defined(DYNAMIC_FREQUENCY_RELATED_CORRECTION_SUPPORT)
        #if NUM_PHASES == 1
            #if defined(NEUTRAL_MONITOR_SUPPORT)
            dynamic_frequency_related_correction(ch);
            #else
            dynamic_frequency_related_correction();
            #endif
        #else
        dynamic_frequency_related_correction(phase, phase_cal, ph);
        #endif
    #endif
    }

    #if defined(LIMP_MODE_SUPPORT)
    if (operating_mode == OPERATING_MODE_LIMP)
    {
        /* We need to work out which is the relevant current to use. */
        #if defined(PHASE_UNBALANCED_DETECTION_SUPPORT)
        x[0] = test_phase_balance(x[0], x[1], thresholds);
        /* In limp mode we have no way to determine if the phase is reversed,
           so just say it is not. */
        phase->status &= ~PHASE_STATUS_REVERSED;
        #endif
    }
    else
    #endif
    {
        /* The power calculation has told us which is the appropriate
           current to use. */
    #if NUM_PHASES == 1  &&  defined(NEUTRAL_MONITOR_SUPPORT)  &&  defined(ON_UNBALANCED_SELECT_HIGHER_READING)
        if ((phase->status & PHASE_STATUS_CURRENT_FROM_NEUTRAL))
            x[0] = x[1];
    #endif
    }
    #if defined(TEMPERATURE_CORRECTION_SUPPORT)
    x[0] = mul48u_32_16(x[0], working_data.temperature_correction.amplitude_factor);
    #endif
    return x[0];
}
#endif

#if defined(FUNDAMENTAL_IRMS_SUPPORT)
    #if NUM_PHASES == 1
static rms_current_t evaluate_fundamental_rms_current(void)
    #else
static rms_current_t evaluate_fundamental_rms_current(struct phase_parms_s *phase, struct phase_calibration_data_s const *phase_cal)
    #endif
{
    rms_current_t x;
    int64_t z;

    if ((phase->status & (PHASE_STATUS_I_OVERRANGE | PHASE_STATUS_I_NEUTRAL_OVERRANGE)))
        return RMS_CURRENT_OVERRANGE;

    #if defined(__TI_COMPILER_VERSION__)
    /* TODO: We seem to need to calculate this way to get the right answer with CCS 5.1 */
    {
        int16_t y;

        z = phase->readings.fundamental_active_power;
        z *= phase->readings.fundamental_active_power;
        y = phase->readings.fundamental_reactive_power;
        y *= phase->readings.fundamental_reactive_power;
        z += y;
    }
    #else
    z = (int64_t) phase->readings.fundamental_active_power*phase->readings.fundamental_active_power
      + (int64_t) phase->readings.fundamental_reactive_power*phase->readings.fundamental_reactive_power;
    #endif
    /* Prevent tiny errors leading to tiny negative values for the fundamental RMS current */
    if (z <= 0)
        return 0;
    /* Avoid divide by zero */
    if (phase->readings.fundamental_V_rms <= 0)
        return 0;
    z = isqrt64(z)/phase->readings.fundamental_V_rms;
    /* We need to scale by 1000000. Multiply by 1000000/(2^6), and then shift down by 6 bits
       less than the 32 needed to get the fraction out of the sqrt answer */
    z *= 15625LL;
    x = z >> (32 - 6);
    #if defined(TEMPERATURE_CORRECTION_SUPPORT)
    x = mul48u_32_16(x, working_data.temperature_correction.amplitude_factor);
    #endif
    return x;
}
#endif

#if defined(CURRENT_THD_SUPPORT)
    #if NUM_PHASES == 1
static thd_t evaluate_current_thd(void)
    #else
static thd_t evaluate_current_thd(struct phase_parms_s *phase, struct phase_calibration_data_s const *phase_cal)
    #endif
{
    int64_t x;
    int64_t y;
    int64_t z;

    #if defined(CURRENT_THD_MEASUREMENT_CUTOFF)
    /* Don't calculate the THD when the current is small, as the results are too noisy */
    if (phase->readings.I_rms < CURRENT_THD_MEASUREMENT_CUTOFF)
        return 0;
    #endif

    /* Avoid silly results if the fundamental appears to exceed the total, either transiently or
       due to rounding issues. */
    if (phase->readings.fundamental_I_rms > phase->readings.I_rms)
        return 0;

    x = (int64_t) phase->readings.fundamental_I_rms*phase->readings.fundamental_I_rms;
    y = (int64_t) phase->readings.I_rms*phase->readings.I_rms;
    /* Prevent tiny errors in x and y from leading to tiny negative values for THD */
    if (x >= y)
        return 0;
    z = y - x;
    #if defined(IEC_THD_SUPPORT)
    z = isqrt64(z);
    z /= phase->readings.fundamental_I_rms;
    z *= 10000;
    z >>= 32;
    #else
    z *= 10000;
    z /= x;
    #endif
    /* Avoid stupid results when the maths is going crazy on zero current and
       out of lock conditions */
    if (z < 0)
        return 0;
    return z;
}
#endif

#if NUM_PHASES > 1  &&  defined(NEUTRAL_MONITOR_SUPPORT)  &&  defined(IRMS_SUPPORT)
static rms_current_t evaluate_neutral_rms_current(void)
{
    rms_current_t x;
    #if defined(TWENTYFOUR_BIT)
    int64_t tmp;
    #else
    int32_t tmp;
    #endif
    int dp;

    /* Calculate the RMS current in 1mA increments. Return -1 for overrange
       (i.e. ADC clip). A side effect of this routine is it updates the dynamic
       phase correction settings, based on the newly calculated current. */
    if ((working_data.neutral.status & PHASE_STATUS_I_OVERRANGE))
        return RMS_CURRENT_OVERRANGE;

    dp = working_data.neutral.metrology.dp_set;
    tmp = div_ac_current(working_data.neutral.metrology.dot_prod[dp].I_sq, working_data.neutral.metrology.dot_prod[dp].sample_count);
    if (tmp < cal_info->neutral.ac_offset)
        return 0;

    /* The ac_offset removes the effect of the AWGN from the ADC front end. AWGN is orthogonal to everything but a true copy
       of itself. This means means we need to subtract the ac_offset in a "Pythagoras" manner", while still squared. */
    #if defined(TWENTYFOUR_BIT)
    x = isqrt64(tmp - cal_info->neutral.ac_offset) >> 26;
    #else
    x = isqrt32(tmp - cal_info->neutral.ac_offset) >> 2;
    #endif
    x = mul48u_32_16(x, cal_info->neutral.I_rms_scale_factor[normal_limp]);
    #if defined(TEMPERATURE_CORRECTION_SUPPORT)
    x = mul48u_32_16(x, working_data.temperature_correction.amplitude_factor);
    #endif
    return x;
}
#endif

#if NUM_PHASES > 1  &&  defined(NEUTRAL_MONITOR_SUPPORT)  &&  defined(RESIDUAL_IRMS_SUPPORT)
static rms_current_t evaluate_residual_3phase_rms_current(void)
{
    rms_current_t x;
    uint16_t status;
    int dp;

    /* Calculate the RMS current in 1mA increments. Return RMS_CURRENT_OVERRANGE for overrange
       (i.e. ADC clipping). */
    status = working_data.phases[0].status | working_data.phases[1].status | working_data.phases[2].status | working_data.neutral.status;
    if ((status & PHASE_STATUS_I_OVERRANGE))
        return RMS_CURRENT_OVERRANGE;

    dp = working_data.neutral.metrology.dp_set;
    #if defined(TWENTYFOUR_BIT)
    x = isqrt64(div_ac_current(working_data.neutral.metrology.dot_prod[dp].residual_I_sq, working_data.neutral.metrology.dot_prod[dp].sample_count)) >> 26;
    #else
    x = isqrt32(div_ac_current(working_data.neutral.metrology.dot_prod[dp].residual_I_sq, working_data.neutral.metrology.dot_prod[dp].sample_count)) >> 2;
    #endif
    /* Use the scaling factor from phase 0 as a a compromise scaling factor. In most meters the phase to phase scaling variation is a
       fraction of a percent. */
    x = mul48u_32_16(x, cal_info->phases[0].current[0].I_rms_scale_factor[normal_limp]);
    #if defined(TEMPERATURE_CORRECTION_SUPPORT)
    x = mul48u_32_16(x, working_data.temperature_correction.amplitude_factor);
    #endif
    return x;
}
#endif

#if NUM_PHASES == 1
static power_t evaluate_active_power(void)
#else
static power_t evaluate_active_power(struct phase_parms_s *phase, struct phase_calibration_data_s const *phase_cal)
#endif
{
    #if defined(TWENTYFOUR_BIT)
    int64_t x[PER_PHASE_CURRENT_CHANNELS];
    #else
    int32_t x[PER_PHASE_CURRENT_CHANNELS];
    #endif
#if defined(PHASE_REVERSED_DETECTION_SUPPORT)
    int reversed;
#endif
#if defined(PHASE_UNBALANCED_DETECTION_SUPPORT)
    static const int32_t thresholds[2] = {PHASE_UNBALANCED_CUTOFF_THRESHOLD_POWER, PHASE_UNBALANCED_TIGHT_THRESHOLD_POWER};
#endif
#if defined(PHASE_REVERSED_DETECTION_SUPPORT)
    #if NUM_PHASES == 1  &&  defined(NEUTRAL_MONITOR_SUPPORT)
    static const uint16_t reversed_masks[PER_PHASE_CURRENT_CHANNELS] =
    {
        PHASE_STATUS_I_REVERSED,
        PHASE_STATUS_I_NEUTRAL_REVERSED
    };
    #else
    static const uint16_t reversed_masks[1] = {PHASE_STATUS_I_REVERSED};
    #endif
#endif
#if NUM_PHASES == 1  &&  defined(NEUTRAL_MONITOR_SUPPORT)
    int ch;
#endif
    int dp;

    if (phase->status & (PHASE_STATUS_V_OVERRANGE | PHASE_STATUS_I_OVERRANGE | PHASE_STATUS_I_NEUTRAL_OVERRANGE))
    {
#if defined(PER_SENSOR_PRECALCULATED_PARAMETER_SUPPORT)
        phase->metrology.current[0].readings.active_power = POWER_OVERRANGE;
    #if NUM_PHASES == 1  &&  defined(NEUTRAL_MONITOR_SUPPORT)
        phase->metrology.current[1].readings.active_power = POWER_OVERRANGE;
    #endif
#endif
        return POWER_OVERRANGE;
    }

    /* We can only do real power assessment in full operating mode. */
    /* If we have neutral monitoring for a single phase meter, we need to measure
       both power levels, and decide between them. Issues to be assessed here are
       whether one or both leads show reverse power, and whether the power levels
       are balanced. */
#if defined(PHASE_REVERSED_DETECTION_SUPPORT)
    /* If we find a negative power level we may be genuinely feeding power to the grid,
       or we may be seeing a tamper condition. This is application dependent. */
    reversed = false;
#endif

    dp = phase->metrology.dp_set;
#if NUM_PHASES == 1  &&  defined(NEUTRAL_MONITOR_SUPPORT)
    for (ch = 0;  ch < 2;  ch++)
#endif
    {
        x[ch] = div_ac_power(phase->metrology.current[ch].dot_prod[dp].P_active, phase->metrology.current[ch].dot_prod[dp].sample_count);
        x[ch] >>= 9;
        x[ch] = mul48_32_16(x[ch], phase_cal->current[ch].P_scale_factor);
#if defined(PER_SENSOR_PRECALCULATED_PARAMETER_SUPPORT)
        phase->metrology.current[ch].readings.active_power = x[ch];
#endif
#if defined(PHASE_REVERSED_DETECTION_SUPPORT)
        if (x[ch] < 0)
        {
    #if defined(ON_REVERSED_SELECT_POSITIVE_READING)
            x[ch] = -x[ch];
    #endif
            phase->status |= reversed_masks[ch];
            if (x[ch] > PHASE_REVERSED_THRESHOLD_POWER)
                reversed = true;
        }
        else
        {
            phase->status &= ~reversed_masks[ch];   
        }
#endif
    }

#if defined(PHASE_UNBALANCED_DETECTION_SUPPORT)
    x[0] = test_phase_balance(x[0], x[1], thresholds);
    if ((phase->status & PHASE_STATUS_UNBALANCED))
    {
        /* When the phase is unbalanced we only look for reversed current in the 
           lead with the higher current. If we do not impose this restriction, coupling
           through a parasitic CT power supply transformer can cause the reverse condition
           to be raised incorrectly. If there is no parasitic supply this test is probably
           a waste of time. */
        if ((phase->status & PHASE_STATUS_CURRENT_FROM_NEUTRAL))
            reversed = phase->status & PHASE_STATUS_I_NEUTRAL_REVERSED;
        else
            reversed = phase->status & PHASE_STATUS_I_REVERSED;
    }
#endif

#if defined(PHASE_REVERSED_DETECTION_SUPPORT)
    if ((phase->status & PHASE_STATUS_REVERSED))
    {
        if (!reversed)
        {
            if (--phase->metrology.current_reversed_persistence_check <= -PHASE_REVERSED_PERSISTENCE_CHECK)
            {
                phase->status &= ~PHASE_STATUS_REVERSED;
                phase->metrology.current_reversed_persistence_check = 0;
            }
        }
        else
        {
            phase->metrology.current_reversed_persistence_check = 0;
        }
    }
    else
    {
        if (reversed)
        {
            if (++phase->metrology.current_reversed_persistence_check >= PHASE_REVERSED_PERSISTENCE_CHECK)
            {
                phase->status |= PHASE_STATUS_REVERSED;
                phase->metrology.current_reversed_persistence_check = 0;
            }
        }
        else
        {
            phase->metrology.current_reversed_persistence_check = 0;
        }
    }
#endif
#if defined(TEMPERATURE_CORRECTION_SUPPORT)
    x[0] = mul48_32_16(x[0], working_data.temperature_correction.power_factor);
#endif
    return x[0];
}

#if defined(REACTIVE_POWER_SUPPORT)  &&  defined(REACTIVE_POWER_BY_QUADRATURE_SUPPORT)
    #if NUM_PHASES == 1
static power_t evaluate_reactive_power(void)
    #else
static power_t evaluate_reactive_power(struct phase_parms_s *phase, struct phase_calibration_data_s const *phase_cal)
    #endif
{
    #if defined(TWENTYFOUR_BIT)
    int64_t x;
    #else
    int32_t x;
    #endif
    power_t y;
    uint16_t scaling;
    #if NUM_PHASES == 1  &&  defined(NEUTRAL_MONITOR_SUPPORT)
    int ch;
    #endif
    int dp;

    if (phase->status & (PHASE_STATUS_V_OVERRANGE | PHASE_STATUS_I_OVERRANGE | PHASE_STATUS_I_NEUTRAL_OVERRANGE))
        return POWER_OVERRANGE;

    dp = phase->metrology.dp_set;
    #if NUM_PHASES == 1  &&  defined(NEUTRAL_MONITOR_SUPPORT)
        #if defined(ON_UNBALANCED_SELECT_HIGHER_READING)
    /* If we have neutral monitoring for a single phase meter, we need to use whichever
       channel has been selected by the anti-tamper validation scheme. */
    ch = (phase->status & PHASE_STATUS_CURRENT_FROM_NEUTRAL)  ?  1  :  0;
        #else
    ch = 0;
        #endif
    #endif

    x = div_ac_power(phase->metrology.current[ch].dot_prod[dp].P_reactive, phase->metrology.current[ch].dot_prod[dp].sample_count);
    /* We need to shift down less than for the active power, as we scaled down the phase shifted voltage, so it could never
       overflow the 16 bit signed range. */
    x >>= (9 - 2);
    /* The power scaling factor has to allow for the gain of the FIR used to phase shift the voltage */
    scaling = ((uint32_t) phase_cal->current[ch].P_scale_factor*phase->metrology.current[ch].quadrature_correction.fir_gain) >> 15;
    y = mul48_32_16(x, scaling);
    #if defined(TEMPERATURE_CORRECTION_SUPPORT)
    y = mul48_32_16(y, working_data.temperature_correction.power_factor);
    #endif
    return y;
}
#endif

#if defined(APPARENT_POWER_SUPPORT)
    #if NUM_PHASES == 1
static int32_t evaluate_apparent_power(void)
    #else
static int32_t evaluate_apparent_power(struct phase_parms_s *phase, struct phase_calibration_data_s const *phase_cal)
    #endif
{
    #if defined(REACTIVE_POWER_BY_QUADRATURE_SUPPORT)
    int64_t z;

    if (phase->readings.active_power == POWER_OVERRANGE  ||  phase->readings.reactive_power == POWER_OVERRANGE)
        return POWER_OVERRANGE;

    /* Calculate apparent (VA) power in 0.01W increments */
        #if defined(__TI_COMPILER_VERSION__)
    /* TODO: We seem to need to calculate this way to get the right answer with CCS 5.1 */
    {
        int64_t y;

        z = phase->readings.active_power;
        z *= phase->readings.active_power;
        y = phase->readings.reactive_power;
        y *= phase->readings.reactive_power;
        z += y;
    }
        #else
    z = (int64_t) phase->readings.active_power*phase->readings.active_power
      + (int64_t) phase->readings.reactive_power*phase->readings.reactive_power;
        #endif
    return isqrt64i(z);
    #else
    rms_voltage_t x;
    rms_current_t y;
        #if NUM_PHASES == 1  &&  defined(NEUTRAL_MONITOR_SUPPORT)
    int ch;
        #endif
    int dp;

    dp = phase->metrology.dp_set;
    /* Calculate VA power in 1mW increments */
    x = isqrt32(div_ac_voltage(phase->metrology.dot_prod[dp].V_sq, phase->metrology.dot_prod[dp].sample_count));
        #if defined(LIMP_MODE_SUPPORT)
    x = (x >> 12)*phase_cal->V_rms_scale_factor[normal_limp];
        #else
    x = (x >> 12)*phase_cal->V_rms_scale_factor;
        #endif
    x >>= 14;

        #if NUM_PHASES == 1  &&  defined(NEUTRAL_MONITOR_SUPPORT)
            #if defined(ON_UNBALANCED_SELECT_HIGHER_READING)
    ch = (phase->status & PHASE_STATUS_CURRENT_FROM_NEUTRAL)  ?  1  :  0;
            #else
    ch = 0;
            #endif
        #endif
    /* The ac_offset removes the effect of the AWGN from the ADC front end. AWGN is orthogonal to everything but a true copy
       of itself. This means means we need to subtract the ac_offset in a "Pythagoras" manner", while still squared. */
        #if defined(TWENTYFOUR_BIT)
    y = isqrt64(div_ac_current(phase->metrology.current[ch].dot_prod[dp].I_sq, phase->metrology.current[ch].dot_prod[dp].sample_count)
              - phase_cal->current[ch].ac_offset) >> 36;
        #else
    y = isqrt32(div_ac_current(phase->metrology.current[ch].dot_prod[dp].I_sq, phase->metrology.current[ch].dot_prod[dp].sample_count)
              - phase_cal->current[ch].ac_offset) >> 12;
        #endif
        #if defined(LIMP_MODE_SUPPORT)
    y *= phase_cal->current[ch].I_rms_scale_factor[normal_limp];
        #else
    y *= phase_cal->current[ch].I_rms_scale_factor;
        #endif
    y >>= 14;

    x *= y;
    x /= 1000;
        #if defined(TEMPERATURE_CORRECTION_SUPPORT)
    x = mul48_32_16(x, working_data.temperature_correction.power_factor);
        #endif
    return x;
    #endif
}
#endif

#if defined(FUNDAMENTAL_ACTIVE_POWER_SUPPORT)
    #if NUM_PHASES == 1
static power_t evaluate_fundamental_active_power(void)
    #else
static power_t evaluate_fundamental_active_power(struct phase_parms_s *phase, struct phase_calibration_data_s const *phase_cal)
    #endif
{
    #if defined(TWENTYFOUR_BIT)
    int64_t x;
    #else
    int32_t x;
    #endif
    int32_t y;
    int32_t z;
    #if NUM_PHASES == 1  &&  defined(NEUTRAL_MONITOR_SUPPORT)
    int ch;
    #endif
    int dp;

    if (phase->status & (PHASE_STATUS_V_OVERRANGE | PHASE_STATUS_I_OVERRANGE | PHASE_STATUS_I_NEUTRAL_OVERRANGE))
        return POWER_OVERRANGE;

    dp = phase->metrology.dp_set;
    #if NUM_PHASES == 1  &&  defined(NEUTRAL_MONITOR_SUPPORT)
        #if defined(ON_UNBALANCED_SELECT_HIGHER_READING)
    /* If we have neutral monitoring for a single phase meter, we need to use whichever
       channel has been selected by the anti-tamper validation scheme. */
    ch = (phase->status & PHASE_STATUS_CURRENT_FROM_NEUTRAL)  ?  1  :  0;
        #else
    ch = 0;
        #endif
    #endif

    x = div_ac_power(phase->metrology.current[ch].dot_prod[dp].P_fundamental_active, phase->metrology.current[ch].dot_prod[dp].sample_count);
    x >>= 6;
    y = mul48_32_16(x, phase_cal->current[ch].P_scale_factor);
    /* Scale by the voltage gain */
    z = div_ac_voltage(phase->metrology.dot_prod[dp].V_fundamental, phase->metrology.current[ch].dot_prod[dp].sample_count);
    /* Scale down by the size of the reference signal to 15 bits */
    z >>= 16;
    y = mul48_32_16(y, (int16_t) z);
    #if defined(TEMPERATURE_CORRECTION_SUPPORT)
    y = mul48_32_16(y, working_data.temperature_correction.power_factor);
    #endif
    return y;
}
#endif

#if defined(FUNDAMENTAL_REACTIVE_POWER_SUPPORT)
    #if NUM_PHASES == 1
static power_t evaluate_fundamental_reactive_power(void)
    #else
static power_t evaluate_fundamental_reactive_power(struct phase_parms_s *phase, struct phase_calibration_data_s const *phase_cal)
    #endif
{
    #if defined(TWENTYFOUR_BIT)
    int64_t x;
    #else
    int32_t x;
    #endif
    power_t y;
    rms_voltage_t z;
    #if NUM_PHASES == 1  &&  defined(NEUTRAL_MONITOR_SUPPORT)
    int ch;
    #endif
    int dp;

    if (phase->status & (PHASE_STATUS_V_OVERRANGE | PHASE_STATUS_I_OVERRANGE | PHASE_STATUS_I_NEUTRAL_OVERRANGE))
        return POWER_OVERRANGE;

    dp = phase->metrology.dp_set;
    #if NUM_PHASES == 1  &&  defined(NEUTRAL_MONITOR_SUPPORT)
        #if defined(ON_UNBALANCED_SELECT_HIGHER_READING)
    /* If we have neutral monitoring for a single phase meter, we need to use whichever
       channel has been selected by the anti-tamper validation scheme. */
    ch = (phase->status & PHASE_STATUS_CURRENT_FROM_NEUTRAL)  ?  1  :  0;
        #else
    ch = 0;
        #endif
    #endif

    x = div_ac_power(phase->metrology.current[ch].dot_prod[dp].P_fundamental_reactive, phase->metrology.current[ch].dot_prod[dp].sample_count);
    x >>= 6;
    y = mul48_32_16(x, phase_cal->current[ch].P_scale_factor);
    /* Scale by the voltage gain */
    z = div_ac_voltage(phase->metrology.dot_prod[dp].V_fundamental, phase->metrology.current[ch].dot_prod[dp].sample_count);
    /* Scale down by the size of the reference signal to 15 bits */
    z >>= 16;
    y = mul48_32_16(y, (int16_t) z);
    #if defined(TEMPERATURE_CORRECTION_SUPPORT)
    y = mul48_32_16(y, working_data.temperature_correction.power_factor);
    #endif
    return y;
}
#endif

#if defined(POWER_FACTOR_SUPPORT)
    #if NUM_PHASES == 1
static int16_t evaluate_power_factor(void)
    #else
static int16_t evaluate_power_factor(struct phase_parms_s *phase, struct phase_calibration_data_s const *phase_cal)
    #endif
{
    power_t p;
    power_t x;
    #if NUM_PHASES == 1  &&  defined(NEUTRAL_MONITOR_SUPPORT)  &&  !defined(REACTIVE_POWER_BY_QUADRATURE_SUPPORT)
    int ch;
    #endif

    p = abs32(phase->readings.active_power);
    #if defined(POWER_FACTOR_MEASUREMENT_CUTOFF)
    if (p < POWER_FACTOR_MEASUREMENT_CUTOFF/10)
        return 10000;
    #endif
    #if defined(APPARENT_POWER_SUPPORT)
    x = abs32(phase->readings.apparent_power);
    #endif
    if (p  &&  x)
    {
        /* Justify for optimal accuracy */
        while ((p & 0x40000000) == 0  &&  (x & 0x40000000) == 0)
        {
            p <<= 1;
            x <<= 1;
        }
        x >>= 16;
        p /= x;
        p *= 10000;
        p >>= 16;
        /* Don't let a little imprecision cause strange answers */
        if (p > 10000)
            p = 10000;
    }
    else
    {
        p = 0;
    }
    /* Use a negative PF to indicate an inductive load */
    #if defined(REACTIVE_POWER_BY_QUADRATURE_SUPPORT)
    if (phase->readings.reactive_power >= 0)
        p = -p;
    if (phase->readings.active_power < 0)
        p = -p;   
    #else
        #if NUM_PHASES == 1  &&  defined(NEUTRAL_MONITOR_SUPPORT)
            #if defined(ON_UNBALANCED_SELECT_HIGHER_READING)
    /* If we have neutral monitoring for a single phase meter, we need to use whichever
       channel has been selected by the anti-tamper validation scheme. */
    ch = (phase->status & PHASE_STATUS_CURRENT_FROM_NEUTRAL)  ?  1  :  0;
            #else
    ch = 0;
            #endif
        #endif
    if (phase->metrology.current[ch].leading < 0)
        p = -p;
    #endif
    return p;
}
#endif

#if defined(ENERGY_SUPPORT)
static energy_t integrate_power_to_energy(struct energy_integrator_t *en, power_t pow, int samples)
{
    energy_t energy;
    energy_t energy_step;

    energy = (energy_t) pow*samples + en->energy_residual;
    energy_step = 0;
    while (energy >= ENERGY_100MWATT_HOUR_THRESHOLD)
    {
        energy -= ENERGY_100MWATT_HOUR_THRESHOLD;
        energy_step++;
    }
    en->energy_residual = energy;
    return energy_step;
}

    #if NUM_PHASES == 1
static void accumulate_phase_energies(void)
    #else
static void accumulate_phase_energies(int ph, struct phase_parms_s *phase, struct phase_calibration_data_s const *phase_cal)
    #endif
{
    energy_t en;
    power_t pow;
    int which;
    int dp;

    if (phase->readings.active_power == POWER_OVERRANGE)
        return;
    if (abs32(phase->readings.active_power) < RESIDUAL_POWER_CUTOFF/10)
        return;
    dp = phase->metrology.dp_set;

    #if defined(ACTIVE_ENERGY_SUPPORT)
    pow = phase->readings.active_power;
    if (phase->readings.active_power >= 0)
    {
        which = ENERGY_DIRECTION_IMPORT;
    }
    else
    {
        which = ENERGY_DIRECTION_EXPORT;
        pow = -pow;
    }
    en = integrate_power_to_energy(&phase->energy.active[which], pow, phase->metrology.current[0].dot_prod[dp].sample_count);
    if (en)
        energy_update(ph, ACTIVE_ENERGY_IMPORTED + which, en);
    #endif

    #if defined(FUNDAMENTAL_ACTIVE_ENERGY_SUPPORT)
    pow = phase->readings.fundamental_active_power;
    if (phase->readings.fundamental_active_power >= 0)
    {
        which = ACTIVE_ENERGY_IMPORTED;
    }
    else
    {
        which = ACTIVE_ENERGY_EXPORTED;
        pow = -pow;
    }
    en = integrate_power_to_energy(&phase->energy.fundamental_active[which - ACTIVE_ENERGY_IMPORTED], pow, phase->metrology.current[0].dot_prod[dp].sample_count);
    if (en)
        energy_update(ph, FUNDAMENTAL_ACTIVE_ENERGY_IMPORTED + which, en);
    #endif

    #if defined(REACTIVE_ENERGY_SUPPORT)
    pow = phase->readings.reactive_power;
    if (phase->readings.reactive_power >= 0)
    {
        which = (phase->readings.active_power >= 0)  ?  ENERGY_QUADRANT_I  :  ENERGY_QUADRANT_II;
    }
    else
    {
        which = (phase->readings.active_power >= 0)  ?  ENERGY_QUADRANT_IV  :  ENERGY_QUADRANT_III;
        pow = -pow;
    }
    en = integrate_power_to_energy(&phase->energy.reactive[which], pow, phase->metrology.current[0].dot_prod[dp].sample_count);
    if (en)
        energy_update(ph, REACTIVE_ENERGY_QUADRANT_I + which, en);
    #endif

    #if defined(FUNDAMENTAL_REACTIVE_ENERGY_SUPPORT)
    pow = phase->readings.fundamental_reactive_power;
    if (phase->readings.fundamental_reactive_power >= 0)
    {
        which = (phase->readings.active_power >= 0)  ?  ENERGY_QUADRANT_I  :  ENERGY_QUADRANT_II;
    }
    else
    {
        which = (phase->readings.active_power >= 0)  ?  ENERGY_QUADRANT_IV  :  ENERGY_QUADRANT_III;
        pow = -pow;
    }
    en = integrate_power_to_energy(&phase->energy.fundamental_reactive[which], pow, phase->metrology.current[0].dot_prod[dp].sample_count);
    if (en)
        energy_update(ph, FUNDAMENTAL_REACTIVE_ENERGY_QUADRANT_I + which, en);
    #endif

    #if defined(APPARENT_ENERGY_SUPPORT)
    pow = phase->readings.apparent_power;
    if (phase->readings.active_power >= 0)
    {
        which = ENERGY_DIRECTION_IMPORT;
    }
    else
    {
        which = ENERGY_DIRECTION_EXPORT;
        pow = pow;
    }
    en = integrate_power_to_energy(&phase->energy.apparent[which], pow, phase->metrology.current[0].dot_prod[dp].sample_count);
    if (en)
        energy_update(ph, APPARENT_ENERGY_IMPORTED + which, en);
    #endif

    #if defined(INTEGRATED_V2_SUPPORT)
    /* Scale by sqrt(1000) */
    pow = phase->readings.V_rms*2072L;
    pow = pow >> 16;
    pow = pow*pow;
    en = integrate_power_to_energy(&phase->energy.integrated_v2, pow, phase->metrology.current[0].dot_prod[dp].sample_count);
    if (en)
        energy_update(ph, INTEGRATED_V2, en);
    #endif

    #if defined(INTEGRATED_I2_SUPPORT)
    /* Scale by sqrt(1000)*1000 */
    pow = phase->readings.I_rms >> 10;
    pow = pow*2122L;
    pow = pow >> 16;
    pow = pow*pow;
    en = integrate_power_to_energy(&phase->energy.integrated_i2, pow, phase->metrology.current[0].dot_prod[dp].sample_count);
    if (en)
        energy_update(ph, INTEGRATED_I2, en);
    #endif
}
#endif

#if defined(TOTAL_ENERGY_SUPPORT)
    #if NUM_PHASES == 1
static void accumulate_total_energies(void)
    #else
static void accumulate_total_energies(struct phase_parms_s *phase, struct phase_calibration_data_s const *phase_cal)
    #endif
{
    energy_t en;
    power_t pow;
    int which;
    int dp;

    if (working_data.totals.readings.active_power == POWER_OVERRANGE)
        return;
    if (abs32(working_data.totals.readings.active_power) < TOTAL_RESIDUAL_POWER_CUTOFF)
        return;
    dp = phase->metrology.dp_set;
    #if defined(TOTAL_ACTIVE_ENERGY_SUPPORT)
    pow = phase->readings.active_power;
    if (working_data.totals.readings.active_power >= 0)
    {
        which = ENERGY_DIRECTION_IMPORT;
    }
    else
    {
        which = ENERGY_DIRECTION_EXPORT;
        pow = -pow;
    }
    en = integrate_power_to_energy(&working_data.totals.energy.active[which], pow, phase->metrology.current[0].dot_prod[dp].sample_count);
    if (en)
        energy_update(FAKE_PHASE_TOTAL, ACTIVE_ENERGY_IMPORTED + which, en);
    #endif
    #if defined(TOTAL_FUNDAMENTAL_ACTIVE_ENERGY_SUPPORT)
    pow = phase->readings.fundamental_active_power;
    if (working_data.totals.readings.fundamental_active_power >= 0)
    {
        which = ENERGY_DIRECTION_IMPORT;
    }
    else
    {
        which = ENERGY_DIRECTION_EXPORT;
        pow = -pow;
    }
    en = integrate_power_to_energy(&working_data.totals.energy.fundamental_active[which], pow, phase->metrology.current[0].dot_prod[dp].sample_count);
    if (en)
        energy_update(FAKE_PHASE_TOTAL, FUNDAMENTAL_ACTIVE_ENERGY_IMPORTED + which, en);
    #endif
    #if defined(TOTAL_REACTIVE_ENERGY_SUPPORT)
    pow = phase->readings.reactive_power;
    if (working_data.totals.readings.reactive_power >= 0)
    {
        which = (working_data.totals.readings.active_power >= 0)  ?  ENERGY_QUADRANT_I  :  ENERGY_QUADRANT_II;
    }
    else
    {
        which = (working_data.totals.readings.active_power >= 0)  ?  ENERGY_QUADRANT_IV  :  ENERGY_QUADRANT_III;
        pow = -pow;
    }
    en = integrate_power_to_energy(&working_data.totals.energy.reactive[which], pow, phase->metrology.current[0].dot_prod[dp].sample_count);
    if (en)
        energy_update(FAKE_PHASE_TOTAL, REACTIVE_ENERGY_QUADRANT_I + which, en);
    #endif
    #if defined(TOTAL_FUNDAMENTAL_REACTIVE_ENERGY_SUPPORT)
    pow = phase->readings.fundamental_reactive_power;
    if (working_data.totals.readings.fundamental_reactive_power >= 0)
    {
        which = (working_data.totals.readings.active_power >= 0)  ?  ENERGY_QUADRANT_I  :  ENERGY_QUADRANT_II;
    }
    else
    {
        which = (working_data.totals.readings.active_power >= 0)  ?  ENERGY_QUADRANT_IV  :  ENERGY_QUADRANT_III;
        pow = -pow;
    }
    en = integrate_power_to_energy(&working_data.totals.energy.fundamental_reactive[which], pow, phase->metrology.current[0].dot_prod[dp].sample_count);
    if (en)
        energy_update(FAKE_PHASE_TOTAL, FUNDAMENTAL_REACTIVE_ENERGY_QUADRANT_I + which, en);
    #endif
    #if defined(TOTAL_APPARENT_ENERGY_SUPPORT)
    pow = phase->readings.apparent_power;
    if (working_data.totals.readings.active_power >= 0)
    {
        which = ENERGY_DIRECTION_IMPORT;
    }
    else
    {
        which = ENERGY_DIRECTION_EXPORT;
        pow = pow;
    }
    en = integrate_power_to_energy(&working_data.totals.energy.apparent[which], pow, phase->metrology.current[0].dot_prod[dp].sample_count);
    if (en)
        energy_update(FAKE_PHASE_TOTAL, APPARENT_ENERGY_IMPORTED + which, en);
    #endif
}
#endif

#if defined(SAG_SWELL_SUPPORT)
    #if NUM_PHASES == 1
static void sag_swell_control(void)
    #else
static void sag_swell_control(struct phase_parms_s *phase, struct phase_calibration_data_s const *phase_cal)
    #endif
{
    int64_t xxx;
    int32_t yyy;
    int64_t zzz;

    /* Find the current sag and swell thresholds, based on the current mains period */
    xxx = MAINS_NOMINAL_VOLTAGE*1000LL*1024LL;
    xxx /= phase_cal->V_rms_scale_factor[normal_limp];
    xxx = xxx*xxx;

    /* Find the scaling for the number of samples in the window */
    yyy = (phase->metrology.voltage_period.period >> 16)*SAG_SWELL_WINDOW_LEN;
    yyy >>= 8;

    zzz = xxx*(((100 - SAG_THRESHOLD)*256)/100)*(((100 - SAG_THRESHOLD)*256)/100);
    zzz >>= 16;
    zzz *= yyy;
    phase->metrology.sag_threshold[1] = zzz >> 16;
    phase->metrology.sag_threshold[2] = zzz >> 32;

    zzz = xxx*(((100 + SWELL_THRESHOLD)*256)/100)*(((100 + SWELL_THRESHOLD)*256)/100);
    zzz >>= 16;
    zzz *= yyy;
    phase->metrology.swell_threshold[1] = zzz >> 16;
    phase->metrology.swell_threshold[2] = zzz >> 32;
}
#endif

#if defined(TEMPERATURE_SUPPORT)
static void evaluate_temperature(void)
{
    //int32_t temp;

    /* Find the temperature, in Celsius, based on the values for the slope and intercept of the
       sensor characteristic found at calibration time. */
    /* The temperature in Celsius is approx. (Vsensor - 986mV)/3.55mV . The exact voltages
       are the subject of calibration. */
    #if defined(TWENTYFOUR_BIT)  &&  !defined(__MSP430_HAS_ADC10_A__)
    temp = raw_temperature_from_adc >> 8;
    #else
    //temp = raw_temperature_from_adc;
    #endif
    temperature_in_celsius= (((long)(raw_temperature_from_adc) - CALADC10_20V_30C) * (850 - 300)) / (CALADC10_20V_85C - CALADC10_20V_30C) + 300;   
    /* We now have the temperature in 1/10ths of degrees C. */
}
#endif

#if defined(LIMP_MODE_SUPPORT)
void metrology_limp_normal_detection(void)
{
    static const rms_voltage_t thresholds[] =
    {
        (rms_voltage_t) LIMP_MODE_VOLTAGE_THRESHOLD*RMS_VOLTAGE_SCALING_FACTOR,
        (rms_voltage_t) NORMAL_MODE_VOLTAGE_THRESHOLD*RMS_VOLTAGE_SCALING_FACTOR
    };

    /* We switch into normal mode if the RMS voltage exceeds a threshold, and
       the DC content of the signal looks OK.
       The threshold we use when switching from limp to normal is higher
       than the threshold we use then switching from normal to limp. This creates
       hysteresis.
       The voltage channel DC offset will not move very much when the meter is
       operating normally, even over wide temperature changes. If our estimate
       moves, there must be some tampering introducing DC into the voltage
       signal, such as a diode between the grid and the meter. */
    if (phase->readings.V_rms >= thresholds[normal_limp]
        &&
        (phase->metrology.V_dc_estimate[normal_limp][0] >> 16) <= phase_cal->upper_v_dc_estimate[normal_limp]
        &&
        (phase->metrology.V_dc_estimate[normal_limp][0] >> 16) >= phase_cal->lower_v_dc_estimate[normal_limp])
    {
        if (operating_mode == OPERATING_MODE_LIMP)
        {
            switch_to_normal_mode();
            normal_limp = 0;
        }
    }
    else
    {
        if (operating_mode == OPERATING_MODE_NORMAL)
        {
            switch_to_limp_mode();
            normal_limp = 1;
        }
    }
}
#endif

#if defined(LIMP_MODE_SUPPORT)
    #if NUM_PHASES == 1
static void calculate_limp_phase_readings(void)
    #else
static void calculate_limp_phase_readings(int ph)
    #endif
{
    #if NUM_PHASES > 1
    struct phase_parms_s *phase;
    struct phase_calibration_data_s const *phase_cal;

    phase = &chan[ph];
    phase_cal = &cal_info->phases[ph];
    #endif
    /* In limp mode we must assess estimated power from only the measured current. */
    /* We cannot properly determine current reversal in this mode. Also, current
       imbalance is really just a measure of which lead is still connected.
       Just treat both the imbalance and reversal conditions as OK */
    #if NUM_PHASES == 1
        #if defined(VRMS_SUPPORT)
    phase->readings.V_rms = evaluate_rms_voltage();
        #endif
        #if defined(IRMS_SUPPORT)
    phase->readings.I_rms = evaluate_rms_current();
        #endif
    #else
        #if defined(VRMS_SUPPORT)
    phase->V_rms = evaluate_rms_voltage(phase, phase_cal);
        #endif
        #if defined(IRMS_SUPPORT)
    phase->readings.I_rms = evaluate_rms_current(phase, phase_cal, ch);
        #endif
    #endif
    phase->readings.active_power = phase->readings.I_rms*MAINS_NOMINAL_VOLTAGE/1000;
}
#endif

#if NUM_PHASES == 1
void calculate_phase_readings(int phx)
#else
void calculate_phase_readings(int ph)
#endif
{
    int dp;

#if NUM_PHASES == 1
    #if defined(LIMP_MODE_SUPPORT)
    if (operating_mode == OPERATING_MODE_LIMP)
    {
        calculate_limp_phase_readings();
        return;
    }
    #endif

    phase->readings.active_power = evaluate_active_power();
    #if defined(FUNDAMENTAL_ACTIVE_POWER_SUPPORT)
    phase->readings.fundamental_active_power = evaluate_fundamental_active_power();
    #endif

    #if defined(REACTIVE_POWER_SUPPORT)
    phase->readings.reactive_power = evaluate_reactive_power();
    #endif
    #if defined(FUNDAMENTAL_REACTIVE_POWER_SUPPORT)
    phase->readings.fundamental_reactive_power = evaluate_fundamental_reactive_power();
    #endif

    #if defined(APPARENT_POWER_SUPPORT)
    phase->readings.apparent_power = evaluate_apparent_power();
    #endif

    #if defined(VRMS_SUPPORT)
    phase->readings.V_rms = evaluate_rms_voltage();
    #endif
    #if defined(FUNDAMENTAL_VRMS_SUPPORT)
    phase->readings.fundamental_V_rms = evaluate_fundamental_rms_voltage();
    #endif
    #if defined(VOLTAGE_THD_SUPPORT)
    phase->readings.voltage_thd = evaluate_voltage_thd();
    #endif

    #if defined(IRMS_SUPPORT)
    phase->readings.I_rms = evaluate_rms_current();
    #endif
    #if defined(FUNDAMENTAL_IRMS_SUPPORT)
    phase->readings.fundamental_I_rms = evaluate_fundamental_rms_current();
    #endif
    #if defined(CURRENT_THD_SUPPORT)
    phase->readings.current_thd = evaluate_current_thd();
    #endif

    #if defined(POWER_FACTOR_SUPPORT)
    /* The power factor should be calculated last */
    phase->readings.power_factor = evaluate_power_factor();
    #endif

    #if defined(MAINS_FREQUENCY_SUPPORT)
    phase->readings.frequency = evaluate_mains_frequency();
    #endif

    #if defined(ENERGY_SUPPORT)  ||  defined(TOTAL_ENERGY_SUPPORT)
    accumulate_phase_energies();
    #endif

    #if defined(SAG_SWELL_SUPPORT)
    sag_swell_control();
    #endif
#else
    struct phase_parms_s *phase;
    struct phase_calibration_data_s const *phase_cal;

    #if defined(LIMP_MODE_SUPPORT)
    if (operating_mode == OPERATING_MODE_LIMP)
        return calculate_limp_phase_readings(ph);
    #endif
    phase = &working_data.phases[ph];
    phase_cal = &cal_info->phases[ph];

    phase->readings.active_power = evaluate_active_power(phase, phase_cal);
    #if defined(TOTAL_ACTIVE_POWER_SUPPORT)
    working_data.totals.readings.active_power = working_data.phases[0].readings.active_power;
        #if NUM_PHASES >= 2
    working_data.totals.readings.active_power += working_data.phases[1].readings.active_power;
        #endif
        #if NUM_PHASES >= 3
    working_data.totals.readings.active_power += working_data.phases[2].readings.active_power;
        #endif
    
    
    #endif
    #if defined(FUNDAMENTAL_ACTIVE_POWER_SUPPORT)
    phase->readings.fundamental_active_power = evaluate_fundamental_active_power(phase, phase_cal);
    #endif

    #if defined(REACTIVE_POWER_SUPPORT)
    phase->readings.reactive_power = evaluate_reactive_power(phase, phase_cal);
        #if defined(TOTAL_REACTIVE_POWER_SUPPORT)
    working_data.totals.readings.reactive_power = working_data.phases[0].readings.reactive_power;
            #if NUM_PHASES >= 2
    working_data.totals.readings.reactive_power += working_data.phases[1].readings.reactive_power;
            #endif
            #if NUM_PHASES >= 3
    working_data.totals.readings.reactive_power += working_data.phases[2].readings.reactive_power;
            #endif
        #endif
    #endif
    #if defined(FUNDAMENTAL_REACTIVE_POWER_SUPPORT)
    phase->readings.fundamental_reactive_power = evaluate_fundamental_reactive_power(phase, phase_cal);
    #endif

    #if defined(APPARENT_POWER_SUPPORT)
    phase->readings.apparent_power = evaluate_apparent_power(phase, phase_cal);
        #if defined(TOTAL_APPARENT_POWER_SUPPORT)
    working_data.totals.readings.apparent_power = working_data.phases[0].readings.apparent_power;
            #if NUM_PHASES >= 2
    working_data.totals.readings.apparent_power += working_data.phases[1].readings.apparent_power;
            #endif
            #if NUM_PHASES >= 3
    working_data.totals.readings.apparent_power += working_data.phases[2].readings.apparent_power;
            #endif
        #endif
    #endif

    #if defined(VRMS_SUPPORT)
    phase->readings.V_rms = evaluate_rms_voltage(phase, phase_cal);
    #endif
    #if defined(FUNDAMENTAL_VRMS_SUPPORT)
    phase->readings.fundamental_V_rms = evaluate_fundamental_rms_voltage(phase, phase_cal);
    #endif
    #if defined(VOLTAGE_THD_SUPPORT)
    phase->readings.voltage_thd = evaluate_voltage_thd(phase, phase_cal);
    #endif

    #if defined(IRMS_SUPPORT)
    phase->readings.I_rms = evaluate_rms_current(phase, phase_cal, ph);
    #endif
    #if defined(FUNDAMENTAL_IRMS_SUPPORT)
    phase->readings.fundamental_I_rms = evaluate_fundamental_rms_current(phase, phase_cal);
    #endif
    #if defined(CURRENT_THD_SUPPORT)
    phase->readings.current_thd = evaluate_current_thd(phase, phase_cal);
    #endif

    #if defined(POWER_FACTOR_SUPPORT)
    /* The power factor should be calculated last */
    phase->readings.power_factor = evaluate_power_factor(phase, phase_cal);
    #endif

    #if defined(MAINS_FREQUENCY_SUPPORT)
    phase->readings.frequency = evaluate_mains_frequency(phase, phase_cal);
    #endif

    #if defined(ENERGY_SUPPORT)
    accumulate_phase_energies(ph, phase, phase_cal);
    #endif

    #if defined(TOTAL_ENERGY_SUPPORT)
    accumulate_total_energies(phase, phase_cal);
    #endif

    #if defined(SAG_SWELL_SUPPORT)
    sag_swell_control(phase, phase_cal);
    #endif
#endif

#if defined(TEMPERATURE_SUPPORT)
    #if NUM_PHASES > 1
    if (ph == 0)
    #endif
        evaluate_temperature();
#endif
    dp = phase->metrology.dp_set;
    memset(&phase->metrology.dot_prod[dp], 0, sizeof(phase->metrology.dot_prod[0]));
    memset(&phase->metrology.current[0].dot_prod[dp], 0, sizeof(phase->metrology.current[0].dot_prod[0]));
#if NUM_PHASES == 1  &&  defined(NEUTRAL_MONITOR_SUPPORT)
    memset(&phase->metrology.current[1].dot_prod[dp], 0, sizeof(phase->metrology.current[1].dot_prod[0]));
#endif
}

#if NUM_PHASES > 1  &&  defined(NEUTRAL_MONITOR_SUPPORT)
/* Calculate the neutral readings for a 3-phase meter with neutral monitoring. */
void calculate_neutral_readings(void)
{
    int dp;

    #if defined(IRMS_SUPPORT)
    working_data.neutral.readings.I_rms = evaluate_neutral_rms_current();
    #endif
    #if defined(RESIDUAL_IRMS_SUPPORT)
    working_data.neutral.readings.residual_I_rms = evaluate_residual_3phase_rms_current();
    #endif
    dp = working_data.neutral.metrology.dp_set;
    memset(&working_data.neutral.metrology.dot_prod[dp], 0, sizeof(working_data.neutral.metrology.dot_prod[0]));
}
#endif

int align_metrology_with_calibration_data(void)
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
        set_sd_phase_correction(&phase->metrology.current[0].in_phase_correction, ph, phase_cal->current[0].phase_correction);
#if NUM_PHASES == 1  &&  defined(NEUTRAL_MONITOR_SUPPORT)
    set_sd_phase_correction(&phase->metrology.current[1].in_phase_correction, NUM_PHASES, phase_cal->current[1].phase_correction);
#endif
    return 0;
}
