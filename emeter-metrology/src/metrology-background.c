/*******************************************************************************
 *  metrology-background.c -
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

extern volatile unsigned long Contador4096;

#include <emeter-toolkit.h>

#include "emeter-metrology.h"

#include "emeter-metrology-internal.h"

#if !defined(NULL)
#define NULL    (void *) 0
#endif

#if NUM_PHASES > 1  &&  defined(__MSP430_HAS_SD24_B3__)  &&  defined(__MSP430_HAS_ADC10_A__)
#define PHASE_1_DELAY_SPLIT     2
#define PHASE_2_DELAY_SPLIT     2
#define PHASE_3_DELAY_SPLIT     2
#else
#define PHASE_1_DELAY_SPLIT     15
#define PHASE_4_DELAY_SPLIT     128
#define PHASE_5_DELAY_SPLIT     128
#define PHASE_6_DELAY_SPLIT     128
#define NEUTRAL_DELAY_SPLIT     128
#endif

#define ADC_VOLT_NEG_TRESHOLD   338
#define ADC_VOLT_POS_TRESHOLD   344

int16_t samples_per_second;

#if defined(TEMPERATURE_SUPPORT)
int32_t raw_temperature_from_adc = 0;
int temperature_sequence = 0;
    #if defined(TRNG_SUPPORT)
uint16_t random_value;
int new_random_value = false;
uint16_t rolling_random;
int rand_bits = 0;
    #endif
#endif

int16_t vcc;
int16_t auxadc;

#if NUM_PHASES > 1  &&  defined(__MSP430_HAS_SD24_B3__)  &&  defined(__MSP430_HAS_ADC10_A__)
/* This is a 3-phase meter, using the ADC10A for the voltage channels. These channels must be
   read through DMA. */
int16_t dma_adc_buffer[16];
#endif

#if defined(__HAS_SD_ADC__)
    #if defined(VOLTAGE_SIGNAL_IS_COMMON)
#define VOLTAGE_CHANNELS 1
    #else
#define VOLTAGE_CHANNELS NUM_PHASES
    #endif
    #if defined(NEUTRAL_MONITOR_SUPPORT)
#define CURRENT_CHANNELS (NUM_PHASES + 1)
    #else
#define CURRENT_CHANNELS NUM_PHASES
    #endif
#endif

/* These are the buffers for one set of samples */
static voltage_sample_t adc_v_buffer[VOLTAGE_CHANNELS];
static current_sample_t adc_i_buffer[CURRENT_CHANNELS];
//static uint8_t adc_i_available;

static __inline__ int32_t abs32(int32_t x)
{
    return (x < 0)  ?  -x  :  x;
}

#if defined(TRNG_SUPPORT)
int trng(uint16_t *val)
{
    if (!new_random_value)
        return -1;
    /* We now know the random value has been refreshed, so we can pick up a new one. */
    *val = random_value;
    /* There is a race here, where we may loose a generated number, but there will be more
       along fairly soon, so its fairly harmless if we loose one. */
    new_random_value = false;
    return 0;
}

uint16_t trng_wait(void)
{
    uint16_t val;

    while (!new_random_value)
        __no_operation();
    /* We now know the random value has been refreshed, so we can pick up a new one. */
    val = random_value;
    /* There is a race here, where we may loose a generated number, but there will be more
       along fairly soon, so its fairly harmless if we loose one. */
    new_random_value = false;
    return val;
}
#endif

#if NUM_PHASES == 1
static void __inline__ log_parameters(void)
{
#else
static void __inline__ log_parameters(int ph)
{
    struct phase_parms_s *phase = &working_data.phases[ph];
#endif
#if NUM_PHASES == 1  &&  defined(NEUTRAL_MONITOR_SUPPORT)
    int ch;
    static const uint16_t current_overrange_masks[PER_PHASE_CURRENT_CHANNELS] = {PHASE_STATUS_I_OVERRANGE, PHASE_STATUS_I_NEUTRAL_OVERRANGE};
#else
    static const uint16_t current_overrange_masks[PER_PHASE_CURRENT_CHANNELS] = {PHASE_STATUS_I_OVERRANGE};
#endif

#if defined(__MSP430__)  &&  defined(__MSP430_HAS_ADC10_A__)  &&  !(NUM_PHASES > 1  &&  defined(__MSP430_HAS_SD24_B3__))
    if (ph == 0)
        ADC10CTL0 |= ADC10SC;
#endif
    /* Take a snapshot of various values for logging purposes; tell the
       foreground to deal with them; and clear the working values ready
       for the next analysis period. */
    if (phase->metrology.V_endstops <= 0)
        phase->status |= PHASE_STATUS_V_OVERRANGE;
    else
        phase->status &= ~PHASE_STATUS_V_OVERRANGE;
    phase->metrology.V_endstops = ENDSTOP_HITS_FOR_OVERLOAD;
    /* Snapshot the DC estimate here, near a zero crossing */
    phase->metrology.V_dc_estimate_logged = dc_filter_voltage_estimate(phase->metrology.V_dc_estimate[normal_limp]);
#if NUM_PHASES > 1  &&  defined(FUNDAMENTAL_VRMS_SUPPORT)
    /* Point ph at the previous phase, for differencing */
    if (--ph < 0)
        ph = 2;
    phase->readings.phase_to_phase_angle = (phase->metrology.pure_phase >> 16)
                                         - (working_data.phases[ph].metrology.pure_phase >> 16);
#endif

#if NUM_PHASES == 1  &&  defined(NEUTRAL_MONITOR_SUPPORT)
    for (ch = 0;  ch < 2;  ch++)
#endif
    {
        if (phase->metrology.current[ch].I_endstops <= 0)
            phase->status |= current_overrange_masks[ch];
        else
            phase->status &= ~current_overrange_masks[ch];
        phase->metrology.current[ch].I_endstops = ENDSTOP_HITS_FOR_OVERLOAD;
        /* Snapshot the DC estimate here, near a zero crossing */
        phase->metrology.current[ch].I_dc_estimate_logged =
            dc_filter_current_estimate(phase->metrology.current[ch].I_dc_estimate[normal_limp]);
    }
    /* Tell the foreground there are things to process, and swap the dot product sets. */
    phase->metrology.dp_set ^= 1;
    phase->status |= PHASE_STATUS_NEW_LOG;
}

#if NUM_PHASES > 1  &&  defined(NEUTRAL_MONITOR_SUPPORT)  &&  defined(IRMS_SUPPORT)
/* This routine logs neutral lead information for poly-phase meters. It is
   not used for single phase meters with neutral monitoring. */
static void __inline__ log_neutral_parameters(void)
{
    if (working_data.neutral.metrology.I_endstops <= 0)
        working_data.neutral.status |= PHASE_STATUS_I_OVERRANGE;
    else
        working_data.neutral.status &= ~PHASE_STATUS_I_OVERRANGE;
    working_data.neutral.metrology.I_endstops = ENDSTOP_HITS_FOR_OVERLOAD;
    working_data.neutral.metrology.I_dc_estimate_logged =
        dc_filter_current_estimate(working_data.neutral.metrology.I_dc_estimate[normal_limp]);
    /* Tell the foreground there are things to process, and swap the dot product sets. */
    working_data.neutral.metrology.dp_set ^= 1;
    working_data.neutral.status |= PHASE_STATUS_NEW_LOG;
}
#endif

static __inline__ int per_sample_dsp(void)
{
    int kick;
    voltage_sample_t V_sample;
    voltage_sample_t V_corrected;
#if defined(REACTIVE_POWER_BY_QUADRATURE_SUPPORT)
    voltage_sample_t V_quad_corrected;
#endif
#if defined(FUNDAMENTAL_POWER_SUPPORT)
    voltage_sample_t V_pure;
    voltage_sample_t V_quad_pure;
    int32_t summy;
#endif
    current_sample_t I_sample[PER_PHASE_CURRENT_CHANNELS];
    current_sample_t I_corrected;
#if defined(RESIDUAL_IRMS_SUPPORT)
    int32_t I_residue;
#endif
#if NUM_PHASES > 1
    struct phase_parms_s *phase;
    int ph;
#endif
#if defined(LIMP_MODE_SUPPORT)
    #if NUM_PHASES == 1  &&  defined(NEUTRAL_MONITOR_SUPPORT)
    static const uint16_t current_pos_masks[PER_PHASE_CURRENT_CHANNELS] = {PHASE_STATUS_I_POS, PHASE_STATUS_I_NEUTRAL_POS};
    #else
    static const uint16_t current_pos_masks[PER_PHASE_CURRENT_CHANNELS] = {PHASE_STATUS_I_POS};
    #endif
#endif
#if NUM_PHASES == 1  &&  defined(NEUTRAL_MONITOR_SUPPORT)
    int ch;
#endif
    int k;
#if defined(MAINS_FREQUENCY_SUPPORT)
    int x;
    int y;
    int z;
#endif
    struct phase_dot_prod_set_s *phase_dot_products;
    struct current_sensor_dot_prod_set_s *sensor_dot_products;
#if NUM_PHASES > 1  &&  defined(NEUTRAL_MONITOR_SUPPORT)
    struct neutral_dot_prod_set_s *neutral_dot_products;
#endif
    int8_t loggers;
    int dp;

    /* Filter away the DC bias.
    
       Do the phase lag compensation. Use a simple FIR approach,
       and absorb the non-unity gain of the filter in the overall
       current/power scaling later on. This is OK for the small
       phase shifts we expect to get. It would cause dynamic
       range problems for larger shifts. Note the some of this
       phase shift is due to the operation of the ADC itself. It
       performs sequential conversions of its 8 inputs, so there is
       some time delay between sampling of the various sensors.
    
       Accumulate power for each of the channels. These will
       be divided by the number of samples at the end of the
       measurement cycles, resulting in an average power
       value for each source.

       If RMS voltage and/or current readings are required, calculate the
       dot products needed to evaluate these. */

    kick = true;
#if defined(RESIDUAL_IRMS_SUPPORT)
    I_residue = 0;
#endif

    loggers = 0;
#if NUM_PHASES > 1
    for (ph = 0, phase = working_data.phases;  ph < NUM_PHASES;  ph++, phase++)
#endif
    {
        dp = phase->metrology.dp_set ^ 1;
        phase_dot_products = &phase->metrology.dot_prod[dp];

#if defined(VOLTAGE_SIGNAL_IS_COMMON)
        V_sample = adc_v_buffer[0];
#else
        V_sample = adc_v_buffer[ph];
#endif
        if ((V_sample >= V_ADC_MAX  ||  V_sample <= V_ADC_MIN)  &&  phase->metrology.V_endstops)
            phase->metrology.V_endstops--;
        V_sample = dc_filter_voltage(phase->metrology.V_dc_estimate[normal_limp], V_sample);

#if defined(VRMS_SUPPORT)  ||  defined(POWER_FACTOR_SUPPORT)
    #if defined(SAG_SWELL_SUPPORT)
        sqac_voltage(phase->metrology.V_sq_cycle, V_sample);
    #else
        sqac_voltage(phase_dot_products->V_sq, V_sample);
    #endif
#endif
        ++phase_dot_products->sample_count;

        /* We need to save the history of the voltage signal if we are performing phase correction, and/or
           measuring the quadrature shifted power (to obtain an accurate measure of one form of the reactive power). */
        phase->metrology.V_history[(int) phase->metrology.V_history_index] = V_sample;

        if (operating_mode == OPERATING_MODE_NORMAL)
        {
            /* Perform bulk delay (i.e. integer sample times) of the voltage signal. */
            V_corrected = phase->metrology.V_history[(phase->metrology.V_history_index - phase->metrology.current[0].in_phase_correction.step) & V_HISTORY_MASK];
#if defined(FUNDAMENTAL_POWER_SUPPORT)
            /* The dot product of the raw and the pure voltage signals allows us to precisely estimate
               the amplitude of the fundamental component of the mains voltage waveform. This is needed,
               during the foreground processing, to correctly scale the answer from the dot product of the
               full scale pure voltage waveform and the current signal.
               The answer from this estimator will only be correct once the pure waveform is properly phase
               locked. */
            V_pure = dds_lookup(phase->metrology.pure_phase);
            mac_voltage(phase_dot_products->V_fundamental, V_corrected, V_pure);

            /* If we look for maximum correlation when the real and synthetic waveforms are in sync:
                    - the sensitivity to errors is not that big around the match
                    - we don't know what the peak should be
                    - we don't know which side of the peak we are.
               If we look for minimum correlation in quadrature signals we solve all three issues in one go. */
            /* Cross correlate the real voltage signal with the synthesised quadrature one, and tune the phase to
               minimise the correlation. This assumes the phase rate is being accurately derived from the mains
               frequency measurement, and we only need to adjust the phase here. This is a sort of PLL, with the
               frequency and phase aspects of the lock being seperately evaluated. */
            V_quad_pure = dds_lookup(phase->metrology.pure_phase + 0x40000000);
            summy = imul16(V_corrected, V_quad_pure);
            /* We need to filter hard at this point, to massively suppress the harmonics. We do this with a single
               pole with a very low turnover point. Obviously, this only tails off at 6dB/octave, so the downside
               it a very slow pull-in during the initial phase locking. */
            phase->metrology.cross_sum += ((summy - phase->metrology.cross_sum) >> 13);
            phase->metrology.pure_phase += (phase->metrology.cross_sum >> 5);
            phase->metrology.pure_phase += phase->metrology.pure_phase_rate;
#endif
        }

#if NUM_PHASES == 1  &&  defined(NEUTRAL_MONITOR_SUPPORT)
        for (ch = 0;  ch < 2;  ch++)
        {
            I_corrected = adc_i_buffer[ch];
#else
        {
            I_corrected = adc_i_buffer[ph];
#endif
            sensor_dot_products = &phase->metrology.current[ch].dot_prod[dp];
            if ((I_corrected >= I_ADC_MAX  ||  I_corrected <= I_ADC_MIN)  &&  phase->metrology.current[ch].I_endstops)
                phase->metrology.current[ch].I_endstops--;
            I_sample[ch] = dc_filter_current(phase->metrology.current[ch].I_dc_estimate[normal_limp], phase->metrology.current[ch].I_history[0]);
#if I_HISTORY_STEPS > 2
            for (k = 0;  k < I_HISTORY_STEPS - 1;  k++)
                phase->metrology.current[ch].I_history[k] = phase->metrology.current[ch].I_history[k + 1];
#else
            phase->metrology.current[ch].I_history[0] = phase->metrology.current[ch].I_history[1];
#endif
            phase->metrology.current[ch].I_history[I_HISTORY_STEPS - 1] = I_corrected;
#if defined(RESIDUAL_IRMS_SUPPORT)
            I_residue += I_sample[ch];
#endif
#if defined(IRMS_SUPPORT)  ||  defined(POWER_FACTOR_SUPPORT)
            sqac_current(sensor_dot_products->I_sq, I_sample[ch]);
#endif
            if (operating_mode == OPERATING_MODE_NORMAL)
            {
                /* Perform bulk delay (i.e. integer sample times) of the voltage signal. */
                V_corrected = phase->metrology.V_history[(phase->metrology.V_history_index - phase->metrology.current[ch].in_phase_correction.step) & V_HISTORY_MASK];
                mac_power(sensor_dot_products->P_active, V_corrected, I_sample[ch]);
#if defined(REACTIVE_POWER_BY_QUADRATURE_SUPPORT)
                V_quad_corrected = (q1_15_mul(phase->metrology.V_history[(phase->metrology.V_history_index - phase->metrology.current[ch].quadrature_correction.step - 1) & V_HISTORY_MASK], phase->metrology.current[ch].quadrature_correction.fir_beta) >> 1)
                                 + (phase->metrology.V_history[(phase->metrology.V_history_index - phase->metrology.current[ch].quadrature_correction.step) & V_HISTORY_MASK] >> 1);
                mac_power(sensor_dot_products->P_reactive, V_quad_corrected, I_sample[ch]);
#endif
#if defined(FUNDAMENTAL_ACTIVE_POWER_SUPPORT)
                mac_power(sensor_dot_products->P_fundamental_active, V_pure, I_sample[ch]);
#endif
#if defined(FUNDAMENTAL_REACTIVE_POWER_SUPPORT)
                mac_power(sensor_dot_products->P_fundamental_reactive, -V_quad_pure, I_sample[ch]);
#endif
            }
            ++sensor_dot_products->sample_count;
        }

        phase->metrology.V_history_index = (phase->metrology.V_history_index + 1) & V_HISTORY_MASK;

        /* Do the power cycle start detection */
        /* There is no hysteresis used here, but since the signal is
           changing rapidly at the zero crossings, and is always of
           large amplitude, miscounting cycles due to general noise
           should not occur. Spikes are another matter. A large spike
           could cause the power cycles to be miscounted, but does not
           matter very much. The cycle counting is not critical to power
           or energy measurement. */
#if defined(MAINS_FREQUENCY_SUPPORT)
    #if defined(LIMP_MODE_SUPPORT)
        if (operating_mode == OPERATING_MODE_LIMP)
            phase->metrology.voltage_period.cycle_samples += 256*LIMP_SAMPLING_RATIO;
        else
    #endif
            phase->metrology.voltage_period.cycle_samples += 256;
#endif
        if (abs(V_corrected - phase->metrology.last_V_sample) <= phase->metrology.since_last*MAX_PER_SAMPLE_VOLTAGE_SLEW)
        {
            /* This doesn't look like a spike - do mains cycle detection, and
               estimate the precise mains period */
#if defined(SAG_POWER_DOWN_SUPPORT)
            if (abs(V_corrected) > phase->metrology.sag_power_down_threshold)
            {
                phase->metrology.samples_since_last_sag_power_down_threshold_crossing = 0;
                //sag_swell_event(ph, SAG_SWELL_VOLTAGE_POWER_DOWN_OK);
            }
            else
            {
                if (++phase->metrology.samples_since_last_sag_power_down_threshold_crossing > SAMPLES_PER_10_SECONDS*SAG_POWER_DOWN_DURATION/10000L)
                    sag_swell_event(ph, SAG_SWELL_VOLTAGE_POWER_DOWN_SAG);
            }
#endif
            if (V_corrected < 0)
            {
                /* We just crossed from positive to negative */
                /* Log the sign of the signal */
                phase->status &= ~PHASE_STATUS_V_POS;
            }
            else
            {
                /* We just crossed from negative to positive */
                if (!(phase->status & PHASE_STATUS_V_POS))
                {
#if defined(MAINS_FREQUENCY_SUPPORT)
                    /* Apply limits to the sample count, to avoid spikes or dying power lines disturbing the
                       frequency reading too much */
                    /* The mains should be <40Hz or >70Hz to fail this test! */
                    if (256*SAMPLES_PER_10_SECONDS/700 <= phase->metrology.voltage_period.cycle_samples
                        &&
                        phase->metrology.voltage_period.cycle_samples <= 256*SAMPLES_PER_10_SECONDS/400)
                    {
                        /* A mains frequency measurement procedure based on interpolating zero crossings,
                           to get a fast update rate for step changes in the mains frequency */
    #if defined(SAG_SWELL_SUPPORT)
                        accum48_48(phase_dot_products->V_sq, phase->metrology.V_sq_cycle);
                        decum48_48(phase->metrology.V_sq_window, phase->metrology.V_sq_prev_cycle[phase->metrology.prev_cycle_ptr]);
                        accum48_48(phase->metrology.V_sq_window, phase->metrology.V_sq_cycle);
                        transfer48(phase->metrology.V_sq_prev_cycle[phase->metrology.prev_cycle_ptr], phase->metrology.V_sq_cycle);
                        if (++phase->metrology.prev_cycle_ptr >= SAG_SWELL_WINDOW_LEN)
                            phase->metrology.prev_cycle_ptr = 0;
                        if (phase->metrology.V_sq_window[2] < phase->metrology.sag_threshold[2]
                            ||
                               (phase->metrology.V_sq_window[2] == phase->metrology.sag_threshold[2]
                                &&
                                (uint16_t) phase->metrology.V_sq_window[1] < (uint16_t) phase->metrology.sag_threshold[1]))
                        {
                            if (phase->metrology.sag_status != -1)
                            {
                                phase->metrology.sag_status = -1;
                                sag_swell_event(ph, SAG_SWELL_VOLTAGE_SAG_ONSET);
                            }
                            else
                            {
                                sag_swell_event(ph, SAG_SWELL_VOLTAGE_SAG_CONTINUING);
                            }
                        }
                        else if (phase->metrology.V_sq_window[2] > phase->metrology.swell_threshold[2]
                                 ||
                                    (phase->metrology.V_sq_window[2] == phase->metrology.swell_threshold[2]
                                     &&
                                     (uint16_t) phase->metrology.V_sq_window[1] > (uint16_t) phase->metrology.swell_threshold[1]))
                        {
                            if (phase->metrology.sag_status != 1)
                            {
                                phase->metrology.sag_status = 1;
                                sag_swell_event(ph, SAG_SWELL_VOLTAGE_SWELL_ONSET);
                            }
                            else
                            {
                                sag_swell_event(ph, SAG_SWELL_VOLTAGE_SWELL_CONTINUING);
                            }
                        }
                        else
                        {
                            if (phase->metrology.sag_status != 0)
                                sag_swell_event(ph, SAG_SWELL_VOLTAGE_NORMAL);
                            phase->metrology.sag_status = 0;
                        }
    #endif
                        /* Interpolate the zero crossing by successive approx. Its faster than just dividing. */
                        z = V_corrected - phase->metrology.last_V_sample;
                        x = 0;
                        y = 0;
                        for (k = 0;  k < 8;  k++)
                        {
                            y <<= 1;
                            z >>= 1;
                            x += z;
                            if (x > V_corrected)
                                x -= z;
                            else
                                y |= 1;
                        }
                        /* Now we need to allow for skipped samples, due to spike detection */
                        z = y;
                        while (phase->metrology.since_last > 1)
                        {
                            z += y;
                            phase->metrology.since_last--;
                        }
                        /* z is now the fraction of a sample interval between the zero
                           crossing and the current sample, in units of 1/256 of a sample */
                        /* A lightly damped single pole filter should now be enough to remove noise and get a
                           stable value for the frequency */
                        phase->metrology.voltage_period.period += ((int32_t) (phase->metrology.voltage_period.cycle_samples - z) << 12) - (phase->metrology.voltage_period.period >> 4);
                        /* Start the next cycle with the residual fraction of a sample */
                        phase->metrology.voltage_period.cycle_samples = z;
                    }
                    else
                    {
                        phase->metrology.voltage_period.cycle_samples = 0;
                    }
#endif
#if defined(POWER_FACTOR_SUPPORT)  &&  !defined(REACTIVE_POWER_BY_QUADRATURE_SUPPORT)
                    /* If we are not measuring the reactive power in a quadrature manner, we
                       need to work out if the current leads or lags the voltage in another way. */
                    /* Determine whether the current leads or lags, in a noise tolerant manner.
                       Testing 50 cycles means we will respond in about one second to a genuine
                       swap between lead and lag. Since that is also about the length of our
                       measurement blocks, this seems a sensible response time. */
#if NUM_PHASES == 1  &&  defined(NEUTRAL_MONITOR_SUPPORT)
                    for (ch = 0;  ch < 2;  ch++)
#endif
                    {
                        if (I_sample[ch] < V_corrected)
                        {
                            if (phase->metrology.current[ch].leading > -50)
                                phase->metrology.current[ch].leading--;
                        }
                        else
                        {
                            if (phase->metrology.current[ch].leading < 50)
                                phase->metrology.current[ch].leading++;
                        }
                    }
#endif
                    /* See if a sufficiently long measurement interval has been
                       recorded, and catch the start of the next cycle. We do not
                       really care how many cycles there are, as long as the block
                       is a reasonable length. Setting a minimum of 1 second is
                       better than counting cycles, as it is not affected by noise
                       spikes. Synchronising to a whole number of cycles reduces
                       block to block jitter, though it doesn't affect the long
                       term accuracy of the measurements. */
                    if (phase_dot_products->sample_count >= samples_per_second)
                        loggers |= 1;
                }
                /* Log the sign of the signal */
                phase->status |= PHASE_STATUS_V_POS;
            }
            phase->metrology.since_last = 0;
            phase->metrology.last_V_sample = V_corrected;
        }
        phase->metrology.since_last++;

        if (phase_dot_products->sample_count >= samples_per_second + 2*SAMPLE_RATE/MAINS_NOMINAL_FREQUENCY)
        {
            /* We don't seem to be detecting the end of a mains cycle, so force
               the end of processing block condition. */
            loggers |= 1;
        }
#if defined(LIMP_MODE_SUPPORT)  &&  defined(MAINS_FREQUENCY_SUPPORT)
    #if NUM_PHASES == 1  &&  defined(NEUTRAL_MONITOR_SUPPORT)
        for (ch = 0;  ch < 2;  ch++)
    #endif
        {
            /* Monitor the cycles and frequency of the current sensors, as limp
               mode is based on these. */
    #if defined(LIMP_MODE_SUPPORT)
            if (operating_mode == OPERATING_MODE_LIMP)
                phase->metrology.current[ch].period.cycle_samples += 256*LIMP_SAMPLING_RATIO;
            else
    #endif
                phase->metrology.current[ch].period.cycle_samples += 256;
            if (I_sample[ch] < 0)
            {
                /* We just crossed from positive to negative */
                /* Log the sign of the signal */
                phase->status &= ~current_pos_masks[ch];
            }
            else
            {
                /* We just crossed from negative to positive */
                if (!(phase->status & current_pos_masks[ch]))
                {
                    /* A negative to positive transition has occurred. Trust it
                       blindly as a genuine zero crossing/start of cycle, even
                       though it might really be due to a noise spike. */
                    if (256*SAMPLES_PER_10_SECONDS/700 <= phase->metrology.current[ch].period.cycle_samples
                        &&
                        phase->metrology.current[ch].period.cycle_samples <= 256*SAMPLES_PER_10_SECONDS/400)
                    {
                        phase->metrology.current[ch].period.period += ((int32_t) phase->metrology.current[ch].period.cycle_samples << 8) - (phase->metrology.current[ch].period.period >> 8);
                    }
                    phase->metrology.current[ch].period.cycle_samples = 0;
                }
                /* Log the sign of the signal */
                phase->status |= current_pos_masks[ch];
            }
        }
#endif
#if NUM_PHASES > 1
        loggers <<= 1;
#endif
    }
    if (loggers)
    {
        /* There are one or more phases to be logged */
#if NUM_PHASES == 1
        log_parameters();
#else
        for (ph = 0;  ph < NUM_PHASES;  ph++)
        {
            if (loggers & (1 << NUM_PHASES))
                log_parameters(ph);
            loggers <<= 1;
        }
#endif
        kick = true;
    }

#if NUM_PHASES > 1  &&  defined(NEUTRAL_MONITOR_SUPPORT)  &&  defined(IRMS_SUPPORT)
    /* For multi-phase meters, neutral monitoring is limited to measuring the
       neutral RMS current, and the residual RMS current. */
    dp = working_data.neutral.metrology.dp_set ^ 1;
    neutral_dot_products = &working_data.neutral.metrology.dot_prod[dp];
    I_corrected = adc_i_buffer[NUM_PHASES];
    #if defined(TEMPERATURE_SUPPORT)  &&  !defined(__MSP430_HAS_ADC10_A__)
    if (temperature_sequence)
    {
        temperature_sequence++;
        if (temperature_sequence == 6)
        {
            /* We are in temperature measurement mode */
            raw_temperature_from_adc += I_corrected - (raw_temperature_from_adc >> 3);
        #if defined(TRNG_SUPPORT)
            rolling_random = (rolling_random << 1) | (I_corrected & 1);
            if (++rand_bits == 16)
            {
                random_value = rolling_random;
                new_random_value = true;
                rand_bits = 0;
            }
        #endif
            /* Select the current input */
            sd_xxxx_reg(SD_INCTL_, NEUTRAL_CURRENT_ADC_CHANNEL) = SD_INCH_CURRENT | SD_NEUTRAL_CURRENT_GAIN;
        }
        else if (temperature_sequence == 11)
        {
            /* We have waited long enough to be back on good current samples */
            temperature_sequence = 0;
        }
        /* TODO: Can't we do better than just stalling on the last value? */
        I_corrected = working_data.neutral.metrology.I_history[I_HISTORY_STEPS - 1];
    }
    else
    #endif
    {
        if ((I_corrected >= I_ADC_MAX  ||  I_corrected <= I_ADC_MIN)  &&  working_data.neutral.metrology.I_endstops)
            working_data.neutral.metrology.I_endstops--;
        I_sample[0] = dc_filter_current(working_data.neutral.metrology.I_dc_estimate[normal_limp], working_data.neutral.metrology.I_history[0]);
        sqac_current(neutral_dot_products->I_sq, I_sample[0]);
    #if defined(RESIDUAL_IRMS_SUPPORT)
        /* Scale the current from the neutral by 1/sqrt(2) */
        I_sample[0] = q1_15_mul(I_sample[0], 21790);
        I_sample[0] -= I_residue;
        sqac_current(neutral_dot_products->residual_I_sq, I_sample[0]);
    #endif
    }
    if (++neutral_dot_products->sample_count >= samples_per_second)
    {
        log_neutral_parameters();
    #if defined(TEMPERATURE_SUPPORT)  &&  !defined(__MSP430_HAS_ADC10_A__)
        temperature_sequence = 1;
        /* Select the temperature diode */
        sd_xxxx_reg(SD_INCTL_, NEUTRAL_CURRENT_ADC_CHANNEL) = SD_INCH_TEMPERATURE | SD_GAIN_TEMPERATURE;
    #endif
    }
    #if I_HISTORY_STEPS > 2
    for (k = 0;  k < I_HISTORY_STEPS - 1;  k++)
        working_data.neutral.metrology.I_history[k] = working_data.neutral.metrology.I_history[k + 1];
    #else
    working_data.neutral.metrology.I_history[0] = working_data.neutral.metrology.I_history[1];
    #endif
    working_data.neutral.metrology.I_history[I_HISTORY_STEPS - 1] = I_corrected;
#endif
    return kick;
}

#if defined(ENERGY_PULSE_SUPPORT)  ||  defined(TOTAL_ENERGY_PULSE_SUPPORT)
static __inline__ void per_sample_energy_pulse_processing(void)
{
#if defined(ENERGY_PULSE_SUPPORT)  &&  NUM_PHASES > 1
    struct phase_parms_s *phase;
    int ph;
#endif
    power_t pow;

    /* We now play the last measurement interval's power level, evaluated
       in the foreground, through this measurement interval. In this way
       we can evenly pace the pulsing of the LED. The only error produced
       by this is the ambiguity in the number of samples per measurement.
       This should not exceed 1 or 2 in over 4000. */

#if defined(TOTAL_ACTIVE_ENERGY_PULSES_PER_KW_HOUR)
    pow = working_data.totals.readings.active_power;
    #if defined(LIMP_MODE_SUPPORT)
    if (operating_mode == OPERATING_MODE_LIMP)
        pow *= LIMP_SAMPLING_RATIO;
    #endif
    #if defined(INHIBIT_NEGATIVE_TOTAL_POWER_ACCUMULATION)
    if (pow > 0)
    #else
    pow = abs32(pow);
    #endif
    {
        if ((working_data.totals.energy.active_energy_pulse.energy_integrator += pow) >= TOTAL_ACTIVE_ENERGY_PULSE_THRESHOLD)
        {
            working_data.totals.energy.active_energy_pulse.energy_integrator -= TOTAL_ACTIVE_ENERGY_PULSE_THRESHOLD;
            working_data.totals.energy.active_energy_pulse.pulse_remaining_time = ENERGY_PULSE_DURATION;
            active_energy_pulse_start(FAKE_PHASE_TOTAL);
        }
    }
    if (working_data.totals.energy.active_energy_pulse.pulse_remaining_time
        &&
        --working_data.totals.energy.active_energy_pulse.pulse_remaining_time == 0)
    {
        active_energy_pulse_end(FAKE_PHASE_TOTAL);
    }
#endif

#if defined(TOTAL_REACTIVE_ENERGY_PULSES_PER_KVAR_HOUR)
    pow = working_data.totals.readings.reactive_power;
    #if defined(LIMP_MODE_SUPPORT)
    if (operating_mode == OPERATING_MODE_LIMP)
        pow *= LIMP_SAMPLING_RATIO;
    #endif
    #if defined(INHIBIT_NEGATIVE_TOTAL_POWER_ACCUMULATION)
    if (pow > 0)
    #else
    pow = abs32(pow);
    #endif
    {
        if ((working_data.totals.energy.reactive_energy_pulse.energy_integrator += pow) >= TOTAL_REACTIVE_ENERGY_PULSE_THRESHOLD)
        {
            working_data.totals.energy.reactive_energy_pulse.energy_integrator -= TOTAL_REACTIVE_ENERGY_PULSE_THRESHOLD;
            working_data.totals.energy.reactive_energy_pulse.pulse_remaining_time = ENERGY_PULSE_DURATION;
            reactive_energy_pulse_start(FAKE_PHASE_TOTAL);
        }
    }
    if (working_data.totals.energy.reactive_energy_pulse.pulse_remaining_time
        &&
        --working_data.totals.energy.reactive_energy_pulse.pulse_remaining_time == 0)
    {
        reactive_energy_pulse_end(FAKE_PHASE_TOTAL);
    }
#endif

#if defined(TOTAL_APPARENT_ENERGY_PULSES_PER_KVA_HOUR)
    pow = working_data.totals.readings.apparent_power;
    #if defined(LIMP_MODE_SUPPORT)
    if (operating_mode == OPERATING_MODE_LIMP)
        pow *= LIMP_SAMPLING_RATIO;
    #endif
    #if defined(INHIBIT_NEGATIVE_TOTAL_POWER_ACCUMULATION)
    if (pow > 0)
    #else
    pow = abs32(pow);
    #endif
    {
        if ((working_data.totals.energy.apparent_energy_pulse.energy_integrator += pow) >= TOTAL_APPARENT_ENERGY_PULSE_THRESHOLD)
        {
            working_data.totals.energy.apparent_energy_pulse.energy_integrator -= TOTAL_APPARENT_ENERGY_PULSE_THRESHOLD;
            working_data.totals.energy.apparent_energy_pulse.pulse_remaining_time = ENERGY_PULSE_DURATION;
            apparent_energy_pulse_start(FAKE_PHASE_TOTAL);
        }
    }
    if (working_data.totals.energy.apparent_energy_pulse.pulse_remaining_time
        &&
        --working_data.totals.energy.apparent_energy_pulse.pulse_remaining_time == 0)
    {
        apparent_energy_pulse_end(FAKE_PHASE_TOTAL);
    }
#endif

#if defined(ENERGY_PULSE_SUPPORT)
    #if NUM_PHASES == 1
#undef ph
#define ph /**/
    #else
    for (ph = 0, phase = working_data.phases;  ph < NUM_PHASES;  ph++, phase++)
    #endif
    {
    #if defined(ACTIVE_ENERGY_PULSES_PER_KW_HOUR)
        pow = phase->readings.active_power;
        #if defined(LIMP_MODE_SUPPORT)
        if (operating_mode == OPERATING_MODE_LIMP)
            pow *= LIMP_SAMPLING_RATIO;
        #endif
        #if defined(INHIBIT_NEGATIVE_POWER_ACCUMULATION)
        if (pow > 0)
        #else
        pow = abs32(pow);
        #endif
        {
            if ((phase->energy.active_energy_pulse.energy_integrator += pow) >= ACTIVE_ENERGY_PULSE_THRESHOLD)
            {
                phase->energy.active_energy_pulse.energy_integrator -= ACTIVE_ENERGY_PULSE_THRESHOLD;
                phase->energy.active_energy_pulse.pulse_remaining_time = ENERGY_PULSE_DURATION;
                active_energy_pulse_start(ph);
            }
        }
        if (phase->energy.active_energy_pulse.pulse_remaining_time
            &&
            --phase->energy.active_energy_pulse.pulse_remaining_time == 0)
        {
            active_energy_pulse_end(ph);
        }
    #endif

    #if defined(REACTIVE_ENERGY_PULSES_PER_KVAR_HOUR)
        pow = phase->readings.reactive_power;
        #if defined(LIMP_MODE_SUPPORT)
        if (operating_mode == OPERATING_MODE_LIMP)
            pow *= LIMP_SAMPLING_RATIO;
        #endif
        #if defined(INHIBIT_NEGATIVE_POWER_ACCUMULATION)
        if (pow > 0)
        #else
        pow = abs32(pow);
        #endif
        {
            if ((phase->energy.reactive_energy_pulse.energy_integrator += pow) >= REACTIVE_ENERGY_PULSE_THRESHOLD)
            {
                phase->energy.reactive_energy_pulse.energy_integrator -= REACTIVE_ENERGY_PULSE_THRESHOLD;
                phase->energy.reactive_energy_pulse.pulse_remaining_time = ENERGY_PULSE_DURATION;
                reactive_energy_pulse_start(ph);
            }
        }
        if (phase->energy.reactive_energy_pulse.pulse_remaining_time
            &&
            --phase->energy.reactive_energy_pulse.pulse_remaining_time == 0)
        {
            reactive_energy_pulse_end(ph);
        }
    #endif

    #if defined(APPARENT_ENERGY_PULSES_PER_KVA_HOUR)
        pow = phase->readings.apparent_power;
        #if defined(LIMP_MODE_SUPPORT)
        if (operating_mode == OPERATING_MODE_LIMP)
            pow *= LIMP_SAMPLING_RATIO;
        #endif
        #if defined(INHIBIT_NEGATIVE_POWER_ACCUMULATION)
        if (pow > 0)
        #else
        pow = abs32(pow);
        #endif
        {
            if ((phase->energy.apparent_energy_pulse.energy_integrator += pow) >= APPARENT_ENERGY_PULSE_THRESHOLD)
            {
                phase->energy.apparent_energy_pulse.energy_integrator -= APPARENT_ENERGY_PULSE_THRESHOLD;
                phase->energy.apparent_energy_pulse.pulse_remaining_time = ENERGY_PULSE_DURATION;
                apparent_energy_pulse_start(ph);
            }
        }
        if (phase->energy.reactive_energy_pulse.pulse_remaining_time
            &&
            --phase->energy.apparent_energy_pulse.pulse_remaining_time == 0)
        {
            apparent_energy_pulse_end(ph);
        }
    #endif
    }
    #if NUM_PHASES == 1
#undef ph
    #endif
#endif
}
#endif

/*-----------------------------------------------------------------------------------
  This is the main interrupt routine where the samples are gathered into a set, with
  allowance for the staggered times at which the samples become available.
  -----------------------------------------------------------------------------------*/
#if defined(__MSP430__)
    #if defined(__MSP430_HAS_SD16_2__)  ||  defined(__MSP430_HAS_SD16_3__)
ISR(SD16, adc_interrupt)
    #endif
    #if defined(__MSP430_HAS_SD16_A3__)  ||  defined(__MSP430_HAS_SD16_A4__)  ||  defined(__MSP430_HAS_SD16_A6__)  ||  defined(__MSP430_HAS_SD16_A7__)
ISR(SD16A, adc_interrupt)
    #endif
    #if defined(__MSP430_HAS_SD24__)  ||  defined(__MSP430_HAS_SD24_2__)  ||  defined(__MSP430_HAS_SD24_3__)  ||  defined(__MSP430_HAS_SD24_4__)  ||  defined(__MSP430_HAS_SD24_A2__)  ||  defined(__MSP430_HAS_SD24_A3__)
ISR(SD24, adc_interrupt)
    #endif
    #if defined(__MSP430_HAS_SD24_B__)
ISR(SD24B, adc_interrupt)
    #endif
#else
void adc_interrupt(void)
#endif
{
#if defined(__HAS_SD_ADC__)

    if (!ADC_VOLTAGE_PENDING(PHASE_1_VOLTAGE_ADC_CHANNEL))
    {
        /* We do not have a complete set of samples yet, but we may need to pick
           up some current values at this time */
        if (ADC_CURRENT_PENDING(PHASE_1_CURRENT_ADC_CHANNEL))         
          adc_i_buffer[0] = ADC_CURRENT(PHASE_1_CURRENT_ADC_CHANNEL);
    #if NUM_PHASES >= 2
        if (ADC_CURRENT_PENDING(PHASE_2_CURRENT_ADC_CHANNEL))
            adc_i_buffer[1] = ADC_CURRENT(PHASE_2_CURRENT_ADC_CHANNEL);
    #endif
    #if NUM_PHASES >= 3
        if (ADC_CURRENT_PENDING(PHASE_3_CURRENT_ADC_CHANNEL))
            adc_i_buffer[2] = ADC_CURRENT(PHASE_3_CURRENT_ADC_CHANNEL);
    #endif
    #if NUM_PHASES >= 4
        if (ADC_CURRENT_PENDING(PHASE_4_CURRENT_ADC_CHANNEL))
            adc_i_buffer[3] = ADC_CURRENT(PHASE_4_CURRENT_ADC_CHANNEL);
    #endif
    #if NUM_PHASES >= 5
        if (ADC_CURRENT_PENDING(PHASE_5_CURRENT_ADC_CHANNEL))
            adc_i_buffer[4] = ADC_CURRENT(PHASE_5_CURRENT_ADC_CHANNEL);
    #endif
    #if NUM_PHASES >= 6
        if (ADC_CURRENT_PENDING(PHASE_6_CURRENT_ADC_CHANNEL))
            adc_i_buffer[5] = ADC_CURRENT(PHASE_6_CURRENT_ADC_CHANNEL);
    #endif
    #if defined(NEUTRAL_MONITOR_SUPPORT)
        if (ADC_CURRENT_PENDING(NEUTRAL_CURRENT_ADC_CHANNEL))
            adc_i_buffer[NUM_PHASES] = ADC_CURRENT(NEUTRAL_CURRENT_ADC_CHANNEL);
    #endif
        return;
    }
    
    SD24BTRGCTL &= ~SD24TRGIFG;
    

    adc_v_buffer[0] = ADC_VOLTAGE(PHASE_1_VOLTAGE_ADC_CHANNEL);
    #if !defined(VOLTAGE_SIGNAL_IS_COMMON)
        #if NUM_PHASES >= 2
    adc_v_buffer[1] = ADC_VOLTAGE(PHASE_2_VOLTAGE_ADC_CHANNEL);
        #endif
        #if NUM_PHASES >= 3
    adc_v_buffer[2] = ADC_VOLTAGE(PHASE_3_VOLTAGE_ADC_CHANNEL);
        #endif
    #endif

    #if defined(__MSP430_HAS_SD24_B3__)  &&  NUM_PHASES > 1
        #if defined(TEMPERATURE_SUPPORT)
    raw_temperature_from_adc  = dma_adc_buffer[15 - 10] ;
        #endif
        #if defined(VCC_MEASURE_SUPPORT)
    vcc = dma_adc_buffer[15 - 11];
        #endif
        #if defined(__MSP430_HAS_AUX_SUPPLY__)
    auxadc = dma_adc_buffer[15 - 12];
        #endif
    #endif

    /* Pick up any current samples which may have occurred a little before the
       voltage sample, but not those which may have occurred just after the
       voltage sample. */
    if (working_data.phases[0].metrology.current[0].in_phase_correction.sd_preloaded_offset < PHASE_1_DELAY_SPLIT
        &&
        ADC_CURRENT_PENDING(PHASE_1_CURRENT_ADC_CHANNEL))
    {          
          adc_i_buffer[0] = ADC_CURRENT(PHASE_1_CURRENT_ADC_CHANNEL);
    }
    #if NUM_PHASES >= 2
    if (working_data.phases[1].metrology.current[0].in_phase_correction.sd_preloaded_offset < PHASE_2_DELAY_SPLIT
        &&
        ADC_CURRENT_PENDING(PHASE_2_CURRENT_ADC_CHANNEL))
    {
        adc_i_buffer[1] = ADC_CURRENT(PHASE_2_CURRENT_ADC_CHANNEL);
    }
    #endif
    #if NUM_PHASES >= 3
    if (working_data.phases[2].metrology.current[0].in_phase_correction.sd_preloaded_offset < PHASE_3_DELAY_SPLIT
        &&
        ADC_CURRENT_PENDING(PHASE_3_CURRENT_ADC_CHANNEL))
    {
        adc_i_buffer[2] = ADC_CURRENT(PHASE_3_CURRENT_ADC_CHANNEL);
    }
    #endif
    #if NUM_PHASES >= 4
    if (working_data.phases[3].metrology.current[0].in_phase_correction.sd_preloaded_offset < PHASE_4_DELAY_SPLIT
        &&
        ADC_CURRENT_PENDING(PHASE_4_CURRENT_ADC_CHANNEL))
    {
        adc_i_buffer[3] = ADC_CURRENT(PHASE_4_CURRENT_ADC_CHANNEL);
    }
    #endif
    #if NUM_PHASES >= 5
    if (working_data.phases[4].metrology.current[0].in_phase_correction.sd_preloaded_offset < PHASE_5_DELAY_SPLIT
        &&
        ADC_CURRENT_PENDING(PHASE_5_CURRENT_ADC_CHANNEL))
    {
        adc_i_buffer[4] = ADC_CURRENT(PHASE_5_CURRENT_ADC_CHANNEL);
    }
    #endif
    #if NUM_PHASES >= 6
    if (working_data.phases[5].metrology.current[0].in_phase_correction.sd_preloaded_offset < PHASE_6_DELAY_SPLIT
        &&
        ADC_CURRENT_PENDING(PHASE_6_CURRENT_ADC_CHANNEL))
    {
        adc_i_buffer[5] = ADC_CURRENT(PHASE_6_CURRENT_ADC_CHANNEL);
    }
    #endif
    #if defined(NEUTRAL_MONITOR_SUPPORT)
        #if NUM_PHASES == 1
    if (working_data.phases[0].metrology.current[1].in_phase_correction.sd_preloaded_offset < NEUTRAL_DELAY_SPLIT
        #else
    if (working_data.neutral.metrology.in_phase_correction.sd_preloaded_offset > NEUTRAL_DELAY_SPLIT
        #endif
        &&
        ADC_CURRENT_PENDING(NEUTRAL_CURRENT_ADC_CHANNEL))
    {
        adc_i_buffer[NUM_PHASES] = ADC_CURRENT(NEUTRAL_CURRENT_ADC_CHANNEL);
    }
    #endif
#endif
    
    /* We have a complete set of samples. Process them. */
    if (per_sample_dsp())
    {
#if defined(__MSP430__)
        /* The foreground may be conserving power (e.g. in limp mode), so we need to kick it. */
        _BIC_SR_IRQ(LPM0_bits);
#endif
    }
    

#if defined(ENERGY_PULSE_SUPPORT)  ||  defined(TOTAL_ENERGY_PULSE_SUPPORT)
    per_sample_energy_pulse_processing();
#endif

#if defined(__HAS_SD_ADC__)
    /* At this point we need to grab any current samples which are available. This will ensure
       the sweep up of current samples as the voltage interrupt occurs can selectively pick up
       only samples occuring between now and when the voltage samples are available. */
    if (ADC_CURRENT_PENDING(PHASE_1_CURRENT_ADC_CHANNEL))         
          adc_i_buffer[0] = ADC_CURRENT(PHASE_1_CURRENT_ADC_CHANNEL);
    #if NUM_PHASES >= 2
    if (ADC_CURRENT_PENDING(PHASE_2_CURRENT_ADC_CHANNEL))
        adc_i_buffer[1] = ADC_CURRENT(PHASE_2_CURRENT_ADC_CHANNEL);
    #endif
    #if NUM_PHASES >= 3
    if (ADC_CURRENT_PENDING(PHASE_3_CURRENT_ADC_CHANNEL))
        adc_i_buffer[2] = ADC_CURRENT(PHASE_3_CURRENT_ADC_CHANNEL);
    #endif
    #if NUM_PHASES >= 4
    if (ADC_CURRENT_PENDING(PHASE_4_CURRENT_ADC_CHANNEL))
        adc_i_buffer[3] = ADC_CURRENT(PHASE_4_CURRENT_ADC_CHANNEL);
    #endif
    #if NUM_PHASES >= 5
    if (ADC_CURRENT_PENDING(PHASE_5_CURRENT_ADC_CHANNEL))
        adc_i_buffer[4] = ADC_CURRENT(PHASE_5_CURRENT_ADC_CHANNEL);
    #endif
    #if NUM_PHASES >= 6
    if (ADC_CURRENT_PENDING(PHASE_6_CURRENT_ADC_CHANNEL))
        adc_i_buffer[5] = ADC_CURRENT(PHASE_6_CURRENT_ADC_CHANNEL);
    #endif
    #if defined(NEUTRAL_MONITOR_SUPPORT)
    if (ADC_CURRENT_PENDING(NEUTRAL_CURRENT_ADC_CHANNEL))
        adc_i_buffer[NUM_PHASES] = ADC_CURRENT(NEUTRAL_CURRENT_ADC_CHANNEL);
    #endif
#endif
  
    
//TDTD   
 //    P8OUT &= ~BIT4;
//int CContador4096=0;

    Contador4096++;

    //if (CContador4096&0x100000) {

//        P8OUT |= BIT4;
    //}
    //else {
    //    P8OUT &= ~BIT4;
    //}

    
    custom_adc_interrupt();

}

#if defined(__MSP430__)  &&  defined(__HAS_SD_ADC__)  &&  NUM_PHASES == 1  &&  defined(LIMP_MODE_SUPPORT)
/* Interrupt to trigger the SD16 ADC in limp mode */
#if defined(TIMER0_A0_VECTOR)
ISR(TIMER0_A0, limp_trigger_interrupt)
#else
ISR(TIMERA0, limp_trigger_interrupt)
#endif
{
    /* Trigger the ADC to perform a single conversion from all inputs. */
    sd_xxxx_reg(SD_PRE_, PHASE_1_VOLTAGE_ADC_CHANNEL) = 0;
    sd_xxxx_reg(SD_PRE_, PHASE_1_CURRENT_ADC_CHANNEL) = 0;
    #if defined(NEUTRAL_MONITOR_SUPPORT)
    sd_xxxx_reg(SD_PRE_, NEUTRAL_CURRENT_ADC_CHANNEL) = 0;
    #endif
    SD_CCTL_TRIGGER |= SD_SC;
}
#endif

int16_t volatile corrected;

#if defined(__MSP430__)  &&  defined(__MSP430_HAS_ADC10_A__)
/* Interrupt to handle the ADC10A in the 6xx family devices. */
ISR(ADC10, adc10_interrupt)
{
    //int16_t corrected;
  #if NUM_PHASES > 1  &&  defined(__MSP430_HAS_SD24_B3__)  &&  defined(__MSP430_HAS_ADC10_A__) && defined(TEMPERATURE_SUPPORT)
      dma_adc_buffer[15 - 10]=ADC10MEM0;
      ADC10IE=0;
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
       ADC10CTL2 =  ADC10RES | ADC10DF;
      ADC10MCTL0 = ADC10SREF_1 | ADC10INCH_15;
      
    /* Start ADC and wait for a trigger from the SD24 */
      ADC10CTL0 |= ADC10ENC;     
  #else
      
    switch (__even_in_range(ADC10IV, ADC10IV_ADC10IFG))
    {
    case ADC10IV_NONE:
        break;
    case ADC10IV_ADC10OVIFG:
        break;
    case ADC10IV_ADC10TOVIFG:
        break;
    case ADC10IV_ADC10HIIFG:
        break;
    case ADC10IV_ADC10LOIFG:
        break;
    case ADC10IV_ADC10INIFG:
        break;
    case ADC10IV_ADC10IFG:
        ADC10IFG &= ~ADC10IFG0;
        if (operating_mode == OPERATING_MODE_NORMAL)
        {
            #if defined(TEMPERATURE_SUPPORT)
            if ((ADC10MCTL0 & ADC10INCH_15) == ADC10INCH_10)
            {
                corrected = ADC10MEM0;
                raw_temperature_from_adc += corrected - (raw_temperature_from_adc >> 3);
            }
            #endif
            #if defined(VCC_MEASURE_SUPPORT)
            if ((ADC10MCTL0 & ADC10INCH_15) == ADC10INCH_11)
                vcc = ADC10MEM0;
            #endif
            #if defined(__MSP430_HAS_AUX_SUPPLY__)
            if ((ADC10MCTL0 & ADC10INCH_15) == ADC10INCH_12)
                auxadc = ADC10MEM0;
            #endif
            if ((ADC10MCTL0 & ADC10INCH_15) == ADC10INCH_15)
            {
                /* End of sequence. Step back to the first channel, and wait. */
                ADC10CTL0 &= ~ADC10ENC;
                ADC10MCTL0 &= ~ADC10INCH_15;
                ADC10CTL0 |= ADC10ENC;
            }
            else
            {
                /* Step to the next channel */
                ADC10CTL0 &= ~ADC10ENC;
                ADC10MCTL0++;
                ADC10CTL0 |= ADC10ENC;
                ADC10CTL0 |= ADC10SC;
            }
        }
        break;
    }
  #endif
}
#endif

ISR(DMA, dma_interrupt)
{
    switch (__even_in_range(DMAIV, DMAIV_DMA2IFG))
    {
    case DMAIV_DMA0IFG:
        DMA0CTL &= ~ DMAIFG;
        break;
    case DMAIV_DMA1IFG: 
       ADC10CTL0 &= ~(ADC10ENC);
       
       #if NUM_PHASES > 1  &&  defined(__MSP430_HAS_SD24_B3__)  &&  defined(__MSP430_HAS_ADC10_A__) && defined(TEMPERATURE_SUPPORT)
       ADC10CTL0 =   ADC10ON | ADC10SHT_6;
       ADC10CTL1 = ADC10SHP | ADC10SSEL_3 | ADC10DIV_5 ;                     // Enable sample timer
       ADC10CTL2 =  ADC10RES;
       ADC10MCTL0 = ADC10SREF_1 + ADC10INCH_10;  // ADC input ch A10 => temp sense 
       ADC10IE = 0x001;                          // ADC_IFG upon conv result-ADCMEMO
       ADC10CTL0 |= ADC10SC;
       DMA1CTL &= ~DMAIFG; 
       #endif
       
       ADC10CTL0 |= ADC10ENC;
      
        break;
    case DMAIV_DMA2IFG:
        DMA2CTL &= ~ DMAIFG;
    //#if defined(UART_0_DMA_SUPPORT)
    //    uart_tx_dma_core(0);
    //#endif
    //#if defined(UART_1_DMA_SUPPORT)
    //    uart_tx_dma_core(1);
    //#endif
        break;
    }
}
