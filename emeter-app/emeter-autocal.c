/*******************************************************************************
 *  emeter-autocal.c
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
#if defined(__MSP430__)
#include <msp430.h>
#endif

#include "emeter-template.h"

#include <emeter-toolkit.h>
#include <emeter-metrology.h>

#include "emeter-main.h"
#include "emeter-app.h"
#include "emeter-autocal.h"

#if defined(SUPPORT_ONE_POINT_AUTOCALIBRATION)

static int8_t autocal_debounce = 0;
static int8_t autocal_progress = 0;

static rms_voltage_t voltage_sums[NUM_PHASES];
static rms_current_t current_sums[NUM_PHASES];
static power_t active_power_sums[NUM_PHASES];

static rms_voltage_t last_voltages[NUM_PHASES];
static rms_current_t last_currents[NUM_PHASES];
static power_t last_active_powers[NUM_PHASES];

static int stable_scans;

#if NUM_PHASES > 1  ||  !defined(NEUTRAL_MONITOR_SUPPORT)
    #define CALIBRATION_CHANNELS NUM_PHASES
#else
    #define CALIBRATION_CHANNELS 2
#endif

void autocalibrate(void)
{
    int phx;
    rms_voltage_t v;
    rms_voltage_t i;
    rms_voltage_t p;
    int stable;
    calibration_scaling_factor_t v_scaling[CALIBRATION_CHANNELS];
    calibration_scaling_factor_t i_scaling[CALIBRATION_CHANNELS];
    int16_t phase_corr[CALIBRATION_CHANNELS];
#if defined(DYNAMIC_CURRENT_RELATED_CORRECTION_SUPPORT)  ||  defined(DYNAMIC_FREQUENCY_RELATED_CORRECTION_SUPPORT)
    int16_t frequency_phase_corr[CALIBRATION_CHANNELS];
    int16_t frequency_gain_corr[CALIBRATION_CHANNELS];
#endif
    int64_t xxx;
    int16_t yyy;
    int16_t zzz;
    int32_t phase;

    switch (autocal_progress)
    {
    case 0:
        /* Waiting for auto-calibration to be initiated */
        if ((AUTOCAL_ENABLE_PORT & AUTOCAL_ENABLE_BIT) == AUTOCAL_ENABLE_BIT_ACTIVE_STATE)
        {
            if (++autocal_debounce >= 3)
            {
                autocal_debounce = 0;
#if NUM_PHASES > 1
                for (phx = 0;  phx < NUM_PHASES;  phx++)
#else
                phx = 0;
#endif
                {
                    voltage_sums[phx] = 0;
                    current_sums[phx] = 0;
                    active_power_sums[phx] = 0;
                    last_voltages[phx] = 0;
                    last_currents[phx] = 0;
                    last_active_powers[phx] = 0;
                }
                stable_scans = 0;
                custom_autocal_indicator(1);
                autocal_progress = 1;
            }
        }
        else
        {
            autocal_debounce = 0;
        }
        break;
    case 1:
        /* Auto-calibration in progress */
        stable = 0;
#if NUM_PHASES > 1
        for (phx = 0;  phx < NUM_PHASES;  phx++)
#else
        phx = 0;
#endif
        {
            v = rms_voltage(phx);
            i = rms_current(phx);
            p = active_power(phx);
            if (v > (rms_voltage_t) AUTOCAL_VOLTAGE*RMS_VOLTAGE_SCALING_FACTOR*(100 - AUTOCAL_MAX_UNCAL_ERROR)/100
                &&
                v < (rms_voltage_t) AUTOCAL_VOLTAGE*RMS_VOLTAGE_SCALING_FACTOR*(100 + AUTOCAL_MAX_UNCAL_ERROR)/100
                &&
                v > (last_voltages[phx] - (rms_voltage_t) RMS_VOLTAGE_SCALING_FACTOR*AUTOCAL_VOLTAGE/1000)
                &&
                v < (last_voltages[phx] + (rms_voltage_t) RMS_VOLTAGE_SCALING_FACTOR*AUTOCAL_VOLTAGE/1000))
            {
                /* The voltage looks stable */
                voltage_sums[phx] += v;
                stable++;
            }
            last_voltages[phx] = v;
            if (i > (rms_current_t) AUTOCAL_CURRENT*RMS_CURRENT_SCALING_FACTOR*(100 - AUTOCAL_MAX_UNCAL_ERROR)/100
                &&
                i < (rms_current_t) AUTOCAL_CURRENT*RMS_CURRENT_SCALING_FACTOR*(100 + AUTOCAL_MAX_UNCAL_ERROR)/100
                &&
                i > (last_currents[phx] - (rms_current_t) RMS_CURRENT_SCALING_FACTOR*AUTOCAL_CURRENT/1000)
                &&
                i < (last_currents[phx] + (rms_current_t) RMS_CURRENT_SCALING_FACTOR*AUTOCAL_CURRENT/1000))
            {
                /* The current looks stable */
                current_sums[phx] += i;
                stable++;
            }
            last_currents[phx] = i;
            if (p > (last_active_powers[phx] - (power_t) POWER_SCALING_FACTOR*AUTOCAL_VOLTAGE*AUTOCAL_CURRENT/1000)
                &&
                p < (last_active_powers[phx] + (power_t) POWER_SCALING_FACTOR*AUTOCAL_VOLTAGE*AUTOCAL_CURRENT/1000))
            {
                /* The active power looks stable */
                active_power_sums[phx] += p;
                stable++;
            }
            last_active_powers[phx] = p;
        }
        if (stable != 3*NUM_PHASES)
        {
            /* If we don't consider all the readings stable, we start again */
#if NUM_PHASES > 1
            for (phx = 0;  phx < NUM_PHASES;  phx++)
#else
            phx = 0;
#endif
            {
                voltage_sums[phx] = 0;
                current_sums[phx] = 0;
                active_power_sums[phx] = 0;
            }
            stable_scans = 0;
            custom_autocal_indicator(1);
        }
        else
        {
            custom_autocal_indicator(2);
            if (++stable_scans == 8)
            {
                /* Everything looks sufficiently stable to calculate a set of calibration constants */
#if NUM_PHASES > 1
                for (phx = 0;  phx < NUM_PHASES;  phx++)
#else
                phx = 0;
#endif
                {
                    /* The scaling factors are straightforward to work out. We can get the voltage and current
                       scalings from the measured values, and we can get the power scaling from the voltage and
                       current scalings. We CANNOT get the power scaling from the measured power, as that
                       reading is currenly distorted by not having the phase correction right. */
                    /* Scale the sums to allow for collecting the sum of 8 results */
                    voltage_sums[phx] >>= 3;
                    current_sums[phx] >>= 3;
                    active_power_sums[phx] >>= 3;

                    v_scaling[phx] = get_V_rms_scaling(phx, 0);
                    xxx = (int64_t) v_scaling[phx]*AUTOCAL_VOLTAGE*RMS_VOLTAGE_SCALING_FACTOR/voltage_sums[phx];
                    v_scaling[phx] = xxx;
    
                    i_scaling[phx] = get_I_rms_scaling(phx, 0);
                    xxx = (int64_t) i_scaling[phx]*AUTOCAL_CURRENT*RMS_CURRENT_SCALING_FACTOR/current_sums[phx];
                    i_scaling[phx] = xxx;
    
                    /* Now we need to work out the phase correction. Here we must assume the phase shift from the
                       test bench is a precise 60 degrees, which is not the case with most test benches, so this may not work
                       out very well. */
                    xxx = (uint64_t) voltage_sums[phx]*current_sums[phx];
                    xxx = active_power_sums[phx]*1000000LLU*32768LLU/xxx;
                    /* xxx should be 0.5*32768 if the phase shift is spot on. If it is more than this the phase shift is <60 degrees. If it is less than this the
                       phase shift is >60 degrees */
    
                    yyy = xxx;
                    /* Start with a phase of 60 degrees (actually 30 degrees, as we are working with a sine table and not as cos table), and search until we
                       find the real phase shift */
                    phase = 0x80000000LU/6LU;
                    if (yyy < 32768U/2U)
                    {
                        do
                        {
                            phase -= 0x80000000LU/36000LU;
                            zzz = dds_interpolated_lookup(phase);
                        }
                        while (yyy < zzz);
                    }
                    else
                    {
                        do
                        {
                            phase += 0x80000000LU/36000LU;
                        }
                        while (yyy > dds_interpolated_lookup(phase));
                    }
                    phase -= 0x80000000LU/6LU;
                    /* Phase should now be the actual phase error. We need to scale it to 1/1024 sample times. */
                    xxx = (int64_t) phase*SAMPLE_RATE*2048LL/(0x100000000LL*MAINS_NOMINAL_FREQUENCY);
                    phase_corr[phx] = get_phase_corr(phx) + xxx;
#if defined(DYNAMIC_CURRENT_RELATED_CORRECTION_SUPPORT)  ||  defined(DYNAMIC_FREQUENCY_RELATED_CORRECTION_SUPPORT)
                    /* Snapshot the frequency related corrections, so they can be restored after the calibration
                       data is cleared. */
                    frequency_phase_corr[phx] = get_frequency_related_phase_corr(phx);
                    frequency_gain_corr[phx] = get_frequency_related_gain_corr(phx);
#endif
                }
                clear_calibration_data();
#if NUM_PHASES > 1
                for (phx = 0;  phx < NUM_PHASES;  phx++)
#else
                phx = 0;
#endif
                {
                    set_V_rms_scaling(phx, 0, v_scaling[phx]);
                    set_I_rms_scaling(phx, 0, i_scaling[phx]);
                    /* Set the power scaling to zero, to force the metrology library to work out the scaling itself from the
                       voltage and current scaling factors */
                    set_P_scaling(phx, 0);
                    set_phase_corr(phx, phase_corr[phx]);
#if defined(DYNAMIC_CURRENT_RELATED_CORRECTION_SUPPORT)  ||  defined(DYNAMIC_FREQUENCY_RELATED_CORRECTION_SUPPORT)
                    set_frequency_related_phase_corr(phx, frequency_phase_corr[phx]);
                    set_frequency_related_gain_corr(phx, frequency_gain_corr[phx]);
#endif
                }
#if defined(NEUTRAL_MONITOR_SUPPORT)
                /* We should be here if we are building a meter which monitors the neutral, but we do not intend to perform a
                   separate calibration of the neutral's gain and phase error. For some people, who use exactly the same sensor
                   in the neutral channel as is used for the phase channels, this might give good enough results. */
                set_I_rms_scaling(FAKE_PHASE_NEUTRAL, 0, i_scaling[CALIBRATION_CHANNELS - 1]);
                /* Set the power scaling to zero, to force the metrology library to work out the scaling itself from the
                   voltage and current scaling factors */
                set_P_scaling(FAKE_PHASE_NEUTRAL, 0);
                set_phase_corr(FAKE_PHASE_NEUTRAL, phase_corr[CALIBRATION_CHANNELS - 1]);
    #if defined(DYNAMIC_CURRENT_RELATED_CORRECTION_SUPPORT)  ||  defined(DYNAMIC_FREQUENCY_RELATED_CORRECTION_SUPPORT)
                set_frequency_related_phase_corr(FAKE_PHASE_NEUTRAL, frequency_phase_corr[CALIBRATION_CHANNELS - 1]);
                set_frequency_related_gain_corr(FAKE_PHASE_NEUTRAL, frequency_gain_corr[CALIBRATION_CHANNELS - 1]);
    #endif
#endif
                align_metrology_with_calibration_data();
                autocal_progress = 2;
                custom_autocal_indicator(3);
            }
        }
        break;
    case 2:
        /* Waiting for the auto-calibration signal to go away */
        if ((AUTOCAL_ENABLE_PORT & AUTOCAL_ENABLE_BIT) == AUTOCAL_ENABLE_BIT_ACTIVE_STATE)
        {
            autocal_debounce = 0;
        }
        else
        {
            if (++autocal_debounce >= 3)
            {
                autocal_debounce = 0;
                custom_autocal_indicator(0);
                autocal_progress = 0;
            }
        }
        break;
    }
}

#endif
