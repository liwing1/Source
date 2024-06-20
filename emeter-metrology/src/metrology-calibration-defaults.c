/*******************************************************************************
 *  metrology-calibration-defaults.c -
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
#include <string.h>
#if !defined(__MSP430__)
#include <stdio.h>
#include <stdlib.h>
#endif
#if defined(__MSP430__)
#include <msp430.h>
#include <msp430_info_mem.h>
#endif

#include <emeter-toolkit.h>

#include "emeter-metrology.h"

#include "emeter-metrology-internal.h"

/* The main per-phase non-volatile parameter structure. Put this in the first available
   chunk of information memory. */
#if defined(__infomem__)
__infomem_uninitialized__ const struct info_mem_s nv_parms;
#elif defined(__MSP430_INFOD_MEM_SIZE__)
__infod_mem_uninitialized__ const struct info_mem_s nv_parms;
#elif defined(__MSP430_INFOC_MEM_SIZE__)
__infoc_mem_uninitialized__ const struct info_mem_s nv_parms;
#elif defined(__MSP430_INFOB_MEM_SIZE__)
__infob_mem_uninitialized__ const struct info_mem_s nv_parms;
#elif defined(__MSP430_INFOA_MEM_SIZE__)
__infoa_mem_uninitialized__ const struct info_mem_s nv_parms;
#endif

#if !defined(DEFAULT_P_SCALE_FACTOR_A)  &&  defined(DEFAULT_V_RMS_SCALE_FACTOR_A)  &&  defined(DEFAULT_I_RMS_SCALE_FACTOR_A)
#define DEFAULT_P_SCALE_FACTOR_A ((calibration_scaling_factor_t) ((uint64_t) DEFAULT_V_RMS_SCALE_FACTOR_A*DEFAULT_I_RMS_SCALE_FACTOR_A*32LLU/1000000LLU))
#endif

#if !defined(DEFAULT_P_SCALE_FACTOR_B)  &&  defined(DEFAULT_V_RMS_SCALE_FACTOR_B)  &&  defined(DEFAULT_I_RMS_SCALE_FACTOR_B)
#define DEFAULT_P_SCALE_FACTOR_B ((calibration_scaling_factor_t) ((uint64_t) DEFAULT_V_RMS_SCALE_FACTOR_B*DEFAULT_I_RMS_SCALE_FACTOR_B*32LLU/1000000LLU))
#endif

#if !defined(DEFAULT_P_SCALE_FACTOR_C)  &&  defined(DEFAULT_V_RMS_SCALE_FACTOR_C)  &&  defined(DEFAULT_I_RMS_SCALE_FACTOR_C)
#define DEFAULT_P_SCALE_FACTOR_C ((calibration_scaling_factor_t) ((uint64_t) DEFAULT_V_RMS_SCALE_FACTOR_C*DEFAULT_I_RMS_SCALE_FACTOR_C*32LLU/1000000LLU))
#endif

#if !defined(DEFAULT_P_SCALE_FACTOR_D)  &&  defined(DEFAULT_V_RMS_SCALE_FACTOR_D)  &&  defined(DEFAULT_I_RMS_SCALE_FACTOR_D)
#define DEFAULT_P_SCALE_FACTOR_D ((calibration_scaling_factor_t) ((uint64_t) DEFAULT_V_RMS_SCALE_FACTOR_D*DEFAULT_I_RMS_SCALE_FACTOR_D*32LLU/1000000LLU))
#endif

#if !defined(DEFAULT_P_SCALE_FACTOR_E)  &&  defined(DEFAULT_V_RMS_SCALE_FACTOR_E)  &&  defined(DEFAULT_I_RMS_SCALE_FACTOR_E)
#define DEFAULT_P_SCALE_FACTOR_E ((calibration_scaling_factor_t) ((uint64_t) DEFAULT_V_RMS_SCALE_FACTOR_E*DEFAULT_I_RMS_SCALE_FACTOR_E*32LLU/1000000LLU))
#endif

#if !defined(DEFAULT_P_SCALE_FACTOR_F)  &&  defined(DEFAULT_V_RMS_SCALE_FACTOR_F)  &&  defined(DEFAULT_I_RMS_SCALE_FACTOR_F)
#define DEFAULT_P_SCALE_FACTOR_F ((calibration_scaling_factor_t) ((uint64_t) DEFAULT_V_RMS_SCALE_FACTOR_F*DEFAULT_I_RMS_SCALE_FACTOR_F*32LLU/1000000LLU))
#endif

#if !defined(DEFAULT_P_SCALE_FACTOR_NEUTRAL)  &&  defined(DEFAULT_V_RMS_SCALE_FACTOR_A)  &&  defined(DEFAULT_I_RMS_SCALE_FACTOR_NEUTRAL)
#define DEFAULT_P_SCALE_FACTOR_NEUTRAL ((calibration_scaling_factor_t) ((uint64_t) DEFAULT_V_RMS_SCALE_FACTOR_A*DEFAULT_I_RMS_SCALE_FACTOR_NEUTRAL*32LLU/1000000LLU))
#endif

#if defined(RESIDUAL_IRMS_SUPPORT)  &&  !defined(DEFAULT_I_RELATIVE_SCALE_FACTOR_NEUTRAL)  &&  defined(DEFAULT_I_RMS_SCALE_FACTOR_A)  &&  defined(DEFAULT_I_RMS_SCALE_FACTOR_B)  &&  defined(DEFAULT_I_RMS_SCALE_FACTOR_C)  &&  defined(DEFAULT_I_RMS_SCALE_FACTOR_NEUTRAL)
#define DEFAULT_I_RELATIVE_SCALE_FACTOR_NEUTRAL ((calibration_scaling_factor_t) ((uint64_t) ((uint32_t) DEFAULT_I_RMS_SCALE_FACTOR_A + DEFAULT_I_RMS_SCALE_FACTOR_B + DEFAULT_I_RMS_SCALE_FACTOR_C)*32768LLU/(3LLU*DEFAULT_I_RMS_SCALE_FACTOR_NEUTRAL)))
#endif

const struct calibration_data_s calibration_defaults =
{
    {
        {
            {
                {
#if defined(LIMP_MODE_SUPPORT)
                    .initial_dc_estimate = {DEFAULT_I_DC_ESTIMATE_A, DEFAULT_I_DC_LIMP_ESTIMATE_A},
#else
                    .initial_dc_estimate = {DEFAULT_I_DC_ESTIMATE_A},
#endif
                    .ac_offset = DEFAULT_I_AC_OFFSET_A,
                    .phase_correction = DEFAULT_BASE_PHASE_A_CORRECTION,
#if defined(DYNAMIC_FREQUENCY_RELATED_CORRECTION_SUPPORT)
                    .frequency_phase_correction = DEFAULT_FREQUENCY_PHASE_FACTOR_A,
                    .frequency_gain_correction = DEFAULT_FREQUENCY_GAIN_FACTOR_A,
#endif
#if defined(IRMS_SUPPORT)
    #if defined(LIMP_MODE_SUPPORT)
                    .I_rms_scale_factor = {DEFAULT_I_RMS_SCALE_FACTOR_A, DEFAULT_I_RMS_LIMP_SCALE_FACTOR_A},
    #else
                    .I_rms_scale_factor = {DEFAULT_I_RMS_SCALE_FACTOR_A},
    #endif
#endif
                    .P_scale_factor = DEFAULT_P_SCALE_FACTOR_A,
                },
#if NUM_PHASES == 1  &&  defined(NEUTRAL_MONITOR_SUPPORT)
                {
                    /* This is for monitoring of the neutral lead in single phase anti-tamper meters */
    #if defined(LIMP_MODE_SUPPORT)
                    .initial_dc_estimate = {DEFAULT_I_DC_ESTIMATE_NEUTRAL, DEFAULT_I_DC_LIMP_ESTIMATE_NEUTRAL},
    #else
                    .initial_dc_estimate = {DEFAULT_I_DC_ESTIMATE_NEUTRAL},
    #endif
                    .ac_offset = DEFAULT_I_AC_OFFSET_NEUTRAL,
                    .phase_correction = DEFAULT_BASE_NEUTRAL_PHASE_CORRECTION,
    #if defined(DYNAMIC_FREQUENCY_RELATED_CORRECTION_SUPPORT)
                    .frequency_phase_correction = DEFAULT_FREQUENCY_PHASE_FACTOR_NEUTRAL,
                    .frequency_gain_correction = DEFAULT_FREQUENCY_GAIN_FACTOR_NEUTRAL,
    #endif
    #if defined(IRMS_SUPPORT)
        #if defined(LIMP_MODE_SUPPORT)
                    .I_rms_scale_factor = {DEFAULT_I_RMS_SCALE_FACTOR_NEUTRAL, DEFAULT_I_RMS_LIMP_SCALE_FACTOR_NEUTRAL},
        #else
                    .I_rms_scale_factor = {DEFAULT_I_RMS_SCALE_FACTOR_NEUTRAL},
        #endif
    #endif
                    .P_scale_factor = DEFAULT_P_SCALE_FACTOR_NEUTRAL,
                }
#endif
            },
#if defined(LIMP_MODE_SUPPORT)
            .initial_v_dc_estimate = {DEFAULT_V_DC_ESTIMATE_A, DEFAULT_V_LIMP_DC_ESTIMATE_A},
            .lower_v_dc_estimate = {DEFAULT_V_DC_ESTIMATE_A - 100, DEFAULT_V_LIMP_DC_ESTIMATE_A - 100},
            .upper_v_dc_estimate = {DEFAULT_V_DC_ESTIMATE_A + 100, DEFAULT_V_LIMP_DC_ESTIMATE_A + 100},
#else
            .initial_v_dc_estimate = {DEFAULT_V_DC_ESTIMATE_A},
#endif
            .v_ac_offset = DEFAULT_V_AC_OFFSET_A,
#if defined(VRMS_SUPPORT)
    #if defined(LIMP_MODE_SUPPORT)
            .V_rms_scale_factor = {DEFAULT_V_RMS_SCALE_FACTOR_A, DEFAULT_V_RMS_LIMP_SCALE_FACTOR_A},
    #else
            .V_rms_scale_factor = {DEFAULT_V_RMS_SCALE_FACTOR_A},
    #endif
#endif
        },
#if NUM_PHASES >= 2
        {
            {
                {
                    .initial_dc_estimate = {DEFAULT_I_DC_ESTIMATE_B},
                    .ac_offset = DEFAULT_I_AC_OFFSET_B,
                    .phase_correction = DEFAULT_BASE_PHASE_B_CORRECTION,
    #if defined(DYNAMIC_FREQUENCY_RELATED_CORRECTION_SUPPORT)
                    .frequency_phase_correction = DEFAULT_FREQUENCY_PHASE_FACTOR_B,
                    .frequency_gain_correction = DEFAULT_FREQUENCY_GAIN_FACTOR_B,
    #endif
    #if defined(IRMS_SUPPORT)
                    .I_rms_scale_factor = {DEFAULT_I_RMS_SCALE_FACTOR_B},
    #endif
                    .P_scale_factor = DEFAULT_P_SCALE_FACTOR_B,
                }
            },
            .initial_v_dc_estimate = {DEFAULT_V_DC_ESTIMATE_B},
            .v_ac_offset = DEFAULT_V_AC_OFFSET_B,
    #if defined(VRMS_SUPPORT)
            .V_rms_scale_factor = DEFAULT_V_RMS_SCALE_FACTOR_B,
    #endif
        },
#endif
#if NUM_PHASES >= 3
        {
            {
                {
                    .initial_dc_estimate = {DEFAULT_I_DC_ESTIMATE_C},
                    .ac_offset = DEFAULT_I_AC_OFFSET_C,
                    .phase_correction = DEFAULT_BASE_PHASE_C_CORRECTION,
    #if defined(DYNAMIC_FREQUENCY_RELATED_CORRECTION_SUPPORT)
                    .frequency_phase_correction = DEFAULT_FREQUENCY_PHASE_FACTOR_C,
                    .frequency_gain_correction = DEFAULT_FREQUENCY_GAIN_FACTOR_C,
    #endif
    #if defined(IRMS_SUPPORT)
                    .I_rms_scale_factor = {DEFAULT_I_RMS_SCALE_FACTOR_C},
    #endif
                    .P_scale_factor = DEFAULT_P_SCALE_FACTOR_C,
                }
            },
            .initial_v_dc_estimate = {DEFAULT_V_DC_ESTIMATE_C},
            .v_ac_offset = DEFAULT_V_AC_OFFSET_C,
    #if defined(VRMS_SUPPORT)
            .V_rms_scale_factor = DEFAULT_V_RMS_SCALE_FACTOR_C,
    #endif
        },
#endif
#if NUM_PHASES >= 4
        {
            {
                {
                    .initial_dc_estimate = {DEFAULT_I_DC_ESTIMATE_D},
                    .ac_offset = DEFAULT_I_AC_OFFSET_D,
                    .phase_correction = DEFAULT_BASE_PHASE_D_CORRECTION,
    #if defined(DYNAMIC_FREQUENCY_RELATED_CORRECTION_SUPPORT)
                    .frequency_phase_correction = DEFAULT_FREQUENCY_PHASE_FACTOR_D,
                    .frequency_gain_correction = DEFAULT_FREQUENCY_GAIN_FACTOR_D,
    #endif
    #if defined(IRMS_SUPPORT)
                    .I_rms_scale_factor = {DEFAULT_I_RMS_SCALE_FACTOR_D},
    #endif
                    .P_scale_factor = DEFAULT_P_SCALE_FACTOR_D,
                }
            },
            .initial_v_dc_estimate = {DEFAULT_V_DC_ESTIMATE_D},
            .v_ac_offset = DEFAULT_V_AC_OFFSET_D,
    #if defined(VRMS_SUPPORT)
            .V_rms_scale_factor = DEFAULT_V_RMS_SCALE_FACTOR_D,
    #endif
        },
#endif
#if NUM_PHASES >= 5
        {
            {
                {
                    .initial_dc_estimate = {DEFAULT_I_DC_ESTIMATE_E},
                    .ac_offset = DEFAULT_I_AC_OFFSET_E,
                    .phase_correction = DEFAULT_BASE_PHASE_E_CORRECTION,
    #if defined(DYNAMIC_FREQUENCY_RELATED_CORRECTION_SUPPORT)
                    .frequency_phase_correction = DEFAULT_FREQUENCY_PHASE_FACTOR_E,
                    .frequency_gain_correction = DEFAULT_FREQUENCY_GAIN_FACTOR_E,
    #endif
    #if defined(IRMS_SUPPORT)
                    .I_rms_scale_factor = {DEFAULT_I_RMS_SCALE_FACTOR_E},
    #endif
                    .P_scale_factor = DEFAULT_P_SCALE_FACTOR_E,
                }
            },
            .initial_v_dc_estimate = {DEFAULT_V_DC_ESTIMATE_E},
            .v_ac_offset = DEFAULT_V_AC_OFFSET_E,
    #if defined(VRMS_SUPPORT)
            .V_rms_scale_factor = DEFAULT_V_RMS_SCALE_FACTOR_E,
    #endif
        },
#endif
#if NUM_PHASES >= 6
        {
            {
                {
                    .initial_dc_estimate = {DEFAULT_I_DC_ESTIMATE_F},
                    .ac_offset = DEFAULT_I_AC_OFFSET_F,
                    .phase_correction = DEFAULT_BASE_PHASE_F_CORRECTION,
    #if defined(DYNAMIC_FREQUENCY_RELATED_CORRECTION_SUPPORT)
                    .frequency_phase_correction = DEFAULT_FREQUENCY_PHASE_FACTOR_F,
                    .frequency_gain_correction = DEFAULT_FREQUENCY_GAIN_FACTOR_F,
    #endif
    #if defined(IRMS_SUPPORT)
                    .I_rms_scale_factor = {DEFAULT_I_RMS_SCALE_FACTOR_F},
    #endif
                    .P_scale_factor = DEFAULT_P_SCALE_FACTOR_F,
                }
            },
            .initial_v_dc_estimate = {DEFAULT_V_DC_ESTIMATE_F},
            .v_ac_offset = DEFAULT_V_AC_OFFSET_F,
    #if defined(VRMS_SUPPORT)
            .V_rms_scale_factor = DEFAULT_V_RMS_SCALE_FACTOR_F,
    #endif
        },
#endif
    },
#if NUM_PHASES > 1  &&  defined(NEUTRAL_MONITOR_SUPPORT)
    {
        .initial_dc_estimate = {DEFAULT_I_DC_ESTIMATE_NEUTRAL},
        .ac_offset = DEFAULT_I_AC_OFFSET_NEUTRAL,
        .phase_correction = DEFAULT_BASE_NEUTRAL_PHASE_CORRECTION,
    #if defined(DYNAMIC_FREQUENCY_RELATED_CORRECTION_SUPPORT)
        .frequency_phase_correction = DEFAULT_FREQUENCY_PHASE_FACTOR_NEUTRAL,
        .frequency_gain_correction = DEFAULT_FREQUENCY_GAIN_FACTOR_NEUTRAL,
    #endif
    #if defined(IRMS_SUPPORT)
        .I_rms_scale_factor = {DEFAULT_I_RMS_SCALE_FACTOR_NEUTRAL},
    #endif
    #if defined(RESIDUAL_IRMS_SUPPORT)
        /* Reuse the P_scale_factor entry to hold the relative scaling factor for residual current measurement */
        .P_scale_factor = DEFAULT_I_RELATIVE_SCALE_FACTOR_NEUTRAL
    #else
        .P_scale_factor = 0
    #endif
    },
#endif
#if defined(TEMPERATURE_SUPPORT)
    .temperature_at_calibration = DEFAULT_ROOM_TEMPERATURE,
    .temperature_sensor_intercept = DEFAULT_TEMPERATURE_INTERCEPT,
    .temperature_sensor_slope = 10L*65536L/DEFAULT_TEMPERATURE_SLOPE
#endif
};

const struct configuration_data_s configuration_defaults = 
{
    .baud_rate = 0,
    .mb_address = 0x68
};

void set_cfg_baud_rate(uint16_t baud_rate)
{
    // clear_calibration_data();
    // flash_memcpy((void *) cal_info, (const void *) &calibration_defaults, sizeof(calibration_defaults));
    // flash_secure();

    // flash_memcpy((void*) &cfg_info->baud_rate, (const void *)&baud_rate, sizeof(uint16_t));
    // //flash_memcpy((void *) cfg_info, (const void *) &baud_rate, sizeof(configuration_defaults));
    // flash_secure();
    
    flash_write_int16((int16_t *) &cfg_info->baud_rate, baud_rate);
    flash_secure();
}

uint16_t get_cfg_baud_rate(void)
{
    return cfg_info->baud_rate;
}

uint16_t get_cfg_mb_address(void)
{
    return cfg_info->mb_address;
}

int16_t get_v_dc_estimate(int phx, int which)
{
    return cal_info->phases[phx].initial_v_dc_estimate[which];
}

void set_v_dc_estimate(int phx, int which, int16_t value)
{
    flash_write_int16((int16_t *) &cal_info->phases[phx].initial_v_dc_estimate[which], value);
#if defined(LIMP_MODE_SUPPORT)
    flash_write_int16((int16_t *) &cal_info->phases[phx].lower_v_dc_estimate[which], value - 100);
    flash_write_int16((int16_t *) &cal_info->phases[phx].upper_v_dc_estimate[which], value + 100);
#endif
    flash_secure();
}

int16_t get_i_dc_estimate(int phx, int which)
{
#if defined(NEUTRAL_MONITOR_SUPPORT)
    if (phx == FAKE_PHASE_NEUTRAL)
    {
    #if NUM_PHASES > 1
        return cal_info->neutral.initial_dc_estimate[which];
    #else
        return cal_info->phases[0].current[1].initial_dc_estimate[which];
    #endif
    }
#endif
    return cal_info->phases[phx].current[0].initial_dc_estimate[which];
}

void set_i_dc_estimate(int phx, int which, int16_t value)
{
#if defined(NEUTRAL_MONITOR_SUPPORT)
    if (phx == FAKE_PHASE_NEUTRAL)
    {
    #if NUM_PHASES > 1
        flash_write_int16((int16_t *) &cal_info->neutral.initial_dc_estimate[which], value);
    #else
        flash_write_int16((int16_t *) &cal_info->phases[0].current[1].initial_dc_estimate[which], value);
    #endif
    }
    else
#endif
    {
        flash_write_int16((int16_t *) &cal_info->phases[phx].current[0].initial_dc_estimate[which], value);
    }
    flash_secure();
}

int32_t get_v_ac_offset(int phx)
{
    return cal_info->phases[phx].v_ac_offset;
}

void set_v_ac_offset(int phx, int32_t value)
{
    flash_write_int32((int32_t *) &cal_info->phases[phx].v_ac_offset, value);
    flash_secure();
}

int32_t get_i_ac_offset(int phx)
{
#if defined(NEUTRAL_MONITOR_SUPPORT)
    if (phx == FAKE_PHASE_NEUTRAL)
    {
    #if NUM_PHASES > 1
        return cal_info->neutral.ac_offset;
    #else
        return cal_info->phases[0].current[1].ac_offset;
    #endif
    }
#endif
    return cal_info->phases[phx].current[0].ac_offset;
}

void set_i_ac_offset(int phx, int32_t value)
{
#if defined(NEUTRAL_MONITOR_SUPPORT)
    if (phx == FAKE_PHASE_NEUTRAL)
    {
    #if NUM_PHASES > 1
        flash_write_int32((int32_t *) &cal_info->neutral.ac_offset, value);
    #else
        flash_write_int32((int32_t *) &cal_info->phases[0].current[1].ac_offset, value);
    #endif
    }
    else
#endif
    {
        flash_write_int32((int32_t *) &cal_info->phases[phx].current[0].ac_offset, value);
    }
    flash_secure();
}

int16_t get_phase_corr(int phx)
{
#if defined(NEUTRAL_MONITOR_SUPPORT)
    if (phx == FAKE_PHASE_NEUTRAL)
    {
    #if NUM_PHASES > 1
        return cal_info->neutral.phase_correction << 3;
    #else
        return cal_info->phases[0].current[1].phase_correction << 3;
    #endif
    }
#endif
    return cal_info->phases[phx].current[0].phase_correction << 3;
}

void set_phase_corr(int phx, int16_t value)
{
    value >>= 3;

#if defined(NEUTRAL_MONITOR_SUPPORT)
    if (phx == FAKE_PHASE_NEUTRAL)
    {
    #if NUM_PHASES > 1
        flash_write_int16((int16_t *) &cal_info->neutral.phase_correction, value);
    #else
        flash_write_int16((int16_t *) &cal_info->phases[0].current[1].phase_correction, value);
    #endif
    }
    else
#endif
    {
        flash_write_int16((int16_t *) &cal_info->phases[phx].current[0].phase_correction, value);
    }
    flash_secure();
}

#if defined(DYNAMIC_CURRENT_RELATED_CORRECTION_SUPPORT)  ||  defined(DYNAMIC_FREQUENCY_RELATED_CORRECTION_SUPPORT)
int16_t get_frequency_related_phase_corr(int phx)
{
#if defined(NEUTRAL_MONITOR_SUPPORT)
    if (phx == FAKE_PHASE_NEUTRAL)
    {
    #if NUM_PHASES > 1
        return cal_info->neutral.frequency_phase_correction;
    #else
        return cal_info->phases[0].current[1].frequency_phase_correction;
    #endif
    }
#endif
    return cal_info->phases[phx].current[0].frequency_phase_correction;
}

void set_frequency_related_phase_corr(int phx, int16_t value)
{
    value >>= 3;

#if defined(NEUTRAL_MONITOR_SUPPORT)
    if (phx == FAKE_PHASE_NEUTRAL)
    {
    #if NUM_PHASES > 1
        flash_write_int16((int16_t *) &cal_info->neutral.frequency_phase_correction, value);
    #else
        flash_write_int16((int16_t *) &cal_info->phases[0].current[1].frequency_phase_correction, value);
    #endif
    }
    else
#endif
    {
        flash_write_int16((int16_t *) &cal_info->phases[phx].current[0].frequency_phase_correction, value);
    }
    flash_secure();
}

int16_t get_frequency_related_gain_corr(int phx)
{
#if defined(NEUTRAL_MONITOR_SUPPORT)
    if (phx == FAKE_PHASE_NEUTRAL)
    {
    #if NUM_PHASES > 1
        return cal_info->neutral.frequency_gain_correction << 3;
    #else
        return cal_info->phases[0].current[1].frequency_gain_correction << 3;
    #endif
    }
#endif
    return cal_info->phases[phx].current[0].frequency_gain_correction << 3;
}

void set_frequency_related_gain_corr(int phx, int16_t value)
{
    value >>= 3;

#if defined(NEUTRAL_MONITOR_SUPPORT)
    if (phx == FAKE_PHASE_NEUTRAL)
    {
    #if NUM_PHASES > 1
        flash_write_int16((int16_t *) &cal_info->neutral.frequency_gain_correction, value);
    #else
        flash_write_int16((int16_t *) &cal_info->phases[0].current[1].frequency_gain_correction, value);
    #endif
    }
    else
#endif
    {
        flash_write_int16((int16_t *) &cal_info->phases[phx].current[0].frequency_gain_correction, value);
    }
    flash_secure();
}
#endif

calibration_scaling_factor_t get_V_rms_scaling(int phx, int which)
{
    return cal_info->phases[phx].V_rms_scale_factor[which];
}

void set_V_rms_scaling(int phx, int which, calibration_scaling_factor_t value)
{
    flash_write_int16((int16_t *) &cal_info->phases[phx].V_rms_scale_factor[which], value);
    flash_secure();
}

calibration_scaling_factor_t get_I_rms_scaling(int phx, int which)
{
#if defined(NEUTRAL_MONITOR_SUPPORT)
    if (phx == FAKE_PHASE_NEUTRAL)
    {
    #if NUM_PHASES > 1
        return cal_info->neutral.I_rms_scale_factor[which];
    #else
        return cal_info->phases[0].current[1].I_rms_scale_factor[which];
    #endif
    }
#endif
    return cal_info->phases[phx].current[0].I_rms_scale_factor[which];
}

void set_I_rms_scaling(int phx, int which, calibration_scaling_factor_t value)
{
#if defined(NEUTRAL_MONITOR_SUPPORT)
    if (phx == FAKE_PHASE_NEUTRAL)
    {
    #if NUM_PHASES > 1
        flash_write_int16((int16_t *) &cal_info->neutral.I_rms_scale_factor[which], value);
    #else
        flash_write_int16((int16_t *) &cal_info->phases[0].current[1].I_rms_scale_factor[which], value);
    #endif
    }
    else
#endif
    {
        flash_write_int16((int16_t *) &cal_info->phases[phx].current[0].I_rms_scale_factor[which], value);
    }
    flash_secure();
}

calibration_scaling_factor_t get_P_scaling(int phx)
{
    int which;

#if NUM_PHASES == 1  &&  defined(NEUTRAL_MONITOR_SUPPORT)
    if (phx == FAKE_PHASE_NEUTRAL)
    {
        phx = 0;
        which = 1;
    }
    else
    {
        which = 0;
    }
#else
    which = 0;
#endif
    return cal_info->phases[phx].current[which].P_scale_factor;
}

void set_P_scaling(int phx, calibration_scaling_factor_t value)
{
    int which;

#if NUM_PHASES == 1  &&  defined(NEUTRAL_MONITOR_SUPPORT)
    if (phx == FAKE_PHASE_NEUTRAL)
    {
        phx = 0;
        which = 1;
    }
    else
    {
        which = 0;
    }
#else
    which = 0;
#endif
    if (value == 0)
    {
        value = (calibration_scaling_factor_t) (((uint64_t) cal_info->phases[phx].V_rms_scale_factor[0]*cal_info->phases[phx].current[which].I_rms_scale_factor[which]*32LLU)/1000000LLU);
    }
    flash_write_int16((int16_t *) &cal_info->phases[phx].current[which].P_scale_factor, value);
    flash_secure();
}

int get_calibration_status(void)
{
    return nv_parms.seg_a.s.header.calibration_status; 
}

void set_calibration_status(int value)
{
    flash_write_int16((int16_t *) &nv_parms.seg_a.s.header.calibration_status, value);
    flash_secure();
}

#if defined(TEMPERATURE_SUPPORT)
int16_t get_temperature_intercept(void)
{
    return cal_info->temperature_sensor_intercept;
}

int16_t get_temperature_slope(void)
{
    return cal_info->temperature_sensor_slope;
}

void set_temperature_parameters(int16_t temperature_at_calibration, int16_t temperature_sensor_intercept, int16_t temperature_sensor_slope)
{
    flash_write_int16((int16_t *) &cal_info->temperature_at_calibration, temperature_at_calibration);
    flash_write_int16((int16_t *) &cal_info->temperature_sensor_intercept, temperature_sensor_intercept);
    flash_write_int16((int16_t *) &cal_info->temperature_sensor_slope, temperature_sensor_slope);
    flash_secure();
}
#endif

int clear_calibration_data(void)
{
    flash_clr((int16_t *) &nv_parms);
    /* If the nv_parms structure is bigger than a single info memory page, we may need to erase an extra page */
#if defined(__MSP430_INFOD_MEM_SIZE__)
    if (sizeof(struct info_mem_s) > __MSP430_INFOD_MEM_SIZE__)
        flash_clr((int16_t *) (((int8_t *) &nv_parms) + __MSP430_INFOD_MEM_SIZE__));
#elif defined(__MSP430_INFOC_MEM_SIZE__)
    if (sizeof(struct info_mem_s) > __MSP430_INFOC_MEM_SIZE__)
        flash_clr((int16_t *) (((int8_t *) &nv_parms) + __MSP430_INFOC_MEM_SIZE__));
#elif defined(__MSP430_INFOB_MEM_SIZE__)
    if (sizeof(struct info_mem_s) > __MSP430_INFOB_MEM_SIZE__)
        flash_clr((int16_t *) (((int8_t *) &nv_parms) + __MSP430_INFOB_MEM_SIZE__));
#elif defined(__MSP430_INFOA_MEM_SIZE__)
    if (sizeof(struct info_mem_s) > __MSP430_INFOA_MEM_SIZE__)
        flash_clr((int16_t *) (((int8_t *) &nv_parms) + __MSP430_INFOA_MEM_SIZE__));
#endif
    return 0;
}

void write_calibration_data(const void* cal_data, const void* cfg_data)
{
  flash_memcpy((void*) cal_info, cal_data, sizeof(calibration_defaults));
  flash_memcpy((void*) cfg_info, cfg_data, sizeof(configuration_defaults));
  flash_secure();   
}

struct calibration_data_s const* get_cal_info(void)
{
    return cal_info;
}

struct configuration_data_s const* get_cfg_info(void)
{
    return cfg_info;
}