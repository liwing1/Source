/*******************************************************************************
 *  metrology-readings.c -
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
#if !defined(__MSP430__)
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#endif
#if defined(__GNUC__)
#include <signal.h>
#endif

#include <msp430.h>

#include <emeter-toolkit.h>

#include "emeter-metrology.h"

#include "emeter-metrology-internal.h"

#if defined(PRECALCULATED_PARAMETER_SUPPORT)
power_t active_power(int phx)
{
    if (phx == FAKE_PHASE_TOTAL)
    #if defined(TOTAL_ACTIVE_POWER_SUPPORT)
        return working_data.totals.readings.active_power;
    #else
        return 0;
    #endif
    #if NUM_PHASES == 1  &&  defined(NEUTRAL_MONITOR_SUPPORT)  &&  defined(PER_SENSOR_PRECALCULATED_PARAMETER_SUPPORT)
    if (phx == FAKE_PHASE_NEUTRAL)
        return working_data.phases[0].metrology.current[1].readings.active_power;
    if (phx == FAKE_PHASE_LIVE)
        return working_data.phases[0].metrology.current[0].readings.active_power;
    #endif
    return working_data.phases[phx].readings.active_power;
}

    #if defined(REACTIVE_POWER_SUPPORT)
power_t reactive_power(int phx)
{
    if (phx == FAKE_PHASE_TOTAL)
        #if defined(TOTAL_REACTIVE_POWER_SUPPORT)
        return working_data.totals.readings.reactive_power;
        #else
        return 0;
        #endif
    return working_data.phases[phx].readings.reactive_power;
}
    #endif

    #if defined(APPARENT_POWER_SUPPORT)
power_t apparent_power(int phx)
{
    if (phx == FAKE_PHASE_TOTAL)
        #if defined(TOTAL_APPARENT_POWER_SUPPORT)
        return working_data.totals.readings.apparent_power;
        #else
        return 0;
        #endif
    return working_data.phases[phx].readings.apparent_power;
}
    #endif

    #if defined(FUNDAMENTAL_ACTIVE_POWER_SUPPORT)
power_t fundamental_active_power(int phx)
{
    if (phx == FAKE_PHASE_TOTAL)
        #if defined(TOTAL_FUNDAMENTAL_ACTIVE_POWER_SUPPORT)
        return working_data.totals.readings.fundamental_active_power;
        #else
        return 0;
        #endif
    return working_data.phases[phx].readings.fundamental_active_power;
}
    #endif

    #if defined(FUNDAMENTAL_REACTIVE_POWER_SUPPORT)
power_t fundamental_reactive_power(int phx)
{
    if (phx == FAKE_PHASE_TOTAL)
        #if defined(TOTAL_FUNDAMENTAL_REACTIVE_POWER_SUPPORT)
        return working_data.totals.readings.fundamental_reactive_power;
        #else
        return 0;
        #endif
    return working_data.phases[phx].readings.fundamental_reactive_power;
}
    #endif

    #if defined(POWER_FACTOR_SUPPORT)
power_factor_t power_factor(int phx)
{
    return working_data.phases[phx].readings.power_factor;
}
    #endif

    #if defined(VRMS_SUPPORT)
rms_voltage_t rms_voltage(int phx)
{
    return working_data.phases[phx].readings.V_rms;
}
    #endif

    #if defined(FUNDAMENTAL_VRMS_SUPPORT)
rms_voltage_t fundamental_rms_voltage(int phx)
{
    return working_data.phases[phx].readings.fundamental_V_rms;
}
    #endif

    #if defined(VOLTAGE_THD_SUPPORT)
thd_t voltage_thd(int phx)
{
    return working_data.phases[phx].readings.voltage_thd;
}
    #endif

    #if defined(IRMS_SUPPORT)
rms_current_t rms_current(int phx)
{
        #if defined(NEUTRAL_MONITOR_SUPPORT)
            #if NUM_PHASES == 1  
                #if defined(PER_SENSOR_PRECALCULATED_PARAMETER_SUPPORT)
    if (phx == FAKE_PHASE_NEUTRAL)
        return working_data.phases[0].metrology.current[1].readings.I_rms;
    if (phx == FAKE_PHASE_LIVE)
        return working_data.phases[0].metrology.current[0].readings.I_rms;
                #endif
            #else
    if (phx == FAKE_PHASE_NEUTRAL)
        return working_data.neutral.readings.I_rms;
            #endif
        #endif
    return working_data.phases[phx].readings.I_rms;
}
    #endif

    #if defined(FUNDAMENTAL_IRMS_SUPPORT)
rms_current_t fundamental_rms_current(int phx)
{
    return working_data.phases[phx].readings.fundamental_I_rms;
}
    #endif

    #if defined(CURRENT_THD_SUPPORT)
thd_t current_thd(int phx)
{
    return working_data.phases[phx].readings.current_thd;
}
    #endif

    #if defined(MAINS_FREQUENCY_SUPPORT)
int16_t mains_frequency(int phx)
{
        #if defined(NEUTRAL_MONITOR_SUPPORT)
    if (phx == FAKE_PHASE_NEUTRAL)
        return 0;
        #endif
     return working_data.phases[phx].readings.frequency;
}
    #endif

    #if NUM_PHASES > 1  &&  defined(FUNDAMENTAL_VRMS_SUPPORT)
phase_angle_t phase_to_phase_angle(int phx)
{
    return working_data.phases[phx].readings.phase_to_phase_angle;
}
    #endif

    #if NUM_PHASES > 1  &&  defined(NEUTRAL_MONITOR_SUPPORT) &&  defined(RESIDUAL_IRMS_SUPPORT)
rms_current_t residual_3phase_rms_current(void)
{
    return working_data.neutral.readings.residual_I_rms;
}
    #endif
#else
power_t active_power(int phx)
{
    return evaluate_active_power(&working_data.phases[phx], &cal_info->phases[phx]);
}

    #if defined(REACTIVE_POWER_SUPPORT)
power_t reactive_power(int phx)
{
    return evaluate_reactive_power(&working_data.phases[phx], &cal_info->phases[phx]);
}
    #endif

    #if defined(APPARENT_POWER_SUPPORT)
power_t apparent_power(int phx)
{
    return evaulate_apparent_power(&working_data.phases[phx], &cal_info->phases[phx]);
}
    #endif

    #if defined(FUNDAMENTAL_ACTIVE_POWER_SUPPORT)
power_t fundamental_active_power(int phx)
{
    if (phx == FAKE_PHASE_TOTAL)
        #if defined(TOTAL_FUNDAMENTAL_ACTIVE_POWER_SUPPORT)
        return working_data.totals.readings.fundamental_active_power;
        #else
        return 0;
        #endif
    return evaluate_fundamental_active_power(&working_data.phases[phx], &cal_info->phases[phx]);
}
    #endif

    #if defined(FUNDAMENTAL_REACTIVE_POWER_SUPPORT)
power_t fundamental_reactive_power(int phx)
{
    if (phx == FAKE_PHASE_TOTAL)
        #if defined(TOTAL_FUNDAMENTAL_REACTIVE_POWER_SUPPORT)
        return working_data.totals.readings.fundamental_reactive_power;
        #else
        return 0;
        #endif
    return evaluate_fundamental_reactive_power(&working_data.phases[phx], &cal_info->phases[phx]);
}
    #endif

    #if defined(POWER_FACTOR_SUPPORT)
power_factor_t power_factor(int phx)
{
    return evaluate_power_factor(&working_data.phases[phx], &cal_info->phases[phx]);
}
    #endif

    #if defined(VRMS_SUPPORT)
rms_voltage_t rms_voltage(int phx)
{
    return evaluate_rms_voltage(&working_data.phases[phx], &cal_info->phases[phx]);
}
    #endif

    #if defined(FUNDAMENTAL_VRMS_SUPPORT)
rms_voltage_t fundamental_rms_voltage(int phx)
{
    return evaluate_fundamental_rms_voltage(&working_data.phases[phx], &cal_info->phases[phx]);
}
    #endif

    #if defined(VOLTAGE_THD_SUPPORT)
thd_t voltage_thd(int phx)
{
    return evaluate_voltage_thd(&working_data.phases[phx], &cal_info->phases[phx], phx);
}
    #endif

    #if defined(IRMS_SUPPORT)
rms_current_t rms_current(int phx)
{
        #if NUM_PHASES > 1  &&  defined(NEUTRAL_MONITOR_SUPPORT)
    if (phx == FAKE_PHASE_NEUTRAL)
        return evaluate_neutral_rms_current();
        #endif
    return evaluate_rms_current(&working_data.phases[phx], &cal_info->phases[phx], phx);
}
    #endif

    #if defined(FUNDAMENTAL_IRMS_SUPPORT)
rms_current_t fundamental_rms_current(int phx)
{
    return evaluate_fundamental_rms_current(&working_data.phases[phx], &cal_info->phases[phx], phx);
}
    #endif

    #if defined(CURRENT_THD_SUPPORT)
thd_t current_thd(int phx)
{
    return evaluate_current_thd(&working_data.phases[phx], &cal_info->phases[phx], phx);
}
    #endif

    #if defined(MAINS_FREQUENCY_SUPPORT)
int16_t mains_frequency(int phx)
{
    return evaluate_mains_frequency(&working_data.phases[phx], &cal_info->phases[phx]);
}
    #endif

    #if NUM_PHASES > 1  &&  defined(FUNDAMENTAL_VRMS_SUPPORT)
phase_angle_t phase_to_phase_angle(int phx)
{
    return evaluate_phase_to_phase_angle(&working_data.phases[phx], &cal_info->phases[phx]);
}
    #endif

    #if NUM_PHASES > 1  &&  defined(RESIDUAL_IRMS_SUPPORT)
rms_current_t residual_3phase_rms_current(void)
{
    return evaluate_residual_3phase_rms_current();
}
    #endif
#endif

int32_t voltage_dc_estimate(int phx)
{
    return working_data.phases[phx].metrology.V_dc_estimate_logged;
}

int32_t current_dc_estimate(int phx)
{
#if defined(NEUTRAL_MONITOR_SUPPORT)
    if (phx == FAKE_PHASE_NEUTRAL)
    #if NUM_PHASES == 1
        return working_data.phases[0].metrology.current[1].I_dc_estimate_logged;
    #else
        return working_data.neutral.metrology.I_dc_estimate_logged;
    #endif
#endif
    return working_data.phases[phx].metrology.current[0].I_dc_estimate_logged;
}

int64_t dot_product(int phx, int which, uint16_t *samples)
{
    #if defined(NEUTRAL_MONITOR_SUPPORT)
    if (phx == FAKE_PHASE_NEUTRAL)
    {
        #if NUM_PHASES == 1
        *samples = working_data.phases[0].metrology.current[1].dot_prod[1].sample_count;
        switch (which)
        {
        case DOT_PRODUCT_TYPE_P_ACTIVE:
            return assign_ac_power(working_data.phases[0].metrology.current[1].dot_prod[1].P_active);
            #if defined(FUNDAMENTAL_ACTIVE_POWER_SUPPORT)
        case DOT_PRODUCT_TYPE_P_FUNDAMENTAL_ACTIVE:
            return assign_ac_power(working_data.phases[0].metrology.current[1].dot_prod[1].P_fundamental_active);
            #endif
            #if defined(REACTIVE_POWER_BY_QUADRATURE_SUPPORT)
        case DOT_PRODUCT_TYPE_P_REACTIVE:
            return assign_ac_power(working_data.phases[0].metrology.current[1].dot_prod[1].P_reactive);
            #endif
            #if defined(FUNDAMENTAL_REACTIVE_POWER_SUPPORT)
        case DOT_PRODUCT_TYPE_P_FUNDAMENTAL_REACTIVE:
            return assign_ac_power(working_data.phases[0].metrology.current[1].dot_prod[1].P_fundamental_reactive);
            #endif
            #if defined(IRMS_SUPPORT)
        case DOT_PRODUCT_TYPE_I_SQUARED:
            return assign_ac_current(working_data.phases[0].metrology.current[1].dot_prod[1].I_sq);
            #endif
        }
        #else
        *samples = working_data.neutral.metrology.dot_prod[1].sample_count;
        switch (which)
        {
            #if defined(IRMS_SUPPORT)
        case DOT_PRODUCT_TYPE_V_SQUARED:
            return assign_ac_current(working_data.neutral.metrology.dot_prod[1].I_sq);
            #endif
            #if defined(RESIDUAL_IRMS_SUPPORT)
        case DOT_PRODUCT_TYPE_I_RESIDUAL_SQUARED:
            return assign_ac_current(working_data.neutral.metrology.dot_prod[1].residual_I_sq);
            #endif
        }
        #endif
    }
    #endif
    *samples = working_data.phases[phx].metrology.current[0].dot_prod[1].sample_count;
    switch (which)
    {
    case DOT_PRODUCT_TYPE_P_ACTIVE:
        return assign_ac_power(working_data.phases[phx].metrology.current[0].dot_prod[1].P_active);
        #if defined(FUNDAMENTAL_ACTIVE_POWER_SUPPORT)
    case DOT_PRODUCT_TYPE_P_FUNDAMENTAL_ACTIVE:
        return assign_ac_power(working_data.phases[phx].metrology.current[0].dot_prod[1].P_fundamental_active);
        #endif
        #if defined(REACTIVE_POWER_BY_QUADRATURE_SUPPORT)
    case DOT_PRODUCT_TYPE_P_REACTIVE:
        return assign_ac_power(working_data.phases[phx].metrology.current[0].dot_prod[1].P_reactive);
        #endif
        #if defined(FUNDAMENTAL_REACTIVE_POWER_SUPPORT)
    case DOT_PRODUCT_TYPE_P_FUNDAMENTAL_REACTIVE:
        return assign_ac_power(working_data.phases[phx].metrology.current[0].dot_prod[1].P_fundamental_reactive);
        #endif
        #if defined(VRMS_SUPPORT)
    case DOT_PRODUCT_TYPE_V_SQUARED:
        return assign_ac_voltage(working_data.phases[phx].metrology.dot_prod[1].V_sq);
        #endif
        #if defined(FUNDAMENTAL_ACTIVE_POWER_SUPPORT)  ||  defined(FUNDAMENTAL_REACTIVE_POWER_SUPPORT)  ||  defined(FUNDAMENTAL_VRMS_SUPPORT)  ||  defined(FUNDAMENTAL_IRMS_SUPPORT)
    case DOT_PRODUCT_TYPE_V_FUNDAMENTAL_SQUARED:
        return assign_ac_voltage(working_data.phases[phx].metrology.dot_prod[1].V_fundamental);
        #endif
        #if defined(IRMS_SUPPORT)
    case DOT_PRODUCT_TYPE_I_SQUARED:
        return assign_ac_current(working_data.phases[phx].metrology.current[0].dot_prod[1].I_sq);
        #endif
    }
    return 0;
}

int temperature(void)
{
#if defined(TEMPERATURE_SUPPORT)
    return temperature_in_celsius;
#else
    return 0x8000;
#endif
}

uint16_t phase_status(int phx)
{
    uint16_t status;

#if NUM_PHASES > 1  &&  defined(NEUTRAL_MONITOR_SUPPORT)
    if (phx == FAKE_PHASE_NEUTRAL)
    {
        status = working_data.neutral.status;
        working_data.neutral.status &= ~PHASE_STATUS_NEW_LOG;
        return status;
    }
#endif
    status = working_data.phases[phx].status;
    working_data.phases[phx].status &= ~PHASE_STATUS_NEW_LOG;
    return status;
}

uint16_t metrology_status(void)
{
    return metrology_state;
}
