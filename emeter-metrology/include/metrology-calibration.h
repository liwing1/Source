/*******************************************************************************
 *  metrology-readings.h -
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

/*! \file */

#if !defined(_METROLOGY_CALIBRATION_H_)
#define _METROLOGY_CALIBRATION_H_

int clear_calibration_data(void);

/*! \brief Get the phase correction calibration factor for a specified phase.
    \param ph The phase number.
    \return The calibration factor.
*/
int16_t get_phase_corr(int phx);

/*! \brief Set the phase correction calibration factor for a specified phase.
    \param ph The phase number.
    \param value The calibration factor.
*/
void set_phase_corr(int phx, int16_t value);

#if defined(DYNAMIC_CURRENT_RELATED_CORRECTION_SUPPORT)  ||  defined(DYNAMIC_FREQUENCY_RELATED_CORRECTION_SUPPORT)
/*! \brief Get the frequency related phase correction calibration factor for a specified phase.
    \param ph The phase number.
    \return The calibration factor.
*/
int16_t get_frequency_related_phase_corr(int phx);

/*! \brief Set the frequency related phase correction calibration factor for a specified phase.
    \param ph The phase number.
    \param The calibration factor.
*/
void set_frequency_related_phase_corr(int phx, int16_t value);

/*! \brief Get the frequency related gain correction calibration factor for a specified phase.
    \param ph The phase number.
    \return The calibration factor.
*/
int16_t get_frequency_related_gain_corr(int phx);

/*! \brief Set the frequency related gain correction calibration factor for a specified phase.
    \param ph The phase number.
    \param The calibration factor.
*/
void set_frequency_related_gain_corr(int phx, int16_t value);
#endif

#if defined(VRMS_SUPPORT)
/*! \brief Get the voltage scaling calibration factor for a specified phase.
    \param ph The phase number.
    \return The calibration factor.
*/
calibration_scaling_factor_t get_V_rms_scaling(int phx, int which);

/*! \brief Set the voltage scaling calibration factor for a specified phase.
    \param ph The phase number.
    \param The calibration factor.
*/
void set_V_rms_scaling(int phx, int which, calibration_scaling_factor_t value);

int16_t get_v_dc_estimate(int phx, int which);

void set_v_dc_estimate(int phx, int which, int16_t value);

int32_t get_v_ac_offset(int phx);

void set_v_ac_offset(int phx, int32_t value);
#endif

#if defined(IRMS_SUPPORT)
/*! \brief Get the current scaling calibration factor for a specified phase.
    \param ph The phase number.
    \return The calibration factor.
*/
calibration_scaling_factor_t get_I_rms_scaling(int phx, int which);

/*! \brief Set the current scaling calibration factor for a specified phase.
    \param ph The phase number.
    \param The calibration factor.
*/
void set_I_rms_scaling(int phx, int which, calibration_scaling_factor_t value);

int16_t get_i_dc_estimate(int phx, int which);

void set_i_dc_estimate(int phx, int which, int16_t value);

int32_t get_i_ac_offset(int phx);

void set_i_ac_offset(int phx, int32_t value);
#endif

/*! \brief Get the power scaling calibration factor for a specified phase.
    \param ph The phase number.
    \return The calibration factor.
*/
calibration_scaling_factor_t get_P_scaling(int phx);

/*! \brief Set the power scaling calibration factor for a specified phase.
    \param ph The phase number.
    \param The calibration factor. If this is set to zero the calibration
           factor is calculated from the voltage and current calibration factors.
*/
void set_P_scaling(int phx, calibration_scaling_factor_t value);

int get_calibration_status(void);

void set_calibration_status(int value);

int16_t get_temperature_intercept(void);

int16_t get_temperature_slope(void);

void set_temperature_parameters(int16_t temperature_at_calibration, int16_t temperature_sensor_intercept, int16_t temperature_sensor_slope);

int align_metrology_with_calibration_data(void);

#if defined(__cplusplus)
}
#endif

#endif
