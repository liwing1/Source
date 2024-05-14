/*******************************************************************************
 *  metrology-nv-structs.h -
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

#if !defined(_METROLOGY_NV_STRUCTS_H_)
#define _METROLOGY_NV_STRUCTS_H_

/*! This structure contains the non-volatile parameters associated with a
    single current sensor (e.g a shunt resistor or a current transformer). */
struct current_sensor_calibration_data_s
{
    /*! An initial value for the DC estimate, to ensure rapid settling after reset. */
    int16_t initial_dc_estimate[PER_PHASE_CURRENT_CHANNELS];
    /*! An offset to remove an approximation of the front end noise from the measured RMS
        current. */
    int32_t ac_offset;
    int16_t phase_correction;
#if defined(DYNAMIC_FREQUENCY_RELATED_CORRECTION_SUPPORT)
    int16_t frequency_phase_correction;
    int16_t frequency_gain_correction;
#endif
#if defined(IRMS_SUPPORT)
    /*! \brief The calibration scaling factor for the measured RMS current in normal and limp mode. */
    calibration_scaling_factor_t I_rms_scale_factor[PER_CHANNEL_FACTORS];
#endif
    /*! The scaling factor between the raw active power value, and 0.1W increments, when in
        normal operation. */
    calibration_scaling_factor_t P_scale_factor;
};

/*! This structure contains the non-volatile parameters associated with a phase. */
struct phase_calibration_data_s
{
    /*! \brief The non-volatile information (e.g. calibration parameters) for a current channel. */
    struct current_sensor_calibration_data_s current[PER_PHASE_CURRENT_CHANNELS];
    /*! \brief A value used to prime the voltage signal DC estimate, to ensure quick settling, when the
               meter is powered up. It is set during the calibration process, based on the DC estimate
               measured at that time. */
    int16_t initial_v_dc_estimate[PER_CHANNEL_FACTORS];
#if defined(LIMP_MODE_SUPPORT)
    /*! \brief Limits for an acceptable voltage signal DC estimate. Values outside this range are considered a tamper
        condition. */
    int16_t lower_v_dc_estimate[PER_CHANNEL_FACTORS];
    int16_t upper_v_dc_estimate[PER_CHANNEL_FACTORS];
#else
#endif
    /*! An offset to remove an approximation of the front end noise from the measured RMS current. */
    int32_t v_ac_offset;
#if defined(VRMS_SUPPORT)
    /*! \brief The calibration scaling factor for the measured RMS voltage in normal and limp mode. */
    calibration_scaling_factor_t V_rms_scale_factor[PER_CHANNEL_FACTORS];
#endif
};

/*! This structure is an image of the non-volatile calibration parameters
    stored in the info memory. */
struct calibration_data_s
{
    /*! \brief The non-volatile per channel information (e.g. calibration parameters). */
    struct phase_calibration_data_s phases[NUM_PHASES];
#if NUM_PHASES > 1  &&  defined(NEUTRAL_MONITOR_SUPPORT)
    /*! \brief The non-volatile neutral lead information (e.g. calibration parameters). */
    struct current_sensor_calibration_data_s neutral;
#endif
#if defined(TEMPERATURE_SUPPORT)
    /*! \brief The room temperature at the time of calibration, in degrees Celcius */
    uint16_t temperature_at_calibration;
    /*! \brief The DC offset of readings from the temperature sensor, relative to zero Celcius. */
    int16_t temperature_sensor_intercept;
    /*! \brief The scaling factor between readings from the temperature sensor, and degrees Celcius. */
    int16_t temperature_sensor_slope;
#endif
#if defined(CORRECTED_RTC_SUPPORTx)
    /*! \brief The basic parts per million error of the 32kHz crystal. */
    int32_t crystal_base_correction;
#endif
};

struct nv_header_s
{
    /*! \brief 0xFFFF (i.e. the erased state of the flash memory) for an uncalibrated meter.
               0x0000 for a calibrated one. */
    int16_t calibration_status;
};

struct configuration_data_s
{
    uint16_t baud_rate; //0:9600 | 1:19200 | 2:57600 | 3:115200
};

struct nv_parms_s
{
    struct nv_header_s header;
    struct calibration_data_s cal_data;
    struct configuration_data_s cfg_data;
};

/*! This union overlays a segment of the info, ensuring correct
   alignment. */
struct info_mem_s
{
    union
    {
        struct nv_parms_s s;
        int8_t x[128];
    } seg_a;
};

#define cal_info                        (&nv_parms.seg_a.s.cal_data)

#if NUM_PHASES == 1
#define phase_cal                       nv_parms.seg_a.s.cal_data.phases
#endif

#endif
