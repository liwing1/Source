/*******************************************************************************
 *  metrology-types.h -
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

#if !defined(_METROLOGY_TYPES_H_)
#define _METROLOGY_TYPES_H_

/*! Voltage measurements are represented in 1mV steps. A 32 bit signed number supports
    the range 1mV to 2147483.647V, which seems OK for all forseeable uses. */
typedef int32_t rms_voltage_t;
/*! A value used to represent the over-range condition for voltage measurements. */
#define RMS_VOLTAGE_OVERRANGE INT32_MAX
/*! The number of digits after the decimal point for voltage measurements, based on
    measurements in Volts. */
#define RMS_VOLTAGE_FRACTIONAL_DIGITS 3
/*! The multiplier to scale a value in Volts to an rms_voltage_t value. */
#define RMS_VOLTAGE_SCALING_FACTOR 1000

/*! Current measurements are represented in 1uA steps. A 32 bit signed number supports
    the range 1uA to 2147.483647A, which seems far more than will ever be needed. */
typedef int32_t rms_current_t;
/*! A value used to represent the over-range condition for current measurements. */
#define RMS_CURRENT_OVERRANGE INT32_MAX
/*! The number of digits after the decimal point for current measurements, based on
    measurements in Amperes. */
#define RMS_CURRENT_FRACTIONAL_DIGITS 6
/*! The multiplier to scale a value in Amperes to an rms_current_t value. */
#define RMS_CURRENT_SCALING_FACTOR 1000000

/*! Power measurements are represented in 1mW/mvar/mVA steps. A 32 bit signed number
    supports the range +-1mW to +-2147483.647W, which seems far more than will ever be
    needed. */
typedef int32_t power_t;
/*! A value used to represent the over-range condition for power measurements. */
#define POWER_OVERRANGE INT32_MAX
/*! The number of digits after the decimal point for power measurements, based on
    measurements in Watts. */
#define POWER_FRACTIONAL_DIGITS 3
/*! The multiplier to scale a value in Watts to a power_t value. */
#define POWER_SCALING_FACTOR 1000

/*! Mains frequencies are represented in 0.01Hz steps. A 16 bit unsigned number supports
    the range 0.01Hz to 655.35Hz, which is clearly more than we will ever need for the
    fundamental. It might not encompass all the harmonic frequencies we could be required
    to display in the future. */
typedef uint16_t frequency_t;
/*! The number of digits after the decimal point for frequency measurements, based on
    measurements in Hertz. */
#define FREQUENCY_FRACTIONAL_DIGITS 2
/*! The multiplier to scale a value in Hertz to a frequency_t value. */
#define FREQUENCY_SCALING_FACTOR 100

/*! Power factor measurements use positive values to represent leading conditions, and
    negative values for lagging conditions. +/-10000 represents unity power factor. */
typedef int16_t power_factor_t;
/*! The number of digits after the decimal point for power factor measurements. */
#define POWER_FACTOR_FRACTIONAL_DIGITS 4
/*! The multiplier to scale a power factor to a power_factor_t value. */
#define POWER_FACTOR_SCALING_FACTOR 10000

/*! Energy measurements are represented in 100mWh/varh/VAh steps. This is a signed value,
    so negative values can be used to represent exported energy. A 64 bit signed number
    supports the range 100mWh to 922337203.6854775807GWh. */
typedef int64_t energy_t;
/*! The number of digits after the decimal point for energy measurements, based on
    measurement in Watt-hours. */
#define ENERGY_FRACTIONAL_DIGITS 1
/*! The multiplier to scale a value in Watt-hours to an energy_t value. */
#define ENERGY_SCALING_FACTOR 10

/*! THD measurements are represented in 0.01% steps. A 16 bit unsigned integer supports the
    range 0% to 655.35% */
typedef uint16_t thd_t;
/*! The number of digits after the decimal point for THD measurements, based on measurement
    in percent. */
#define THD_FRACTIONAL_DIGITS 2
/*! The multiplier to scale a value in percent to a thd_t value. */
#define THD_SCALING_FACTOR 100

/*! Phase angle measurements are represented in 0.005493 degree steps. A 16 bit signed
    integer supports the range -180 degrees to +179.9945 degrees. */
typedef int16_t phase_angle_t;

#if defined(TWENTYFOUR_BIT)
/*! We use 16 bit voltage samples */
typedef int16_t voltage_sample_t;
/*! We use 24 bit current samples, stored as 32 bit integers. */
typedef int32_t current_sample_t;

/*! We use 48 bit dot product accumulators for voltage*voltage */
typedef int16_t voltage_accum_t[3];
/*! We use 64 bit dot product accumulators for voltage*current */
typedef int64_t current_accum_t[1];
/*! We use 64 bit dot product accumulators for current*current */
typedef int64_t power_accum_t[1];

/*! We use 32 bit registers for the voltage DC estimators */
typedef int32_t voltage_dc_estimate_t[1];
/*! We use 48 bit registers for the current DC estimators */
typedef int16_t current_dc_estimate_t[3];
#else
/*! We use 16 bit voltage samples */
typedef int16_t voltage_sample_t;
/*! We use 16 bit current samples */
typedef int16_t current_sample_t;

/*! We use 48 bit dot product accumulators for voltage*voltage */
typedef int16_t voltage_accum_t[3];
/*! We use 48 bit dot product accumulators for voltage*current */
typedef int16_t current_accum_t[3];
/*! We use 48 bit dot product accumulators for current*current */
typedef int16_t power_accum_t[3];

/*! We use 32 bit registers for the voltage DC estimators */
typedef int32_t voltage_dc_estimate_t[1];
/*! We use 32 bit registers for the current DC estimators */
typedef int32_t current_dc_estimate_t[1];
#endif

/*! We use 16 bit unsigned integers for most calibration factors */
typedef uint16_t calibration_scaling_factor_t;

#endif
