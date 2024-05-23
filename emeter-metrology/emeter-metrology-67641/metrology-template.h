/*******************************************************************************
 *  metrology-template.h - MSP430F67641 3-phase distribution version
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

/*! This selects the number of phases. */
#define NUM_PHASES 3

/*! This switch selects the use of 24 bit values for the current signal, where the
    ADC supports this. If this symbol is not defined, 16 bit values are used.
    There is nothing to be gained by using more than 16 bits for the the voltage
    signal, so we don't. */
#define TWENTYFOUR_BIT

/*! ADC channel allocation */
#define PHASE_1_CURRENT_ADC_CHANNEL                 0
#define PHASE_1_VOLTAGE_ADC_CHANNEL                 5
#define PHASE_2_CURRENT_ADC_CHANNEL                 1
#define PHASE_2_VOLTAGE_ADC_CHANNEL                 4
#define PHASE_3_CURRENT_ADC_CHANNEL                 2
#define PHASE_3_VOLTAGE_ADC_CHANNEL                 3

/*! The gain setting for the first current channel channel of the SD16,
    for devices using the SD16 sigma-delta ADC.
    This must be set to suit the shunt or CT in use. Typical values for a
    shunt are GAIN_16 (x16 gain) or GAIN_32 (x32 gain). Typical values for a
    CT are GAIN_1 (x1 gain) or GAIN_2 (x2 gain). */
#define SD_LIVE_CURRENT_GAIN                        SD24GAIN_1

#define SD_CLOCK_DIVISION                           24



/*! The nominal mains frequency, in Hz. This is used to prime the mains frequency
    measurement, and make its initial value settle quickly. It is not currently used after
    reset. */
#define MAINS_NOMINAL_FREQUENCY                     50

/*! This selects the nominal voltage used for sag/swell detection, and power calculations
    in limp mode, in volts */

#define MAINS_NOMINAL_VOLTAGE                       230


#define VCC_MEASURE_SUPPORT     

/*! This selects the basis current */
#define MAINS_BASIS_CURRENT                         10

/*! This selects the maximum operating current */
#define MAINS_MAXIMUM_CURRENT                       100



/*! This switch enables mains frequency measurement. This may be used as a
    meter feature. It may be a requirement, if non-linear CT sensors are used. */
#define MAINS_FREQUENCY_SUPPORT

/*! This selects support for RMS voltage measurement. */
#define VRMS_SUPPORT

/*! This selects support for sag and swell detection. */
#define SAG_SWELL_SUPPORT
/*! This selects the number of mains cycles over which sag and swell detection works. */
#define SAG_SWELL_WINDOW_LEN                        5
/*! This selects the percentage fall from the nominal voltage for sag detection. */
#define SAG_THRESHOLD                               20
/*! This selects the percentage rise above the nominal voltage for swell detection. */
#define SWELL_THRESHOLD                             20   





/*! This selects support for RMS current measurement. */
#define IRMS_SUPPORT




/*! This selects support for reactive power measurement. */
#define REACTIVE_POWER_SUPPORT

/*! This selects support for reactive power measurement through quadrature processing.
    This is only effective when REACTIVE_POWER_SUPPORT is enabled. */
#define REACTIVE_POWER_BY_QUADRATURE_SUPPORT


/*! The selects support for apparent or VA power measurement. */
#define APPARENT_POWER_SUPPORT

#define POWER_FACTOR_SUPPORT

/*! Tiny power levels should not record energy at all, as they may just be rounding
    errors, noise, or the consumption of the meter itself. This value is the cutoff level,
    in milliwatts. */
#define RESIDUAL_POWER_CUTOFF                       2500

/*! Tiny power levels should not record at all, as they may just be rounding errors,
    noise, or the consumption of the meter itself. This value is the cutoff level,
    in 0.01W increments. */
#define TOTAL_RESIDUAL_POWER_CUTOFF                 250



/*! This sets the number of pulses per kilo-watt hour the meter will produce at
    each phase's active energy pulse LED/opto-coupler/LCD segment. It does not affect the
    energy accumulation process. */
#define ACTIVE_ENERGY_PULSES_PER_KW_HOUR            1600


/*! This switch selects support for measuring the active energy consumption on a phase
    by phase basis. */
#define ACTIVE_ENERGY_SUPPORT

/*! This switch selects support for measuring the reactive energy consumption on a phase
    by phase basis. */
#define REACTIVE_ENERGY_SUPPORT

/*! This switch selects support for measuring the apparent energy consumption on a phase
    by phase basis. */
#define APPARENT_ENERGY_SUPPORT


/*! This switch selects support for measuring the total active energy consumption. */
#define TOTAL_ACTIVE_ENERGY_SUPPORT

/*! This switch selects support for measuring the total reactive energy consumption. */
#define TOTAL_REACTIVE_ENERGY_SUPPORT

/*! This switch selects support for measuring the total apparent energy consumption. */
#define TOTAL_APPARENT_ENERGY_SUPPORT


/*! This sets the number of pulses per kilo-watt hour the meter will produce at
    its total active energy pulse LED/opto-coupler/LCD segment. It does not affect the
    energy accumulation process. */
#define TOTAL_ACTIVE_ENERGY_PULSES_PER_KW_HOUR      1600

#define TOTAL_REACTIVE_ENERGY_PULSES_PER_KVAR_HOUR  1600

/*! The duration of the LED on time for an energy pulse. This is measured in
    ADC samples (i.e. increments 1/3276.8s). The maximum allowed is 255, giving a
    pulse of about 78ms. 163 gives a 50ms pulse. */
#define ENERGY_PULSE_DURATION                       80

/*! This switch enables use of the MSP430's internal temperature diode to
    measure the meter's temperature. */
#define TEMPERATURE_SUPPORT

/*! Normally the meter software only calculates the properly scaled values
    for voltage, current, etc. as these values are needed. This define
    enables additional global parameters, which are regularly updated with
    all the metrics gathered by the meter. This is generally less efficient,
    as it means calculating things more often than necessary. However, some
    may find this easier to use, so it is offered as a choice for the meter
    designer. */
#define PRECALCULATED_PARAMETER_SUPPORT

/*! This is called every ADC interrupt, after the main DSP work has finished.
    It can be used for things like custom keypad operations. It is important
    this is a very short routine, as it is called from the main ADC interrupt. */
#define custom_adc_interrupt()                      /**/

//Currently Disabled
    /*! This selects support for IEC style THD readings. If this is not defined, IEEE style
        readings will be produced for any THD measurements. */
    #undef IEC_THD_SUPPORT/*! This selects support for fundamental RMS voltage measurement. */
    #undef FUNDAMENTAL_VRMS_SUPPORT/*! This selects support for measuring the THD in the voltage waveform. */
    #undef VOLTAGE_THD_SUPPORT

    /*! If defined, this selects a voltage below which voltage THD will not be
        measured. This prevents silly values for THD calculated from nothing but
        noise. It is measured in millivolts. */
    #define VOLTAGE_THD_MEASUREMENT_CUTOFF              10000/*! This selects support for fundamental RMS current measurement. */
    #undef FUNDAMENTAL_IRMS_SUPPORT

    /*! This selects support for measuring the THD in the current waveform. */
    #undef CURRENT_THD_SUPPORT

    /*! If defined, this selects a voltage below which voltage THD will not be
        measured. This prevents silly values for THD calculated from nothing but
        noise. It is measured in microamps. */
    #define CURRENT_THD_MEASUREMENT_CUTOFF              100000
    /*! This selects support for fundamental active power measurement. */
    #undef FUNDAMENTAL_ACTIVE_POWER_SUPPORT
    /*! This selects support for fundamental reactive power measurement. */
    #undef FUNDAMENTAL_REACTIVE_POWER_SUPPORT




/*These features are not applicable to the TI F67641 EVM */
    #undef DYNAMIC_CURRENT_RELATED_CORRECTION_SUPPORT
    #undef DYNAMIC_FREQUENCY_RELATED_CORRECTION_SUPPORT
    #undef TRNG_SUPPORT
    #undef VOLTAGE_SIGNAL_IS_COMMON
    #undef LIMP_MODE_SUPPORT

    /*! This switch enables temperature correction of the metrology. */
    #undef TEMPERATURE_CORRECTION_SUPPORT
    #undef NEUTRAL_MONITOR_SUPPORT

