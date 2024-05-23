/*******************************************************************************
 *  metrology-structs.h -
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

#if !defined(_METROLOGY_STRUCTS_H_)
#define _METROLOGY_STRUCTS_H_


/* N.B.
   When built for devices using the SD16/24 converter, this software uses a sampling
   rate of 4096.0/s. For a mains supply at exactly 50Hz, each sample is
   4.3945 degrees of phase from the last. */

#define FLASH_MAIN_PAGE_SIZE                512
#define FLASH_INFO_PAGE_SIZE                128

#if !defined(I_HISTORY_STEPS)
#define I_HISTORY_STEPS 2
#endif

#if defined(TWENTYFOUR_BIT)
#define dc_filter_voltage           dc_filter16
#define dc_filter_voltage_init      dc_filter16_init
#define dc_filter_voltage_estimate  dc_filter16_estimate
#define dc_filter_current           dc_filter24
#define dc_filter_current_init      dc_filter24_init
#define dc_filter_current_estimate  dc_filter24_estimate
#define sqac_voltage                sqac48_16
#define sqac_current                sqac64_24
#define mac_power                   mac64_16_24
#define mac_voltage                 mac48_16
#define transfer_ac_voltage(y,x)    transfer48(y,x)
#define assign_ac_voltage(x)        int48_to_64(x)
#define transfer_ac_current(y,x)    (y[0] = x[0], x[0] = 0)
#define assign_ac_current(x)        x[0]
#define transfer_ac_power(y,x)      (y[0] = x[0], x[0] = 0)
#define assign_ac_power(x)          x[0]
#define div_ac_voltage(y,x)         div48(y,x)
#define div_ac_current(y,x)         *y/x
#define div_ac_power(y,x)           *y/x
#else
#define dc_filter_voltage           dc_filter16
#define dc_filter_voltage_init      dc_filter16_init
#define dc_filter_voltage_estimate  dc_filter16_estimate
#define dc_filter_current           dc_filter16
#define dc_filter_current_init      dc_filter16_init
#define dc_filter_current_estimate  dc_filter16_estimate
#define sqac_voltage                sqac48_16
#define sqac_current                sqac48_16
#define mac_power                   mac48_16
#define mac_voltage                 mac48_16
#define transfer_ac_voltage(y,x)    transfer48(y,x)
#define assign_ac_voltage(x)        int48_to_64(x)
#define transfer_ac_current(y,x)    transfer48(y,x)
#define assign_ac_current(x)        int48_to_64(x)
#define transfer_ac_power(y,x)      transfer48(y,x)
#define assign_ac_power(x)          int48_to_64(x)
#define div_ac_voltage(y,x)         div48(y,x)
#define div_ac_current(y,x)         div48(y,x)
#define div_ac_power(y,x)           div48(y,x)
#endif

#if defined(NEUTRAL_MONITOR_SUPPORT)
#define NUM_CURRENT_CHANNELS                        (NUM_PHASES + 1)
#define NEUTRAL_CURRENT_CHANNEL                     NUM_PHASES
#else
#define NUM_CURRENT_CHANNELS                        NUM_PHASES
#endif
/* This is the accumulated power value that represents one hundred milliwatt-hours
   of energy. It is equal to 100 times the number of ADC samples per hour. This
   results from the energy being accumulated at the same rate as the ADC samples,
   and the power being in 1mW increments. */
#define ENERGY_100MWATT_HOUR_THRESHOLD              (SAMPLES_PER_10_SECONDS*3600L*10L)

/* This is for the LED pulse */
#if defined(TOTAL_ACTIVE_ENERGY_PULSES_PER_KW_HOUR)
#define TOTAL_ACTIVE_ENERGY_PULSE_THRESHOLD         (ENERGY_100MWATT_HOUR_THRESHOLD/TOTAL_ACTIVE_ENERGY_PULSES_PER_KW_HOUR)*10000LL
#endif

#if defined(TOTAL_REACTIVE_ENERGY_PULSES_PER_KVAR_HOUR)
#define TOTAL_REACTIVE_ENERGY_PULSE_THRESHOLD       (ENERGY_100MWATT_HOUR_THRESHOLD/TOTAL_REACTIVE_ENERGY_PULSES_PER_KVAR_HOUR)*10000LL
#endif

#if defined(TOTAL_APPARENT_ENERGY_PULSES_PER_KVA_HOUR)
#define TOTAL_APPARENT_ENERGY_PULSE_THRESHOLD       (ENERGY_100MWATT_HOUR_THRESHOLD/TOTAL_APPARENT_ENERGY_PULSES_PER_KVA_HOUR)*10000LL
#endif

#if defined(ACTIVE_ENERGY_PULSES_PER_KW_HOUR)
#define ACTIVE_ENERGY_PULSE_THRESHOLD               (ENERGY_100MWATT_HOUR_THRESHOLD/ACTIVE_ENERGY_PULSES_PER_KW_HOUR)*10000LL
#endif

#if defined(REACTIVE_ENERGY_PULSES_PER_KVAR_HOUR)
#define REACTIVE_ENERGY_PULSE_THRESHOLD             (ENERGY_100MWATT_HOUR_THRESHOLD/REACTIVE_ENERGY_PULSES_PER_KVAR_HOUR)*10000LL
#endif

#if defined(APPARENT_ENERGY_PULSES_PER_KVA_HOUR)
#define APPARENT_ENERGY_PULSE_THRESHOLD             (ENERGY_100MWATT_HOUR_THRESHOLD/APPARENT_ENERGY_PULSES_PER_KVA_HOUR)*10000LL
#endif

/*! The following is the number of times an ADC may hit the endstops before
    it is declared to be truly overloaded. A very low value makes the meter
    less tolerant of noise spikes. A large value tends to ignore real overload. */
#define ENDSTOP_HITS_FOR_OVERLOAD                   20

/* The current channel is always zero unless we are building single phase metrology
   for anti-tamper applications. */
#if !(NUM_PHASES == 1  &&  defined(NEUTRAL_MONITOR_SUPPORT))
#define ch                                          0
#endif

#if !defined(LIMP_MODE_SUPPORT)
#define normal_limp                                 0
#endif

typedef struct
{
    int32_t v2;
    int32_t v3;
} goertzel_state_t;

struct phase_correction_s
{
    /*! \brief The step into the history data used for phase correction. */
    int16_t step;
    int16_t fir_beta;
    int16_t fir_gain;
};

struct phase_correction_sd_s
{
    /*! \brief The step into the history data used for phase correction. */
    int16_t step;
    int16_t sd_preloaded_offset;
};

struct mains_period_s
{
    /*! \brief This is a count of the number of samples to date in the current mains cycle. */
    uint16_t cycle_samples;
    /*! \brief This is the current estimate of the length of a mains cycle, in units of 1/256th
               of a sample time. */
    uint32_t period;
};

/* This structure contains the current set of readings from one phase. In anti-tamper meters,
   where there may be equivalent readings from multiple sensors, this structure is used to
   hold the readings considered definitive. */
struct phase_readings_s
{
#if defined(VRMS_SUPPORT)
    /*! \brief The RMS voltage, in 1mV steps */
    rms_voltage_t V_rms;
#endif
#if defined(FUNDAMENTAL_VRMS_SUPPORT)
    /*! \brief The fundamental RMS voltage, in 1mV steps */
    rms_voltage_t fundamental_V_rms;
    #if NUM_PHASES > 1
    /*! \brief The phase difference between the current phase and the previous one */
    phase_angle_t phase_to_phase_angle;
    #endif
#endif
#if defined(VOLTAGE_THD_SUPPORT)
    /*! \brief The THD in the voltage waveform, in 0.01% steps */
    thd_t voltage_thd;
#endif
#if defined(IRMS_SUPPORT)
    /*! \brief The RMS current, in 1uA steps */
    rms_current_t I_rms;
#endif
#if defined(FUNDAMENTAL_IRMS_SUPPORT)
    /*! \brief The fundamental RMS current, in 1uA steps */
    rms_current_t fundamental_I_rms;
#endif
#if defined(CURRENT_THD_SUPPORT)
    /*! \brief The THD in the current waveform, in 0.01% steps */
    thd_t current_thd;
#endif
    /*! \brief The active power, in steps of 10uW */
    power_t active_power;
#if defined(FUNDAMENTAL_ACTIVE_POWER_SUPPORT)
    /*! \brief The fundamental active power, in steps of 10uW */
    power_t fundamental_active_power;
#endif
#if defined(REACTIVE_POWER_SUPPORT)
    /*! \brief The reactive power, in steps of 10uW */
    power_t reactive_power;
#endif
#if defined(FUNDAMENTAL_REACTIVE_POWER_SUPPORT)
    /*! \brief The fundamental reactive power, in steps of 10uW */
    power_t fundamental_reactive_power;
#endif
#if defined(APPARENT_POWER_SUPPORT)
    /*! \brief The apparent power, in steps of 10uW */
    power_t apparent_power;
#endif
#if defined(MAINS_FREQUENCY_SUPPORT)
    /*! \brief The mains frequency, in steps of 1/100Hz */
    frequency_t frequency;
#endif
#if defined(POWER_FACTOR_SUPPORT)
    /*! \brief The power factor, in steps of 0.0001 */
    power_factor_t power_factor;
#endif
};

#if defined(PER_SENSOR_PRECALCULATED_PARAMETER_SUPPORT)
/* This structure contains the current set of readings from one current sensor. It is used in
   anti-tamper meters to hold per-sensor readings. */
struct sensor_readings_s
{
    #if defined(IRMS_SUPPORT)
    /*! \brief The RMS current, in 1uA steps */
    rms_current_t I_rms;
    #endif
    /*! \brief The active power, in 10uW steps */
    power_t active_power;
};
#endif

#if defined(NEUTRAL_MONITOR_SUPPORT)
struct neutral_readings_s
{
    #if defined(IRMS_SUPPORT)
    /*! \brief The RMS current, in steps of 1uA */
    rms_current_t I_rms;
    #endif
    #if defined(RESIDUAL_IRMS_SUPPORT)
    /*! \brief The RMS current, in steps of 1uA */
    rms_current_t residual_I_rms;
    #endif
};
#endif

struct phase_dot_prod_set_s
{
#if defined(VRMS_SUPPORT)  ||  defined(POWER_FACTOR_SUPPORT)
    /*! \brief Accumulated squares of the phase corrected voltage (V.V). These are used to
               calculate the RMS voltage, and the power factor. We need separate accumulation
               for each current sensor, as we accumulate the phase corrected voltage which matches
               the current sensor's phase shift. The actual phase correction is not important for this.
               However, if we use just one accumulated voltage we would need to use a scaling factor before
               it can be applied. This might reduce precision, so we do things this way. */
    voltage_accum_t V_sq;
#endif
#if defined(FUNDAMENTAL_ACTIVE_POWER_SUPPORT)  ||  defined(FUNDAMENTAL_REACTIVE_POWER_SUPPORT)  ||  defined(FUNDAMENTAL_VRMS_SUPPORT)  ||  defined(FUNDAMENTAL_IRMS_SUPPORT)
    /*! The V.Vpure dot product. */
    voltage_accum_t V_fundamental;
#endif
    /*! \brief This is the number of samples accumulated to date in the block of processed
               signals being gathered. */
    int16_t sample_count;
};

struct current_sensor_dot_prod_set_s
{
    /*! The active power dot product (i.e. V.I) currently in progress. */
    power_accum_t P_active;
#if defined(REACTIVE_POWER_SUPPORT)
    /*! The reactive power dot product (i.e. V90.I). */
    power_accum_t P_reactive;
#endif
#if defined(FUNDAMENTAL_ACTIVE_POWER_SUPPORT)
    /*! The active power dot product (i.e. Vpure.I). */
    power_accum_t P_fundamental_active;
#endif
#if defined(FUNDAMENTAL_REACTIVE_POWER_SUPPORT)
    /*! The reactive power dot product (i.e. Vpure90.I). */
    power_accum_t P_fundamental_reactive;
#endif
#if defined(IRMS_SUPPORT)
    /*! The current dot product (i.e. I.I). */
    current_accum_t I_sq;
#endif
    /*! The number of ADC samples gathered in the dot products. */
    int16_t sample_count;
};

struct neutral_dot_prod_set_s
{
#if defined(IRMS_SUPPORT)
    current_accum_t I_sq;
#endif
#if defined(RESIDUAL_IRMS_SUPPORT)
    current_accum_t residual_I_sq;
#endif
    /*! \brief This is the number of samples accumulated to date in the block of processed
               signals being gathered. */
    int16_t sample_count;
};

/*! This structure contains the working parameters associated with a
    single current sensor (e.g a shunt resistor or a current transformer). */
struct current_sensor_parms_s
{
    /* Channel's DC estimates, accumulated powers, and the logged
       accumulated power from the last measurement cycle. */
    /*! \brief Current channel DC estimates. We need separate estimates for each gain
               stage, as the amps may have significant temperature related DC offsets. */
    current_dc_estimate_t I_dc_estimate[PER_CHANNEL_FACTORS];
    int32_t I_dc_estimate_logged;
    /*! We use 2 sets of dot products, and ping-pong between them. At any moment one is
        in the hands of the background activity, and the other is in the hands of the
        foreground activity. */
    struct current_sensor_dot_prod_set_s dot_prod[2];
#if defined(__HAS_SD_ADC__)
    struct phase_correction_sd_s in_phase_correction;
#else
    /*! \brief The beta of the FIR/interpolator used for in-phase phase correction. */
    struct phase_correction_s in_phase_correction;
#endif
#if defined(REACTIVE_POWER_BY_QUADRATURE_SUPPORT)
    struct phase_correction_s quadrature_correction;
#endif
    /*! \brief History of the current samples, for use when making large phase corrections */
    current_sample_t I_history[I_HISTORY_STEPS];
#if defined(LIMP_MODE_SUPPORT)  &&  defined(MAINS_FREQUENCY_SUPPORT)
    struct mains_period_s period;
#endif
#if defined(PER_SENSOR_PRECALCULATED_PARAMETER_SUPPORT)
    struct sensor_readings_s readings;
#endif
    /*! \brief The number of times the ADC sample has hit one of the end-stops,
               during the current accumulation period. */
    int8_t I_endstops;
#if defined(POWER_FACTOR_SUPPORT)  &&  !defined(REACTIVE_POWER_BY_QUADRATURE_SUPPORT)
    int8_t leading;
#endif
#if defined(HARMONICS_SUPPORT)
    goertzel_state_t harm_1;
    goertzel_state_t harm_3;
    goertzel_state_t harm_5;
    int32_t pow_1;
    int32_t pow_3;
    int32_t pow_5;
    int16_t harmonic_step;
    int16_t harmonic_samples;
#endif
    /*! \brief The gain stage currently in use */
    int stage_in_use;
    /*! \brief A countdown used to check when it is safe to switch to the higher gain stage. */
    int stage_hits;
};

struct esp_phase_metrology_s
{
    /*! \brief This value counts down each time an ADC sample from the voltage
               channel hits the end of the ADC range. It is used to detect an
               overrange signal in a robust way, ignoring a small number of samples
               hits the ends of the ADC range, which might just be a few noise spikes. */
    int8_t V_endstops;

    /*! \brief Voltage channel DC estimates. We need separate estimates for normal
               and limp mode, as in limp mode we use a different ADC reference. */
    voltage_dc_estimate_t V_dc_estimate[PER_CHANNEL_FACTORS];
    int32_t V_dc_estimate_logged;

    /*! \brief The previous voltage channel sample. */
    int16_t last_V_sample;

    /*! \brief Accumulated squares of the phase corrected voltage (V.V). These are used to
               calculate the RMS voltage, and the power factor. We need separate accumulation
               for each current sensor, as we accumulate the phase corrected voltage which matches
               the current sensor's phase shift. The actual phase correction is not important for this.
               However, if we use just one accumulated voltage we would need to use a scaling factor before
               it can be applied. This might reduce precision, so we do things this way. */
    voltage_accum_t V_sq[2];

    struct mains_period_s voltage_period;

    /*! \brief This is the number of samples accumulated to date in the block of processed
               signals being gathered. */
    int16_t sample_count[2];

    /*! \brief The number of samples since the last one which did not appear to be a mains spike */
    int since_last;

    struct
    {
        /*! This value counts down each time an ADC sample from the current
            channel hits the end of the ADC range. It is used to detect an
            overrange signal in a robust way, ignoring a small number of samples
            hits the ends of the ADC range, which might just be a few noise spikes. */
        int8_t I_endstops;

        /* Channel's DC estimates, accumulated powers, and the logged
           accumulated power from the last measurement cycle. */
        current_dc_estimate_t I_dc_estimate;
        int32_t I_dc_estimate_logged;

        /*! The current dot product (i.e. I.I). */
        current_accum_t I_sq[2];

        struct mains_period_s period;

        /*! \brief This is the number of samples accumulated to date in the block of processed
                   signals being gathered. */
        int16_t sample_count[2];

        struct
        {
#if defined(IRMS_SUPPORT)
            /*! \brief The RMS current, in 1uA steps */
            rms_current_t I_rms;
#endif
            /*! \brief The active power, in 10uW steps */
            power_t active_power;
#if defined(REACTIVE_POWER_SUPPORT)
            /*! \brief The reactive power, in 10uW steps */
            power_t reactive_power;
#endif
#if defined(APPARENT_POWER_SUPPORT)
            /*! \brief The apparent power, in 10uW steps */
            power_t apparent_power;
#endif
        } readings;
#if NUM_PHASES == 1  &&  defined(NEUTRAL_MONITOR_SUPPORT)
    } current[2];
#else
    } current[1];
#endif
    /*! The index of the dot product set currently for foreground processing. */
    int8_t dp_set;
};

struct phase_metrology_s
{
    /*! \brief Voltage channel DC estimates. We need separate estimates for normal
               and limp mode, as in limp mode we use a different ADC reference. */
    voltage_dc_estimate_t V_dc_estimate[PER_CHANNEL_FACTORS];
    int32_t V_dc_estimate_logged;
    /*! \brief Voltage history, used for phase correction and/or reactive power estimation
               by the quadrature method. */
#if defined(REACTIVE_POWER_BY_QUADRATURE_SUPPORT)
    int16_t V_history[32];
#else
    int16_t V_history[4];
#endif
    /*! \brief This is a pointer to the current latest value in the voltage sample history
               buffer. */
    int8_t V_history_index;
#if defined(FUNDAMENTAL_POWER_SUPPORT)
    uint32_t pure_phase;
    int32_t pure_phase_rate;
    int32_t cross_sum;
#endif
#if defined(SAG_POWER_DOWN_SUPPORT)
    voltage_sample_t sag_power_down_threshold;
    int16_t samples_since_last_sag_power_down_threshold_crossing;
#endif
#if defined(SAG_SWELL_SUPPORT)
    voltage_accum_t V_sq_cycle;
    voltage_accum_t V_sq_prev_cycle[SAG_SWELL_WINDOW_LEN];
    voltage_accum_t V_sq_window;
    uint8_t prev_cycle_ptr;
    voltage_accum_t sag_threshold;
    voltage_accum_t swell_threshold;
    int8_t sag_status;
#endif
    /*! We use 2 sets of dot products, and ping-pong between them. At any moment one is
        in the hands of the background activity, and the other is in the hands of the
        foreground activity. */
    struct phase_dot_prod_set_s dot_prod[2];
    /*! This is the data for the current sensors. */
    struct current_sensor_parms_s current[PER_PHASE_CURRENT_CHANNELS];
#if defined(MAINS_FREQUENCY_SUPPORT)
    struct mains_period_s voltage_period;
#endif
    /*! \brief The number of samples since the last one which did not appear to be a mains spike */
    int since_last;
    /*! \brief The previous voltage channel sample. */
    int16_t last_V_sample;

    /*! \brief This value counts down each time an ADC sample from the voltage
               channel hits the end of the ADC range. It is used to detect an
               overrange signal in a robust way, ignoring a small number of samples
               hits the ends of the ADC range, which might just be a few noise spikes. */
    int8_t V_endstops;

#if defined(PHASE_REVERSED_DETECTION_SUPPORT)
    /* Persistence check counter for phase current reversal. */
    int8_t current_reversed_persistence_check;
#endif
#if defined(PHASE_UNBALANCED_DETECTION_SUPPORT)
    /* Persistence check counter current imbalance detection. */
    int8_t current_unbalanced_persistence_check;
#endif
    /*! The index of the dot product set currently for foreground processing. */
    int8_t dp_set;
};

#if defined(ENERGY_PULSE_SUPPORT)  ||  defined(TOTAL_ENERGY_PULSE_SUPPORT)
struct energy_pulse_s
{
    int64_t energy_integrator;
    uint8_t pulse_remaining_time;
};
#endif

#if defined(ENERGY_SUPPORT)
struct energy_integrator_t
{
    /*! The residual fraction of an energy increment left over after integration */
    int32_t energy_residual;
};

struct phase_energy_s
{
    #if defined(ACTIVE_ENERGY_SUPPORT)
        #if defined(ACTIVE_ENERGY_PULSES_PER_KW_HOUR)
    struct energy_pulse_s active_energy_pulse;
        #endif
    struct energy_integrator_t active[2];
    #endif
    #if defined(FUNDAMENTAL_ACTIVE_ENERGY_SUPPORT)
        #if defined(ACTIVE_ENERGY_PULSES_PER_KW_HOUR)
    struct energy_pulse_s fundamental_active_energy_pulse;
        #endif
    struct energy_integrator_t fundamental_active[2];
    #endif
    #if defined(REACTIVE_ENERGY_SUPPORT)
        #if defined(REACTIVE_ENERGY_PULSES_PER_KVAR_HOUR)
    struct energy_pulse_s reactive_energy_pulse;
        #endif
    struct energy_integrator_t reactive[4];
    #endif
    #if defined(FUNDAMENTAL_REACTIVE_ENERGY_SUPPORT)
        #if defined(REACTIVE_ENERGY_PULSES_PER_KVAR_HOUR)
    struct energy_pulse_s fundamental_reactive_energy_pulse;
        #endif
    struct energy_integrator_t fundamental_reactive[4];
    #endif
    #if defined(APPARENT_ENERGY_SUPPORT)
        #if defined(APPARENT_ENERGY_PULSES_PER_KVA_HOUR)
    struct energy_pulse_s apparent_energy_pulse;
        #endif
    struct energy_integrator_t apparent[2];
    #endif
    #if defined(INTEGRATED_V2_SUPPORT)
        #if defined(INTEGRATED_V2_PULSES_PER_KVA_HOUR)
    struct energy_pulse_s integrated_v2_energy_pulse;
        #endif
    struct energy_integrator_t integrated_v2;
    #endif
    #if defined(INTEGRATED_I2_SUPPORT)
        #if defined(INTEGRATED_I2_PULSES_PER_KVA_HOUR)
    struct energy_pulse_s integrated_i2_energy_pulse;
        #endif
    struct energy_integrator_t integrated_i2;
    #endif
};
#endif

struct neutral_metrology_s
{
    /* Channel's DC estimates, accumulated powers, and the logged
       accumulated power from the last measurement cycle. */
    current_dc_estimate_t I_dc_estimate[PER_CHANNEL_FACTORS];
    int32_t I_dc_estimate_logged;
    /*! We use 2 sets of dot products, and ping-pong between them. At any moment one is
        in the hands of the background activity, and the other is in the hands of the
        foreground activity. */
    struct neutral_dot_prod_set_s dot_prod[2];
#if defined(__HAS_SD_ADC__)
    /*! \brief History of the current samples, for use when making large phase corrections */
    current_sample_t I_history[I_HISTORY_STEPS];
    struct phase_correction_sd_s in_phase_correction;
#else
    /*! \brief The beta of the FIR/interpolator used for in-phase phase correction. */
    struct phase_correction_s in_phase_correction;
#endif
    /*! This value counts down each time an ADC sample from the current
        channel hits the end of the ADC range. It is used to detect an
        overrange signal in a robust way, ignoring a small number of samples
        hits the ends of the ADC range, which might just be a few noise spikes. */
    int8_t I_endstops;
    /*! The index of the dot product set currently for foreground processing. */
    int8_t dp_set;
};

/*! This structure holds all the working data for one phase. */
struct phase_parms_s
{
    struct phase_readings_s readings;
#if defined(ENERGY_SUPPORT)
    struct phase_energy_s energy;
#endif
#if defined(ESP_SUPPORT)
    struct esp_phase_metrology_s metrology;
#else
    struct phase_metrology_s metrology;
#endif
    /*! \brief Status bits */
    uint16_t /*volatile*/ status;
};

#if NUM_PHASES > 1  &&  defined(NEUTRAL_MONITOR_SUPPORT)
/*! This structure holds all the working data for a multi-phase meter's
    neutral monitoring. */
struct neutral_parms_s
{
    struct neutral_readings_s readings;
    struct neutral_metrology_s metrology;
    /*! \brief Status bits */
    uint16_t /*volatile*/ status;
};
#endif

#if defined(REACTIVE_POWER_BY_QUADRATURE_SUPPORT)
#define V_HISTORY_MASK      0x1F
#else
#define V_HISTORY_MASK      0x3
#endif

#if defined(TOTAL_POWER_SUPPORT)
struct total_readings_s
{
    #if defined(TOTAL_ACTIVE_POWER_SUPPORT)
    power_t active_power;
    #endif

    #if defined(TOTAL_FUNDAMENTAL_ACTIVE_POWER_SUPPORT)
    power_t fundamental_active_power;
    #endif

    #if defined(TOTAL_REACTIVE_POWER_SUPPORT)
    power_t reactive_power;
    #endif

    #if defined(TOTAL_FUNDAMENTAL_REACTIVE_POWER_SUPPORT)
    power_t fundamental_reactive_power;
    #endif

    #if defined(TOTAL_APPARENT_POWER_SUPPORT)
    power_t apparent_power;
    #endif
};
#endif

#if defined(TOTAL_ENERGY_SUPPORT)
struct total_energy_s
{
    #if defined(TOTAL_ACTIVE_ENERGY_SUPPORT)
        #if defined(TOTAL_ACTIVE_ENERGY_PULSES_PER_KW_HOUR)
    struct energy_pulse_s active_energy_pulse;
        #endif
    struct energy_integrator_t active[2];
    #endif

    #if defined(TOTAL_FUNDAMENTAL_ACTIVE_ENERGY_SUPPORT)
        #if defined(TOTAL_FUNDAMENTAL_ACTIVE_ENERGY_PULSES_PER_KW_HOUR)
    struct energy_pulse_s fundamental_active_energy_pulse;
        #endif
    struct energy_integrator_t fundamental_active[2];
    #endif

    #if defined(TOTAL_REACTIVE_ENERGY_SUPPORT)
        #if defined(TOTAL_REACTIVE_ENERGY_PULSES_PER_KVAR_HOUR)
    struct energy_pulse_s reactive_energy_pulse;
        #endif
    struct energy_integrator_t reactive[4];
    #endif

    #if defined(TOTAL_FUNDAMENTAL_REACTIVE_ENERGY_SUPPORT)
        #if defined(TOTAL_FUNDAMENTAL_REACTIVE_ENERGY_PULSES_PER_KVAR_HOUR)
    struct energy_pulse_s fundamental_reactive_energy_pulse;
        #endif
    struct energy_integrator_t fundamental_reactive[4];
    #endif

    #if defined(TOTAL_APPARENT_ENERGY_SUPPORT)
        #if defined(TOTAL_APPARENT_ENERGY_PULSES_PER_KVA_HOUR)
    struct energy_pulse_s apparent_energy_pulse;
        #endif
    struct energy_integrator_t apparent[2];
    #endif
};
#endif

#if defined(TOTALS_SUPPORT)
struct totals_s
{
    #if defined(TOTAL_POWER_SUPPORT)
    struct total_readings_s readings;
    #endif
    #if defined(TOTAL_ENERGY_SUPPORT)
    struct total_energy_s energy;
    #endif
};
#endif

#if defined(TEMPERATURE_CORRECTION_SUPPORT)
struct temperature_correction_s
{
    /* This value is a multiplication factor to compensates for variations in ADC amplitude gain
       due to temperature. It can be applied to measurements which are "amplitude" measurements,
       such as Vrms and Irms. */
    uint16_t amplitude_factor;
    /* This value is a multiplication factor to compensates for variations in ADC power gain
       due to temperature. It can be applied to measurements which are "power" measurements,
       such as active power and reactive power. It applies the square of correction applied by 
       the amplitude correction factor, which for small correction is close to twice the
       correction. */
    uint16_t power_factor;
};
#endif

struct metrology_data_s
{
    struct phase_parms_s phases[NUM_PHASES];
#if NUM_PHASES > 1  &&  defined(NEUTRAL_MONITOR_SUPPORT)
    struct neutral_parms_s neutral;
#endif
#if defined(TOTALS_SUPPORT)
    struct totals_s totals;
#endif
#if defined(TEMPERATURE_CORRECTION_SUPPORT)
    struct temperature_correction_s temperature_correction;
#endif
};

#if NUM_PHASES == 1
#define ph                              0
#define phase                           working_data.phases
#endif

#include "metrology-host.h"
#include "metrology-decs.h"
#endif
