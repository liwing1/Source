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

#if !defined(_METROLOGY_READINGS_H_)
#define _METROLOGY_READINGS_H_

/*! Name the three phases, and fake phase numbers for neutral and aggregate values */
enum
{
    /*! The red phase */
    PHASE_RED = 0,
    /*! The yellow phase */
    PHASE_YELLOW = 1,
    /*! The blue phase */
    PHASE_BLUE = 2,

    /*! The following do not represent actual phases, but are numbers used to select
        the appropriate information */
    FAKE_PHASE_TOTAL = 3,
    /*! For single phase this is used to select per sensor information for the neutral lead.
        For 3-phase this is used to select the neutral */
    FAKE_PHASE_NEUTRAL = 4,
    /*! For single phase this is used to select per sensor information for the live lead */
    FAKE_PHASE_LIVE = 5
};

/*! These select one of the dot product to be monitored */
enum
{
    /*! The active power dot product */
    DOT_PRODUCT_TYPE_P_ACTIVE = 0,
    /*! The reactive power dot product */
    DOT_PRODUCT_TYPE_P_REACTIVE = 1,
    /*! The voltage dot product */
    DOT_PRODUCT_TYPE_V_SQUARED = 2,
    /*! The current dot product */
    DOT_PRODUCT_TYPE_I_SQUARED = 3,
    /*! The fundamental voltage dot product */
    DOT_PRODUCT_TYPE_V_FUNDAMENTAL_SQUARED = 4,
    /*! The fundamental active power dot product */
    DOT_PRODUCT_TYPE_P_FUNDAMENTAL_ACTIVE = 5,
    /*! The fundaental reactive power dot product */
    DOT_PRODUCT_TYPE_P_FUNDAMENTAL_REACTIVE = 6,
    /*! The residual current dot product */
    DOT_PRODUCT_TYPE_I_RESIDUAL_SQUARED = 7
};

/*! These are the flags defined for each phase's status variable. */
enum
{
    /*! This flag in a channel's status variable indicates there is fresh gathered data from the
        background activity to be post-processed by the foreground activity. */
    PHASE_STATUS_NEW_LOG = 0x0001,

    /*! This flag in a channel's status variable indicates the voltage signal is currently in
        the positive half of its cycle. */
    PHASE_STATUS_V_POS = 0x0002,

    /*! This flag in a channel's status variable indicates the current signal is currently in
        the positive half of its cycle. */
    PHASE_STATUS_I_POS = 0x0004,
    PHASE_STATUS_ENERGY_LOGABLE = 0x0008,

    /*! This flag in a channel's status variable indicates the voltage signal was in overload
        during the last logged interval. Overload is determined by an excessive number of
        samples hitting the end-stops of the ADC's range. */
    PHASE_STATUS_V_OVERRANGE = 0x0010,

    /*! This flag in a channel's status variable indicates the phase current signal was in overload
        during the last logged interval. Overload is determined by an excessive number of
        samples hitting the end-stops of the ADC's range. */
    PHASE_STATUS_I_OVERRANGE = 0x0020,

    /*! This flag in a channel's status variable indicates the phase current signal was reversed
        during the last logged interval. */
    PHASE_STATUS_I_REVERSED = 0x0040,

    /*! This flag in a channel's status variable indicates the phase current signal was in overload
        during the last logged interval. Overload is determined by an excessive number of
        samples hitting the end-stops of the ADC's range. This is only used if the meter supports
        monitoring of both the live and neutral leads for anti-tamper management. */
    PHASE_STATUS_I_NEUTRAL_OVERRANGE = 0x0080,

    /*! This flag in a channel's status variable indicates the neutral current signal was
        reversed during the last logged interval. This is only used if the meter supports
        monitoring of both the live and neutral leads for anti-tamper management. */
    PHASE_STATUS_I_NEUTRAL_REVERSED = 0x0100,

    /*! This flag in a channel's status variable indicates the neutral current is the one
        currently being used. This means it has been judged by the anti-tamper logic to be
        the measurement which can best be trusted. This is only used if the meter supports
        monitoring of both the live and neutral leads for anti-tamper management. */
    PHASE_STATUS_CURRENT_FROM_NEUTRAL = 0x0200,

    /*! This flag in a channel's status variable indicates the neutral current signal is
        currently in the positive half of its cycle. This is only used if the meter supports
        monitoring of both the live and neutral leads for anti-tamper management. */
    PHASE_STATUS_I_NEUTRAL_POS = 0x0800,

    /*! This flag in a channel's status variable indicates the power has been declared to be
        reversed, after the anti-tamper logic has processed the raw indications. Live neutral
        or both leads may be reversed when this bit is set. */
    PHASE_STATUS_REVERSED = 0x1000,

    /*! This flag in a channel's status variable indicates the power (current in limp mode)
        has been declared to be unbalanced, after the anti-tamper logic has processed the
        raw indications. */
    PHASE_STATUS_UNBALANCED = 0x2000,

    /*! This flag in a channel's status variable indicates the phases order are not in ABC,
    but it's actually in CBA. */
    PHASE_STATUS_OUT_OF_ORDER = 0x4000
};

/*! Metrology status values. */
enum
{
    /*! This bit indicates the meter is in the power down state. */
    METROLOGY_STATUS_POWER_DOWN = 0x0004,

    /*! This bit indicates the current status of the meter is "current flow is reversed", after
        all persistence checking, and other safeguards, have been used to check the validity of the
        reverse indication. */
    METROLOGY_STATUS_REVERSED = 0x0100,

    /*! This bit indicates the current status of the meter is "current flow is earthed", after
        all persistence checking, and other safeguards, have been used to check the validity of the
        earthed indication. */
    METROLOGY_STATUS_EARTHED = 0x0200,

    /*! This bit indicates the phase voltage is OK. */
    METROLOGY_STATUS_PHASE_VOLTAGE_OK = 0x0400,

    /*! This bit indicates the battery condition is OK. If battery monitoring is not enabled, this bit
        is not used. */
    METROLOGY_STATUS_BATTERY_OK = 0x0800
};

/*! Operating modes */
enum
{
    /*! The meter is operating normally*/
    OPERATING_MODE_NORMAL = 0,
#if defined(LIMP_MODE_SUPPORT)
    /*! The meter is an anti-tamper meter in limp (live only) mode */
    OPERATING_MODE_LIMP = 1,
    /*! The meter is an anti-tamper meter in limp (live only) mode, reading in short bursts */
    OPERATING_MODE_LIMP_BURST = 2,
#endif
    /*! The meter is in a battery powered state with automated meter reading, LCD and RTC functioning. */
    OPERATING_MODE_AMR_ONLY = 3,
    /*! The meter is in a battery powered state with only the LCD and RTC functioning. */
    OPERATING_MODE_LCD_ONLY = 4,
    /*! The meter is in a battery powered state with only the minimum of features (probably just the RTC) functioning. */
    OPERATING_MODE_POWERFAIL = 5
};

/*! Active and apparent energy flows */
enum
{
    /*! Energy is being consumed from the grid. */
    ENERGY_DIRECTION_IMPORT = 0,
    /*! Energy is being fed to the grid. */
    ENERGY_DIRECTION_EXPORT = 1
};

/*! Reactive energy flows */
enum
{
    /*! The reactive energy in quadrant I. */
    ENERGY_QUADRANT_I = 0,
    /*! The reactive energy in quadrant II. */
    ENERGY_QUADRANT_II = 1,
    /*! The reactive energy in quadrant III. */
    ENERGY_QUADRANT_III = 2,
    /*! The reactive energy in quadrant IV. */
    ENERGY_QUADRANT_IV = 3
};

/* The various classifications of energy, which may be accumulated by the application. */
enum
{
    ACTIVE_ENERGY_IMPORTED = 0,
    ACTIVE_ENERGY_EXPORTED = 1,
    FUNDAMENTAL_ACTIVE_ENERGY_IMPORTED = 2,
    FUNDAMENTAL_ACTIVE_ENERGY_EXPORTED = 3,
    REACTIVE_ENERGY_QUADRANT_I = 4,
    REACTIVE_ENERGY_QUADRANT_II = 5,
    REACTIVE_ENERGY_QUADRANT_III = 6,
    REACTIVE_ENERGY_QUADRANT_IV = 7,
    FUNDAMENTAL_REACTIVE_ENERGY_QUADRANT_I = 8,
    FUNDAMENTAL_REACTIVE_ENERGY_QUADRANT_II = 9,
    FUNDAMENTAL_REACTIVE_ENERGY_QUADRANT_III = 10,
    FUNDAMENTAL_REACTIVE_ENERGY_QUADRANT_IV = 11,
    APPARENT_ENERGY_IMPORTED = 12,
    APPARENT_ENERGY_EXPORTED = 13,
    INTEGRATED_V2 = 14,
    INTEGRATED_I2 = 15
};

#if defined(__cplusplus)
extern "C"
{
#endif

/*! This variable specifies the current operating mode of the meter. */
extern int8_t operating_mode;
#if defined(LIMP_MODE_SUPPORT)
/*! 0 if in notmal mode. 1 if in limp (live only) mode. */
extern int normal_limp;
#endif

/*! \brief Get the active power reading for a specified phase.
    \param ph The phase number, or:
    <ul>
        <li>FAKE_PHASE_NEUTRAL may be used to obtain the active power measured from the neutral lead in single phase anti-tamper meters.</li>
        <li>FAKE_PHASE_LIVE may be used to obtain the active power measured from the live lead in single phase anti-tamper meters.</li>
        <li>FAKE_PHASE_TOTAL may be used to obtain the total active power measured from all phases.</li>
    </ul>
    \return The active power.
*/
power_t active_power(int ph);

/*! \brief Get the reactive power reading for a specified phase.
    \param ph The phase number, or:
    <ul>
        <li>FAKE_PHASE_TOTAL may be used to obtain the total reactive power measured from all phases.</li>
    </ul>
    \return The reactive power.
*/
power_t reactive_power(int ph);

/*! \brief Get the apparent power reading for a specified phase.
    \param ph The phase number, or:
    <ul>
        <li>FAKE_PHASE_TOTAL may be used to obtain the total apparent power measured from all phases.</li>
    </ul>
    \return The apparent power.
*/
power_t apparent_power(int ph);

/*! \brief Get the fundamental active power reading for a specified phase.
    \param ph The phase number, or:
    <ul>
        <li>FAKE_PHASE_TOTAL may be used to obtain the total fundamental active power measured from all phases.</li>
    </ul>
    \return The fundamental active power.
*/
power_t fundamental_active_power(int ph);

/*! \brief Get the fundamental reactive power reading for a specified phase.
    \param ph The phase number, or:
    <ul>
        <li>FAKE_PHASE_TOTAL may be used to obtain the total fundamental reactive power measured from all phases.</li>
    </ul>
    \return The fundamental reactive power.
*/
power_t fundamental_reactive_power(int ph);

/*! \brief Get the power factor reading for a specified phase.
    \param ph The phase number.
    \return The power factor.
*/
power_factor_t power_factor(int ph);

/*! \brief Get the RMS voltage reading for a specified phase.
    \param ph The phase number.
    \return The RMS voltage.
*/
rms_voltage_t rms_voltage(int ph);

/*! \brief Get the fundamental RMS voltage reading for a specified phase.
    \param ph The phase number.
    \return The fundamental RMS voltage.
*/
rms_voltage_t fundamental_rms_voltage(int ph);

/*! \brief Get the voltage THD reading for a specified phase.
    \param ph The phase number.
    \return The voltage THD.
*/
thd_t voltage_thd(int ph);

/*! \brief Get the RMS current reading for a specified phase.
    \param ph The phase number, or:
    <ul>
        <li>FAKE_PHASE_NEUTRAL may be used to obtain the RMS current measured from the neutral lead in single phase anti-tamper meters.</li>
        <li>FAKE_PHASE_LIVE may be used to obtain the RMS current measured from the live lead in single phase anti-tamper meters.</li>
    </ul>
    \return The RMS current.
*/
rms_current_t rms_current(int ph);

/*! \brief Get the fundamental RMS current reading for a specified phase.
    \param ph The phase number.
    \return The fundamental RMS current.
*/
rms_current_t fundamental_rms_current(int ph);

/*! \brief Get the current THD reading for a specified phase.
    \param ph The phase number.
    \return The current THD.
*/
thd_t current_thd(int ph);

/*! \brief Get the residual current.
    \return The residual current.
*/
rms_current_t residual_3phase_rms_current(void);

/*! \brief Get the mains frequency reading for a specified phase.
    \param ph The phase number.
    \return The mains frequency.
*/
int16_t mains_frequency(int ph);

/*! \brief Get the angle between the specified phase and the previous one.
    \return The phase difference.
*/
phase_angle_t phase_to_phase_angle(int ph);

#if !defined(ESP_SUPPORT)
int32_t voltage_dc_estimate(int ph);

int32_t current_dc_estimate(int ph);

int64_t dot_product(int ph, int which, uint16_t *samples);
#endif

#if defined(TEMPERATURE_SUPPORT)
int temperature(void);
#endif

/*! \brief Get the status flags for a specified phase.
    \return The status flags.
*/
uint16_t phase_status(int ph);

/*! \brief Get the metrology status flags.
    \return The status flags.
*/
uint16_t metrology_status(void);

void energy_update(int ph, int type, energy_t amount);

#if defined(TOTAL_ACTIVE_ENERGY_PULSES_PER_KW_HOUR)  ||  defined(ACTIVE_ENERGY_PULSES_PER_KW_HOUR)
    #if NUM_PHASES == 1
void active_energy_pulse_start(void);
void active_energy_pulse_end(void);
    #else
void active_energy_pulse_start(int ph);
void active_energy_pulse_end(int ph);
    #endif
#endif

#if defined(TOTAL_REACTIVE_ENERGY_PULSES_PER_KVAR_HOUR)  ||  defined(REACTIVE_ENERGY_PULSES_PER_KVAR_HOUR)
    #if NUM_PHASES == 1
void reactive_energy_pulse_start(void);
void reactive_energy_pulse_end(void);
    #else
void reactive_energy_pulse_start(int ph);
void reactive_energy_pulse_end(int ph);
    #endif
#endif

#if defined(TOTAL_APPARENT_ENERGY_PULSES_PER_KVAR_HOUR)  ||  defined(APPARENT_ENERGY_PULSES_PER_KVAR_HOUR)
    #if NUM_PHASES == 1
void apparent_energy_pulse_start(void);
void apparent_energy_pulse_end(void);
    #else
void apparent_energy_pulse_start(int ph);
void apparent_energy_pulse_end(int ph);
    #endif
#endif

#if defined(SAG_SWELL_SUPPORT)
/* This callback routine is called with one of the following event codes
         2 = The swell condition has continued for another mains cycle
         1 = Onset of the swell condition
         0 = The voltage has returned to normal
        -1 = Onset of the sag condition
        -2 = The sag condition has continued for another mains cycle */

enum
{
    SAG_SWELL_VOLTAGE_POWER_DOWN_SAG = -3,
    SAG_SWELL_VOLTAGE_SAG_CONTINUING = -2,
    SAG_SWELL_VOLTAGE_SAG_ONSET = -1,
    SAG_SWELL_VOLTAGE_NORMAL = 0,
    SAG_SWELL_VOLTAGE_SWELL_ONSET = 1,
    SAG_SWELL_VOLTAGE_SWELL_CONTINUING = 2,
    SAG_SWELL_VOLTAGE_POWER_DOWN_OK = 3
};

void sag_swell_event(int ph, int event);
#endif

#if defined(__cplusplus)
}
#endif

#endif
