/*******************************************************************************
 *  metrology-foreground.h -
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

#if !defined(_METROLOGY_FOREGROUND_H_)
#define _METROLOGY_FOREGROUND_H_

extern struct metrology_data_s working_data;

/*! This variable is used as a series of flag bits for various purposes in the meter. */
extern uint16_t metrology_state;

/*! The current sampling rate, in samples per second. If the meter supports a very
    low power limp mode, this value may change with the mode of the meter. */
extern int16_t samples_per_second;

void set_phase_correction(struct phase_correction_s *s, int correction);

#if defined(__HAS_SD_ADC__)
void set_sd_phase_correction(struct phase_correction_sd_s *s, int phx, int correction);
#endif

/* Callback routines, which much be provided by the application */

/*! This function switches the meter to the normal operating state. It is usually used when the meter
    is in the low power state, or limp mode, and the conditions for normal operation have been detected.
    \brief Switch the meter to its normal operating mode. */
void switch_to_normal_mode(void);

/*! This function switches the meter to the limp, or live only, mode of operatiion. It is usually used
    the meter is running, but the measured voltage is too low to be valid.
    \brief Switch the meter to limp mode (also known as live only mode). */
void switch_to_limp_mode(void);

/*! This function switches the meter to the lowe power state, where only the RTC and power monitoring
    functions continue to operate within a timer interrupt routine. This is usually used when power
    failure has been detected. This function does not return until power has been restored. The
    machine sits in LPM3 within the function, waiting for actions within an interrupt routine to wake
    it up.
    \brief Switch the meter to minimum power operation (usual on supply failure). */
void switch_to_powerfail_mode(void);

#if defined(ESP_SUPPORT)
void esp_init(void);
void esp_start_measurement(void);
void esp_start_calibration(void);
void esp_set_active(void);
void esp_set_idle(void);
#endif

#endif
