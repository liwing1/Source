/*******************************************************************************
 *  emeter-main.h -
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

#if !defined(_METER_MAIN_H_)
#define _METER_MAIN_H_

#if !defined(NO_ENERGY_ACCUMULATION)
enum
{
#if defined(ACTIVE_ENERGY_SUPPORT)  ||  defined(TOTAL_ACTIVE_ENERGY_SUPPORT)
    APP_ACTIVE_ENERGY_IMPORTED,
    APP_ACTIVE_ENERGY_EXPORTED,
#endif
#if defined(FUNDAMENTAL_ACTIVE_ENERGY_SUPPORT)  ||  defined(TOTAL_FUNDAMENTAL_ACTIVE_ENERGY_SUPPORT)
    APP_FUNDAMENTAL_ACTIVE_ENERGY_IMPORTED,
    APP_FUNDAMENTAL_ACTIVE_ENERGY_EXPORTED,
#endif
#if defined(REACTIVE_ENERGY_SUPPORT)  ||  defined(TOTAL_REACTIVE_ENERGY_SUPPORT)
    APP_REACTIVE_ENERGY_QUADRANT_I,
    APP_REACTIVE_ENERGY_QUADRANT_II,
    APP_REACTIVE_ENERGY_QUADRANT_III,
    APP_REACTIVE_ENERGY_QUADRANT_IV,
#endif
#if defined(FUNDAMENTAL_REACTIVE_ENERGY_SUPPORT)  ||  defined(TOTAL_FUNDAMENTAL_REACTIVE_ENERGY_SUPPORT)
    APP_FUNDAMENTAL_REACTIVE_ENERGY_QUADRANT_I,
    APP_FUNDAMENTAL_REACTIVE_ENERGY_QUADRANT_II,
    APP_FUNDAMENTAL_REACTIVE_ENERGY_QUADRANT_III,
    APP_FUNDAMENTAL_REACTIVE_ENERGY_QUADRANT_IV,
#endif
#if defined(APPARENT_ENERGY_SUPPORT)  ||  defined(TOTAL_APPARENT_ENERGY_SUPPORT)
    APP_APPARENT_ENERGY_IMPORTED,
    APP_APPARENT_ENERGY_EXPORTED,
#endif
#if defined(INTEGRATED_V2_SUPPORT)  ||  defined(TOTAL_INTEGRATED_V2_SUPPORT)
    APP_INTEGRATED_V2,
#endif
#if defined(INTEGRATED_I2_SUPPORT)  ||  defined(TOTAL_INTEGRATED_I2_SUPPORT)
    APP_INTEGRATED_I2,
#endif
    APP_TOTAL_ENERGY_BINS
};

/* These are all the possible types of consumed energies */
#if NUM_PHASES == 1
extern energy_t energy_consumed[1][APP_TOTAL_ENERGY_BINS];
#else
extern energy_t energy_consumed[NUM_PHASES + 1][APP_TOTAL_ENERGY_BINS];
#endif
#endif

#if defined(SAG_SWELL_SUPPORT)
extern uint16_t sag_events[NUM_PHASES];
extern uint32_t sag_duration[NUM_PHASES]; 
extern uint16_t swell_events[NUM_PHASES];
extern uint32_t swell_duration[NUM_PHASES]; 
#endif

#endif
