/*******************************************************************************
 *  metrology-interactions.h -
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

/* Some features demand others as a prerequisite. Resolve these issues */

#if defined(ACTIVE_ENERGY_PULSES_PER_KW_HOUR)
#define ACTIVE_ENERGY_SUPPORT
#endif
#if defined(ACTIVE_ENERGY_SUPPORT)
#define ACTIVE_POWER_SUPPORT
#endif

#if defined(REACTIVE_ENERGY_PULSES_PER_KVAR_HOUR)
#define REACTIVE_ENERGY_SUPPORT
#endif
#if defined(REACTIVE_ENERGY_SUPPORT)
#define REACTIVE_POWER_SUPPORT
#endif

#if defined(APPARENT_ENERGY_PULSES_PER_KVA_HOUR)
#define APPARENT_ENERGY_SUPPORT
#endif
#if defined(APPARENT_ENERGY_SUPPORT)
#define APPARENT_POWER_SUPPORT
#endif

#if defined(ACTIVE_ENERGY_SUPPORT)  ||  defined(REACTIVE_ENERGY_SUPPORT)  ||  defined(APPARENT_ENERGY_SUPPORT)
#define ENERGY_SUPPORT
#endif

#if defined(ACTIVE_ENERGY_PULSES_PER_KW_HOUR)  ||  defined(REACTIVE_ENERGY_PULSES_PER_KVAR_HOUR)  ||  defined(APPARENT_ENERGY_PULSES_PER_KVA_HOUR)
#define ENERGY_PULSE_SUPPORT
#endif

#if defined(TOTAL_ACTIVE_ENERGY_PULSES_PER_KW_HOUR)
#define TOTAL_ACTIVE_ENERGY_SUPPORT
#endif

#if defined(TOTAL_REACTIVE_ENERGY_PULSES_PER_KVAR_HOUR)
#define TOTAL_REACTIVE_ENERGY_SUPPORT
#endif

#if defined(TOTAL_APPARENT_ENERGY_PULSES_PER_KVA_HOUR)
#define TOTAL_APPARENT_ENERGY_SUPPORT
#endif

#if defined(TOTAL_ACTIVE_ENERGY_PULSES_PER_KW_HOUR)  ||  defined(TOTAL_REACTIVE_ENERGY_PULSES_PER_KVAR_HOUR)  ||  defined(TOTAL_APPARENT_ENERGY_PULSES_PER_KVA_HOUR)
#define TOTAL_ENERGY_PULSE_SUPPORT
#endif

#if defined(TOTAL_ACTIVE_ENERGY_SUPPORT)
#define TOTAL_ACTIVE_POWER_SUPPORT
#endif

#if defined(TOTAL_REACTIVE_ENERGY_SUPPORT)
#define TOTAL_REACTIVE_POWER_SUPPORT
#endif

#if defined(TOTAL_APPARENT_ENERGY_SUPPORT)
#define TOTAL_APPARENT_POWER_SUPPORT
#endif

#if defined(TOTAL_ACTIVE_POWER_SUPPORT)  ||  defined(TOTAL_REACTIVE_POWER_SUPPORT)  ||  defined(TOTAL_APPARENT_POWER_SUPPORT)
#define TOTAL_POWER_SUPPORT
#endif

#if defined(TOTAL_ACTIVE_ENERGY_SUPPORT)  ||  defined(TOTAL_REACTIVE_ENERGY_SUPPORT)  ||  defined(TOTAL_APPARENT_ENERGY_SUPPORT)
#define TOTAL_ENERGY_SUPPORT
#endif

#if defined(TOTAL_POWER_SUPPORT)  ||  defined(TOTAL_ENERGY_SUPPORT)
#define TOTALS_SUPPORT
#endif

#if defined(TOTAL_REACTIVE_POWER_SUPPORT)
#define REACTIVE_POWER_SUPPORT
#endif

#if defined(TOTAL_APPARENT_POWER_SUPPORT)
#define APPARENT_POWER_SUPPORT
#endif

#if defined(VOLTAGE_THD_SUPPORT)
#define FUNDAMENTAL_VRMS_SUPPORT
#endif

#if defined(CURRENT_THD_SUPPORT)
#define FUNDAMENTAL_IRMS_SUPPORT
#endif

#if defined(FUNDAMENTAL_IRMS_SUPPORT)
#define FUNDAMENTAL_ACTIVE_POWER_SUPPORT
#define FUNDAMENTAL_REACTIVE_POWER_SUPPORT
#endif

#if defined(FUNDAMENTAL_ACTIVE_POWER_SUPPORT)  ||  defined(FUNDAMENTAL_REACTIVE_POWER_SUPPORT)
#define FUNDAMENTAL_POWER_SUPPORT
#endif

#if defined(FUNDAMENTAL_ACTIVE_ENERGY_SUPPORT)  ||  defined(FUNDAMENTAL_REACTIVE_ENERGY_SUPPORT)
#define FUNDAMENTAL_ENERGY_SUPPORT
#endif

#if defined(APPARENT_POWER_SUPPORT)
    #if !defined(IRMS_SUPPORT)
#define IRMS_SUPPORT
    #endif
    #if !defined(VRMS_SUPPORT)
#define VRMS_SUPPORT
    #endif
#endif

#if defined(TRNG_SUPPORT)
#define TEMPERATURE_SUPPORT)
#endif

#if NUM_PHASES == 1  &&  defined(TOTAL_POWER_SUPPORT)
#error TOTAL_POWER_SUPPORT selected for single phase metrology
#endif

/* The current channel is always zero unless we are building single phase metrology
   for anti-tamper applications. */
#if NUM_PHASES == 1  &&  defined(NEUTRAL_MONITOR_SUPPORT)
#define PER_PHASE_CURRENT_CHANNELS      2
#else
#define PER_PHASE_CURRENT_CHANNELS      1
#endif

/* The number of scaling factors, DC estimates, etc. per voltage or current channel
   depends on whether limp mode is supported. */
#if defined(LIMP_MODE_SUPPORT)
#define PER_CHANNEL_FACTORS             2
#else
#define PER_CHANNEL_FACTORS             1
#endif
