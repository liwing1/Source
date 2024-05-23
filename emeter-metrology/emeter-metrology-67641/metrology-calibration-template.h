/*******************************************************************************
 *  metrology-calibration-template.h - MSP430F67641 3-phase distribution version
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



#define CALADC10_20V_30C                            *((unsigned int *)0x1A1E)
#define CALADC10_20V_85C                            *((unsigned int *)0x1A20)

#define DEFAULT_V_RMS_SCALE_FACTOR_A                48462
#define DEFAULT_V_DC_ESTIMATE_A                     20500
#define DEFAULT_V_AC_OFFSET_A                       9

#define DEFAULT_V_RMS_SCALE_FACTOR_B                48462
#define DEFAULT_V_DC_ESTIMATE_B                     20500
#define DEFAULT_V_AC_OFFSET_B                       9

#define DEFAULT_V_RMS_SCALE_FACTOR_C                48462
#define DEFAULT_V_DC_ESTIMATE_C                     20500
#define DEFAULT_V_AC_OFFSET_C                       9

#define DEFAULT_I_RMS_SCALE_FACTOR_A                19530
#define DEFAULT_I_DC_ESTIMATE_A                     0
#define DEFAULT_I_AC_OFFSET_A                       29000

#define DEFAULT_I_RMS_SCALE_FACTOR_B                19530
#define DEFAULT_I_DC_ESTIMATE_B                     0
#define DEFAULT_I_AC_OFFSET_B                       29000

#define DEFAULT_I_RMS_SCALE_FACTOR_C                19530
#define DEFAULT_I_DC_ESTIMATE_C                     0
#define DEFAULT_I_AC_OFFSET_C                       29000

/* The following will be assigned automatically if they are not set here */
#define DEFAULT_P_SCALE_FACTOR_A                    30342
#define DEFAULT_P_SCALE_FACTOR_B                    30342
#define DEFAULT_P_SCALE_FACTOR_C                    30342

/* Value is phase angle in 1/256th of a sample increments. */
#define DEFAULT_BASE_PHASE_A_CORRECTION             (-242)
#define DEFAULT_BASE_PHASE_B_CORRECTION             (-242)
#define DEFAULT_BASE_PHASE_C_CORRECTION             (-242)

/*The below macros are not used in the code.  The phase and gain macros below are not applicable to the TI F67641 EVM.
  The temperature macros could be used to switch to a more accurate way of calculating temperature(assuming the proper
  values of the slope and intercept macros, which may be device dependent.*/
    /*! This defines the default offset for the temperature diode, in ADC units */
    #define DEFAULT_TEMPERATURE_INTERCEPT               20830

    /*! This defines the default scaling factor for the temperature diode, in ADC units */
    #define DEFAULT_TEMPERATURE_SLOPE                   4400
    #define DEFAULT_ROOM_TEMPERATURE                    250
    #define DEFAULT_PHASE_CORRECTION1                   0
    #define DEFAULT_GAIN_CORRECTION1                    0
    #define DEFAULT_PHASE_CORRECTION2                   0
    #define DEFAULT_GAIN_CORRECTION2                    0

    #define DEFAULT_FREQUENCY_PHASE_FACTOR              500
    #define DEFAULT_FREQUENCY_GAIN_FACTOR               0
