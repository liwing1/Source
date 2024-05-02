/*******************************************************************************
 *  metrology-corrections.h -
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

#if defined(DYNAMIC_FREQUENCY_RELATED_CORRECTION_SUPPORT)
    #if NUM_PHASES == 1
        #if defined(NEUTRAL_MONITOR_SUPPORT)
void dynamic_frequency_related_correction(int ch);
        #else
void dynamic_frequency_related_correction(void);
        #endif
    #else
void dynamic_frequency_related_correction(struct phase_parms_s *phase, struct phase_calibration_data_s const *phase_cal, int ph);
    #endif
#endif

#if defined(DYNAMIC_CURRENT_RELATED_CORRECTION_SUPPORT)
    #if NUM_PHASES == 1
        #if defined(NEUTRAL_MONITOR_SUPPORT)
void dynamic_current_related_correction(int ch);
        #else
void dynamic_current_related_correction(void);
        #endif
    #else
void dynamic_current_related_correction(struct phase_parms_s *phase, struct phase_calibration_data_s const *phase_cal, int ph);
    #endif
#endif
