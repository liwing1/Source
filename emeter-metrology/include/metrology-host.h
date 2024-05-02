/*******************************************************************************
 *  metrology-host.h -
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

#if !defined(__MSP430__)

    #if defined(TWENTYFOUR_BIT)
/*! The minimum value which the ADCs can produce. */
#define V_ADC_MIN                       -31000
#define I_ADC_MIN                       (-31000L*256L)
/*! The maximum value which the ADCs can produce. */
#define V_ADC_MAX                       31000
#define I_ADC_MAX                       (31000L*256L)
    #else
/*! The minimum value which the ADCs can produce. */
#define V_ADC_MIN                       -31000
#define I_ADC_MIN                       -31000
/*! The maximum value which the ADCs can produce. */
#define V_ADC_MAX                       31000
#define I_ADC_MAX                       31000
    #endif

/*! This function is the interrupt service routine for the ADC. It performs the phase
    corrections and calculation of dot products needed to measure the mains power
    parameters. It also assesses supplementary information, such as the mains frequency.
    It controls any energy pulse LEDs/display segments. It scans and debounces any keys.
    It monitors for power failure.
    \brief ADC interrupt service routine. */
extern void adc_interrupt(void);

/*! This function opens the data source file, when the software is built as a model running on a
    host computer. */
extern int start_host_environment(int argc, char *argv[]);

/*! This function gets the ADC values for a single sample of the mains waveforms, when the software
    is built as a model running on a host computer. */
extern void get_sample(int16_t adc_buffer[]);

/*! This function loads a set of non-volatile memory parameters from a text file, when the software
    is built as a model running on a host computer. */
extern int host_load_parms(struct nv_parms_s *nv);
extern int host_store_parms(struct nv_parms_s *nv);

/*! This function logs the set of non-volatile memory parameters being used, when the software
    is built as a model running on a host computer. */
extern void log_parms(struct nv_parms_s *nv);

#endif
