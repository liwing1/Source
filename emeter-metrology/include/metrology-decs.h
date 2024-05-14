/*******************************************************************************
 *  metrology-decs.h -
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

#if !defined(_METROLOGY_DECS_H_)
#define _METROLOGY_DECS_H_

#include "metrology-nv-structs.h"

extern const struct calibration_data_s calibration_defaults;
extern const struct configuration_data_s configuration_defaults;
extern const struct info_mem_s nv_parms;

#if defined(__MSP430__)
/*! The maximum per sample change in the voltage signal, before we declare a voltage spike. */
#define MAX_PER_SAMPLE_VOLTAGE_SLEW     4096

#include "metrology-instantiate-adcs.h"
#endif

#define PHASE_SHIFT_FIR_TABLE_ELEMENTS      256

/*! The table of FIR coefficients to produce a range of phase shifts from -1/2 an ADC sample interval
    to +1/2 an ADC sample interval. When the SD16 is used, the hardware phase shifting capability of
    the ADC is used, instead of this table. */
extern const int16_t phase_shift_fir_coeffs[PHASE_SHIFT_FIR_TABLE_ELEMENTS][2];

#endif
