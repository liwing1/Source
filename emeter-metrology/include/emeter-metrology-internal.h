/*******************************************************************************
 *  emeter-metrology-internal.h -
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

#if !defined(_EMETER_METROLOGY_INTERNAL_H_)
#define _EMETER_METROLOGY_INTERNAL_H_

#include "metrology-template.h"
#include "metrology-calibration-template.h"

#if     defined(__MSP430_HAS_SD16_2__) \
    ||  defined(__MSP430_HAS_SD16_3__) \
    ||  defined(__MSP430_HAS_SD16_A3__) \
    ||  defined(__MSP430_HAS_SD16_A4__) \
    ||  defined(__MSP430_HAS_SD16_A6__) \
    ||  defined(__MSP430_HAS_SD16_A7__) \
    ||  defined(__MSP430_HAS_SD24_2__) \
    ||  defined(__MSP430_HAS_SD24_3__) \
    ||  defined(__MSP430_HAS_SD24_4__) \
    ||  defined(__MSP430_HAS_SD24_A2__) \
    ||  defined(__MSP430_HAS_SD24_A3__) \
    ||  defined(__MSP430_HAS_SD24_B__)
#define __HAS_SD_ADC__
#endif

#if defined(__HAS_SD_ADC__)
#define SAMPLE_RATE                         4096
#define ADC_BITS                            16
#define SAMPLES_PER_10_SECONDS              40960
#define LIMP_SAMPLES_PER_10_SECONDS         10240
#define LIMP_SAMPLING_RATIO                 4
#endif

#include "metrology-interactions.h"

#include "metrology-types.h"

#include "metrology-structs.h"

#include "metrology-nv-structs.h"

#include "metrology-foreground.h"

#include "metrology-background.h"

#include "metrology-corrections.h"

#include "metrology-setup.h"

#include "metrology-flash.h"

#endif
