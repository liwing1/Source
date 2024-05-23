/*******************************************************************************
 *  isr_compat.h -
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

#ifndef _ISR_COMPAT_H_
#define _ISR_COMPAT_H_

/* Cross compiler interrupt service routine compatibility definitions */
/* This code currently allows for:
        MSPGCC - the GNU tools for the MSP430
        Quadravox AQ430
        IAR Version 1 (old syntax)
        IAR Versions 2 and 3 (new syntax)
        Rowley Crossworks
        Code Composer Essentials

   These macros allow us to define interrupt routines for all
   compilers with a common syntax:

    ISR(<interrupt>, <routine name>)
    {
    }

   e.g.

    ISR(ADC12, adc_service_routine)
    {
        ADC12CTL0 &= ~ENC;
        ADC12CTL0 |= ENC;
    }
*/

    

#if defined(__AQCOMPILER__)
    /* This is the Quadravox compiler */
#define ISR(a,b) void _INTERRUPT[a##_VECTOR] b(void)
#elif defined(__IAR_SYSTEMS_ICC__)  &&  (((__TID__ >> 8) & 0x7f) == 43)  &&  (__VER__ < 200)
    /* This is V1.xx of the IAR compiler. */

#define ISR(a,b) interrupt[a##_VECTOR] void b(void)
#elif defined(__IAR_SYSTEMS_ICC__) // &&  (((__TID__ >> 8) & 0x7f) == 43)  &&  (__VER__ < 700)
	/* A tricky #define to stringify _Pragma parameters */

#define __PRAGMA__(x) _Pragma(#x)
    /* This is V2.xx, V3.xx, V4.xx, or V5.xx of the IAR compiler. */
#define ISR(a,b) \
__PRAGMA__(vector=a##_VECTOR) \
__interrupt void b(void)
#elif defined(__CROSSWORKS_MSP430)
    /* This is the Rowley Crossworks compiler */
#define ISR(a,b) void b __interrupt[a##_VECTOR](void)
#elif defined(__TI_COMPILER_VERSION__)
    /* This is the Code Composer Studio compiler. */
	/* A tricky #define to stringify _Pragma parameters */
#define __PRAGMA__(x) _Pragma(#x)
#define ISR(a,b) \
__PRAGMA__(vector=a##_VECTOR) \
__interrupt void a##_ISR(void)
#elif defined(__TI_COMPILER_VERSION)
    /* This is the Code Composer Essentials compiler. */
#define ISR(a,b) __interrupt void b(void); \
a##_ISR(b) \
__interrupt void b(void)
#elif defined(__GNUC__)  &&  defined(__MSP430__)
    /* This is the MSPGCC compiler */
#define ISR(a,b) __attribute__((interrupt(a##_VECTOR))) b(void)
#else
    #error Compiler not recognised.
#endif

#endif
