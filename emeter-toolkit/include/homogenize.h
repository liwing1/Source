/*******************************************************************************
 *  homogenize.h -
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

#if !defined(_HOMOGENIZE_H_)

/* Deal with some naming differences between devices, and try to homogenize them
   by using the most recent forms of these names. */

#if !defined(TA0R)  &&  !defined(TA0R_)
#define TA0R                TAR       /* Timer A */
#endif
#if !defined(TA0CTL)  &&  !defined(TA0CTL_)
#define TA0CTL              TACTL     /* Timer A Control */
#endif
#if !defined(TA0CCTL0)  &&  !defined(TA0CCTL0_)
#define TA0CCTL0            TACCTL0   /* Timer A Capture/Compare Control 0 */
#endif
#if !defined(TA0CCTL1)  &&  !defined(TA0CCTL1_)
#define TA0CCTL1            TACCTL1   /* Timer A Capture/Compare Control 1 */
#endif
#if !defined(TA0CCTL2)  &&  !defined(TA0CCTL2_)
#define TA0CCTL2            TACCTL2   /* Timer A Capture/Compare Control 2 */
#endif
#if !defined(TA0CCR0)  &&  !defined(TA0CCR0_)
#define TA0CCR0             TACCR0    /* Timer A Capture/Compare 0 */
#endif
#if !defined(TA0CCR1)  &&  !defined(TA0CCR1_)
#define TA0CCR1             TACCR1    /* Timer A Capture/Compare 1 */
#endif
#if !defined(TA0CCR2)  &&  !defined(TA0CCR2_)
#define TA0CCR2             TACCR2    /* Timer A Capture/Compare 2 */
#endif
#if !defined(TIMER0_A0_VECTOR)
#define TIMER0_A0_VECTOR    TIMERA0_VECTOR
#endif

/* Use naming from the EUSCI module for the USCI module */

#if defined(__MSP430_HAS_USCI_A0__)
#if !defined(USCI_UART_UCRXIFG)
#define USCI_UART_UCRXIFG USCI_UCRXIFG
#endif
#if !defined(USCI_UART_UCTXIFG)
#define USCI_UART_UCTXIFG USCI UCTXIFG
#endif
#if !defined(USCI_UART_UCSTTIFG)
#define USCI_UART_UCSTTIFG USCI UCSTTIFG
#endif
#endif

#endif
