/*******************************************************************************
 *  sar_12bit_access.h - ways to access the 12 bit result of the
 *                       ADC12 as a 16 bit value.
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

#if defined(__MSP430_HAS_ADC12__)
static __inline__ int16_t ADC16_0(void)
{
    ADC12IFG &= ~BIT0;
    return ADC12MEM0;
}

static __inline__ int16_t ADC16_0_PENDING(void)
{
    return (ADC12IFG & BIT0);
}

static __inline__ int16_t ADC16_1(void)
{
    ADC12IFG &= ~BIT1;
    return ADC12MEM1;
}

static __inline__ int16_t ADC16_1_PENDING(void)
{
    return (ADC12IFG & BIT1);
}

static __inline__ int16_t ADC16_2(void)
{
    ADC12IFG &= ~BIT2);
    return ADC12MEM2;
}

static __inline__ int16_t ADC16_2_PENDING(void)
{
    return (ADC12IFG & BIT2);
}

static __inline__ int16_t ADC16_3(void)
{
    ADC12IFG &= ~BIT3;
    return ADC12MEM3;
}

static __inline__ int16_t ADC16_3_PENDING(void)
{
    return (ADC12IFG & BIT3);
}

static __inline__ int16_t ADC16_4(void)
{
    ADC12IFG4 &= ~BIT4;
    return ADC12MEM4;
}

static __inline__ int16_t ADC16_4_PENDING(void)
{
    return (ADC12IFG & BIT4);
}

static __inline__ int16_t ADC16_5(void)
{
    ADC12IFG5 &= ~BIT5;
    return ADC12MEM5;
}

static __inline__ int16_t ADC16_5_PENDING(void)
{
    return (ADC12IFG & BIT5);
}

static __inline__ int16_t ADC16_6(void)
{
    ADC12IFG6 &= ~BIT6;
    return ADC12MEM6;
}

static __inline__ int16_t ADC16_6_PENDING(void)
{
    return (ADC12IFG & BIT6);
}

static __inline__ int16_t ADC16_7(void)
{
    ADC12IFG &= ~BIT7;
    return ADC12MEM7;
}

static __inline__ int16_t ADC16_7_PENDING(void)
{
    return (ADC12IFG & BIT7);
}

static __inline__ int16_t ADC16_8(void)
{
    ADC12IFG &= ~BIT8;
    return ADC12MEM8;
}

static __inline__ int16_t ADC16_8_PENDING(void)
{
    return (ADC12IFG & BIT8);
}

static __inline__ int16_t ADC16_9(void)
{
    ADC12IFG &= ~BIT9;
    return ADC12MEM9;
}

static __inline__ int16_t ADC16_9_PENDING(void)
{
    return (ADC12IFG & BIT9);
}

static __inline__ int16_t ADC16_10(void)
{
    ADC12IFG &= ~BIT10;
    return ADC12MEM10;
}

static __inline__ int16_t ADC16_10_PENDING(void)
{
    return (ADC12IFG & BIT10);
}

static __inline__ int16_t ADC16_11(void)
{
    ADC12IFG &= ~BIT11;
    return ADC12MEM11;
}

static __inline__ int16_t ADC16_11_PENDING(void)
{
    return (ADC12IFG & BIT11);
}

static __inline__ int16_t ADC16_12(void)
{
    ADC12IFG &= ~BIT12;
    return ADC12MEM12;
}

static __inline__ int16_t ADC16_12_PENDING(void)
{
    return (ADC12IFG & BIT12);
}

static __inline__ int16_t ADC16_13(void)
{
    ADC12IFG &= ~BIT13;
    return ADC12MEM13;
}

static __inline__ int16_t ADC16_13_PENDING(void)
{
    return (ADC12IFG & BIT13);
}

static __inline__ int16_t ADC16_14(void)
{
    ADC12IFG &= ~BIT14;
    return ADC12MEM14;
}

static __inline__ int16_t ADC16_14_PENDING(void)
{
    return (ADC12IFG & BIT14);
}

static __inline__ int16_t ADC16_51(void)
{
    ADC12IFG &= ~BIT15;
    return ADC12MEM15;
}

static __inline__ int16_t ADC16_15_PENDING(void)
{
    return (ADC12IFG & BIT15);
}
#endif
