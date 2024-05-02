/*******************************************************************************
 *  sar_10bit_access.h - ways to access the 10 bit result of the
 *                       ADC10 as a 16 bit value.
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

#if defined(__MSP430_HAS_ADC10__)
static __inline__ int16_t ADC16_0(void)
{
    SD16CCTL0 &= ~SD16IFG;
    return ADC10MEM0;
}

static __inline__ int16_t ADC16_0_PENDING(void)
{
    return (SD16CCTL0 & SD16IFG);
}

static __inline__ int16_t ADC16_1(void)
{
    SD16CCTL1 &= ~SD16IFG;
    return ADC10MEM1;
}

static __inline__ int16_t ADC16_1_PENDING(void)
{
    return (SD16CCTL1 & SD16IFG);
}

static __inline__ int16_t ADC16_2(void)
{
    SD16CCTL2 &= ~SD16IFG;
    return ADC10MEM2;
}

static __inline__ int16_t ADC16_3(void)
{
    SD16CCTL3 &= ~SD16IFG;
    return ADC10MEM3;
}

static __inline__ int16_t ADC16_4(void)
{
    SD16CCTL4 &= ~SD16IFG;
    return ADC10MEM4;
}

static __inline__ int16_t ADC16_5(void)
{
    SD16CCTL5 &= ~SD16IFG;
    return ADC10MEM5;
}

static __inline__ int16_t ADC16_6(void)
{
    SD16CCTL6 &= ~SD16IFG;
    return ADC10MEM6;
}

static __inline__ int16_t ADC16_7(void)
{
    SD16CCTL7 &= ~SD16IFG;
    return ADC10MEM7;
}

static __inline__ int16_t ADC16_8(void)
{
    SD16CCTL8 &= ~SD16IFG;
    return ADC10MEM8;
}

static __inline__ int16_t ADC16_9(void)
{
    SD16CCTL9 &= ~SD16IFG;
    return ADC10MEM9;
}

static __inline__ int16_t ADC16_10(void)
{
    SD16CCTL10 &= ~SD16IFG;
    return ADC10MEM10;
}
#endif
