/*******************************************************************************
 *  sigma_delta_16bit_access.h - ways to access the 16 bit result of the various
 *                               sigma-delta variants as a 16 bit value.
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

#if defined(__MSP430_HAS_SD16_2__)  ||  defined(__MSP430_HAS_SD16_3__) \
    || \
    defined(__MSP430_HAS_SD16_A3__)  ||  defined(__MSP430_HAS_SD16_A4__)  ||  defined(__MSP430_HAS_SD16_A6__)  ||  defined(__MSP430_HAS_SD16_A7__)
static __inline__ int16_t ADC16_0(void)
{
    SD16CCTL0 &= ~SD16IFG;
    return SD16MEM0;
}

static __inline__ int16_t ADC16_0_PENDING(void)
{
    return (SD16CCTL0 & SD16IFG);
}

static __inline__ int16_t ADC16_1(void)
{
    SD16CCTL1 &= ~SD16IFG;
    return SD16MEM1;
}

static __inline__ int16_t ADC16_1_PENDING(void)
{
    return (SD16CCTL1 & SD16IFG);
}
#endif

#if defined(__MSP430_HAS_SD16_3__) \
    || \
    defined(__MSP430_HAS_SD16_A3__)  ||  defined(__MSP430_HAS_SD16_A4__)  ||  defined(__MSP430_HAS_SD16_A6__)  ||  defined(__MSP430_HAS_SD16_A7__)
static __inline__ int16_t ADC16_2(void)
{
    SD16CCTL2 &= ~SD16IFG;
    return SD16MEM2;
}

static __inline__ int16_t ADC16_2_PENDING(void)
{
    return (SD16CCTL2 & SD16IFG);
}
#endif

#if defined(__MSP430_HAS_SD16_A4__)  ||  defined(__MSP430_HAS_SD16_A6__)  ||  defined(__MSP430_HAS_SD16_A7__)
static __inline__ int16_t ADC16_3(void)
{
    SD16CCTL3 &= ~SD16IFG;
    return SD16MEM3;
}

static __inline__ int16_t ADC16_3_PENDING(void)
{
    return (SD16CCTL3 & SD16IFG);
}
#endif

#if defined(__MSP430_HAS_SD16_A6__)  ||  defined(__MSP430_HAS_SD16_A7__)
static __inline__ int16_t ADC16_4(void)
{
    SD16CCTL4 &= ~SD16IFG;
    return SD16MEM4;
}

static __inline__ int16_t ADC16_4_PENDING(void)
{
    return (SD16CCTL4 & SD16IFG);
}

static __inline__ int16_t ADC16_5(void)
{
    SD16CCTL5 &= ~SD16IFG;
    return SD16MEM5;
}

static __inline__ int16_t ADC16_5_PENDING(void)
{
    return (SD16CCTL5 & SD16IFG);
}
#endif

#if defined(__MSP430_HAS_SD16_A7__)
static __inline__ int16_t ADC16_6(void)
{
    SD16CCTL6 &= ~SD16IFG;
    return SD16MEM6;
}

static __inline__ int16_t ADC16_6_PENDING(void)
{
    return (SD16CCTL6 & SD16IFG);
}
#endif

#if defined(__MSP430_HAS_SD24_2__)  ||  defined(__MSP430_HAS_SD24_3__)  ||  defined(__MSP430_HAS_SD24_4__)  ||  defined(__MSP430_HAS_SD24_A2__)  ||  defined(__MSP430_HAS_SD24_A3__)
static __inline__ int16_t ADC16_0(void)
{
    SD24CCTL0 &= ~SD24IFG;
    return SD24MEM0;
}

static __inline__ int16_t ADC16_0_PENDING(void)
{
    return (SD24CCTL0 & SD24IFG);
}

static __inline__ int16_t ADC16_1(void)
{
    SD24CCTL1 &= ~SD24IFG;
    return SD24MEM1;
}

static __inline__ int16_t ADC16_1_PENDING(void)
{
    return (SD24CCTL1 & SD24IFG);
}
#endif

#if defined(__MSP430_HAS_SD24_A3__)
static __inline__ int16_t ADC16_2(void)
{
    SD24CCTL2 &= ~SD24IFG;
    return SD24MEM2;
}

static __inline__ int16_t ADC16_2_PENDING(void)
{
    return (SD24CCTL2 & SD24IFG);
}
#endif

#if defined(__MSP430_HAS_SD24_B2__)  ||  defined(__MSP430_HAS_SD24_B3__)  ||  defined(__MSP430_HAS_SD24_B4__)  ||  defined(__MSP430_HAS_SD24_B6__)  ||  defined(__MSP430_HAS_SD24_B7__)
static __inline__ int16_t ADC16_0(void)
{
    SD24BIFG &= ~SD24IFG0;
    return SD24BMEMH0;
}

static __inline__ int16_t ADC16_0_PENDING(void)
{
    return (SD24BIFG & SD24IFG0);
}

static __inline__ int16_t ADC16_1(void)
{
    SD24BIFG &= ~SD24IFG1;
    return SD24BMEMH1;
}

static __inline__ int16_t ADC16_1_PENDING(void)
{
    return (SD24BIFG & SD24IFG1);
}
#endif

#if defined(__MSP430_HAS_SD24_B3__)  ||  defined(__MSP430_HAS_SD24_B4__)  ||  defined(__MSP430_HAS_SD24_B6__)  ||  defined(__MSP430_HAS_SD24_B7__)
static __inline__ int16_t ADC16_2(void)
{
    SD24BIFG &= ~SD24IFG2;
    return SD24BMEMH2;
}

static __inline__ int16_t ADC16_2_PENDING(void)
{
    return (SD24BIFG & SD24IFG2);
}
#endif

#if defined(__MSP430_HAS_SD24_B4__)  ||  defined(__MSP430_HAS_SD24_B6__)  ||  defined(__MSP430_HAS_SD24_B7__)
static __inline__ int16_t ADC16_3(void)
{
    SD24BIFG &= ~SD24IFG3;
    return SD24BMEMH3;
}

static __inline__ int16_t ADC16_3_PENDING(void)
{
    return (SD24BIFG & SD24IFG3);
}
#endif

#if defined(__MSP430_HAS_SD24_B6__)  ||  defined(__MSP430_HAS_SD24_B7__)
static __inline__ int16_t ADC16_4(void)
{
    SD24BIFG &= ~SD24IFG4;
    return SD24BMEMH4;
}

static __inline__ int16_t ADC16_4_PENDING(void)
{
    return (SD24BIFG & SD24IFG4);
}

static __inline__ int16_t ADC16_5(void)
{
    SD24BIFG &= ~SD24IFG5;
    return SD24BMEMH5;
}

static __inline__ int16_t ADC16_5_PENDING(void)
{
    return (SD24BIFG & SD24IFG5);
}
#endif

#if defined(__MSP430_HAS_SD24_B7__)
static __inline__ int16_t ADC16_6(void)
{
    SD24BIFG &= ~SD24IFG6;
    return SD24BMEMH6;
}

static __inline__ int16_t ADC16_6_PENDING(void)
{
    return (SD24BIFG & SD24IFG6);
}
#endif
