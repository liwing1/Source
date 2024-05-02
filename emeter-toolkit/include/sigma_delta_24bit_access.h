/*******************************************************************************
 *  sigma_delta_24bit_access.h - ways to access the 24 bit result of the various
 *                               sigma-delta variants as a 32 bit value.
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

/*! Overlay 16 and 32 bit values */
typedef union
{
    /*! A 16 bit way to access a 32 bit value */
    int16_t by16[2];
    /*! A 32 bit way to access a 32 bit value */
    int32_t by32;
} sh_lo_t;

#if defined(__MSP430_HAS_SD16_2__)  ||  defined(__MSP430_HAS_SD16_3__) \
    || \
    defined(__MSP430_HAS_SD16_A3__)  ||  defined(__MSP430_HAS_SD16_A4__)  ||  defined(__MSP430_HAS_SD16_A6__)  ||  defined(__MSP430_HAS_SD16_A7__)
static __inline__ int32_t ADC32_0(void)
{
    sh_lo_t val;

    SD16CCTL0 &= ~(SD16LSBACC | SD16IFG);
    val.by16[1] = (int16_t) SD16MEM0 >> 8;
    SD16CCTL0 |= SD16LSBACC;
    val.by16[0] = (int16_t) SD16MEM0;
    return val.by32;
}

static __inline__ int16_t ADC32_0_PENDING(void)
{
    return (SD16CCTL0 & SD16IFG);
}

static __inline__ int32_t ADC32_1(void)
{
    sh_lo_t val;

    SD16CCTL1 &= ~(SD16LSBACC | SD16IFG);
    val.by16[1] = (int16_t) SD16MEM1 >> 8;
    SD16CCTL1 |= SD16LSBACC;
    val.by16[0] = (int16_t) SD16MEM1;
    return val.by32;
}

static __inline__ int16_t ADC32_1_PENDING(void)
{
    return (SD16CCTL0 & SD16IFG);
}
#endif

#if defined(__MSP430_HAS_SD16_3__) \
    || \
    defined(__MSP430_HAS_SD16_A3__)  ||  defined(__MSP430_HAS_SD16_A4__)  ||  defined(__MSP430_HAS_SD16_A6__)  ||  defined(__MSP430_HAS_SD16_A7__)
static __inline__ int32_t ADC32_2(void)
{
    sh_lo_t val;

    SD16CCTL2 &= ~(SD16LSBACC | SD16IFG);
    val.by16[1] = (int16_t) SD16MEM2 >> 8;
    SD16CCTL2 |= SD16LSBACC;
    val.by16[0] = (int16_t) SD16MEM2;
    return val.by32;
}

static __inline__ int16_t ADC32_2_PENDING(void)
{
    return (SD16CCTL2 & SD16IFG);
}
#endif

#if defined(__MSP430_HAS_SD16_A4__)  ||  defined(__MSP430_HAS_SD16_A6__)  ||  defined(__MSP430_HAS_SD16_A7__)
static __inline__ int32_t ADC32_3(void)
{
    sh_lo_t val;

    SD16CCTL3 &= ~(SD16LSBACC | SD16IFG);
    val.by16[1] = (int16_t) SD16MEM3 >> 8;
    SD16CCTL3 |= SD16LSBACC;
    val.by16[0] = (int16_t) SD16MEM3;
    return val.by32;
}

static __inline__ int16_t ADC32_3_PENDING(void)
{
    return (SD16CCTL3 & SD16IFG);
}
#endif

#if defined(__MSP430_HAS_SD16_A6__)  ||  defined(__MSP430_HAS_SD16_A7__)
static __inline__ int32_t ADC32_4(void)
{
    sh_lo_t val;

    SD16CCTL4 &= ~(SD16LSBACC | SD16IFG);
    val.by16[1] = (int16_t) SD16MEM4 >> 8;
    SD16CCTL4 |= SD16LSBACC;
    val.by16[0] = (int16_t) SD16MEM4;
    return val.by32;
}

static __inline__ int16_t ADC32_4_PENDING(void)
{
    return (SD16CCTL4 & SD16IFG);
}

static __inline__ int32_t ADC32_5(void)
{
    sh_lo_t val;

    SD16CCTL5 &= ~(SD16LSBACC | SD16IFG);
    val.by16[1] = (int16_t) SD16MEM5 >> 8;
    SD16CCTL5 |= SD16LSBACC;
    val.by16[0] = (int16_t) SD16MEM5;
    return val.by32;
}

static __inline__ int16_t ADC32_5_PENDING(void)
{
    return (SD16CCTL5 & SD16IFG);
}
#endif

#if defined(__MSP430_HAS_SD16_A7__)
static __inline__ int32_t ADC32_6(void)
{
    sh_lo_t val;

    SD16CCTL6 &= ~(SD16LSBACC | SD16IFG);
    val.by16[1] = (int16_t) SD16MEM6 >> 8;
    SD16CCTL6 |= SD16LSBACC;
    val.by16[0] = (int16_t) SD16MEM6;
    return val.by32;
}

static __inline__ int16_t ADC32_6_PENDING(void)
{
    return (SD16CCTL6 & SD16IFG);
}
#endif

#if defined(__MSP430_HAS_SD24_2__)  ||  defined(__MSP430_HAS_SD24_3__)  ||  defined(__MSP430_HAS_SD24_4__)  ||  defined(__MSP430_HAS_SD24_A2__)  ||  defined(__MSP430_HAS_SD24_A3__)
static __inline__ int32_t ADC32_0(void)
{
    sh_lo_t val;

    SD24CCTL0 &= ~(SD24LSBACC | SD24IFG);
    val.by16[1] = (int16_t) SD24MEM0 >> 8;
    SD24CCTL0 |= SD24LSBACC;
    val.by16[0] = (int16_t) SD24MEM0;
    return val.by32;
}

static __inline__ int16_t ADC32_0_PENDING(void)
{
    return (SD24CCTL0 & SD24IFG);
}

static __inline__ int32_t ADC32_1(void)
{
    sh_lo_t val;

    SD24CCTL1 &= ~(SD24LSBACC | SD24IFG);
    val.by16[1] = (int16_t) SD24MEM1 >> 8;
    SD24CCTL1 |= SD24LSBACC;
    val.by16[0] = (int16_t) SD24MEM1;
    return val.by32;
}

static __inline__ int16_t ADC32_1_PENDING(void)
{
    return (SD24CCTL1 & SD24IFG);
}
#endif

#if defined(__MSP430_HAS_SD24_A3__)
static __inline__ int32_t ADC32_2(void)
{
    sh_lo_t val;

    SD24CCTL2 &= ~(SD24LSBACC | SD24IFG);
    val.by16[1] = (int16_t) SD24MEM2 >> 8;
    SD24CCTL2 |= SD24LSBACC;
    val.by16[0] = (int16_t) SD24MEM2;
    return val.by32;
}

static __inline__ int16_t ADC32_2_PENDING(void)
{
    return (SD24CCTL2 & SD24IFG);
}
#endif

#if defined(__MSP430_HAS_SD24_B2__)  ||  defined(__MSP430_HAS_SD24_B3__)  ||  defined(__MSP430_HAS_SD24_B4__)  ||  defined(__MSP430_HAS_SD24_B6__)  ||  defined(__MSP430_HAS_SD24_B7__)
static __inline__ int32_t ADC32_0(void)
{
    return (int32_t) SD24BMEM0_32 >> 1;
}

static __inline__ int16_t ADC32_0_PENDING(void)
{
    return (SD24BIFG & SD24IFG0);
}

static __inline__ int32_t ADC32_1(void)
{
    return (int32_t) SD24BMEM1_32 >> 1;
}

static __inline__ int16_t ADC32_1_PENDING(void)
{
    return (SD24BIFG & SD24IFG1);
}
#endif

#if defined(__MSP430_HAS_SD24_B3__)  ||  defined(__MSP430_HAS_SD24_B4__)  ||  defined(__MSP430_HAS_SD24_B6__)  ||  defined(__MSP430_HAS_SD24_B7__)
static __inline__ int32_t ADC32_2(void)
{
    return (int32_t) SD24BMEM2_32 >> 1;
}

static __inline__ int16_t ADC32_2_PENDING(void)
{
    return (SD24BIFG & SD24IFG2);
}
#endif

#if defined(__MSP430_HAS_SD24_B4__)  ||  defined(__MSP430_HAS_SD24_B6__)  ||  defined(__MSP430_HAS_SD24_B7__)
static __inline__ int32_t ADC32_3(void)
{
    return (int32_t) SD24BMEM3_32 >> 1;
}

static __inline__ int16_t ADC32_3_PENDING(void)
{
    return (SD24BIFG & SD24IFG3);
}
#endif

#if defined(__MSP430_HAS_SD24_B6__)  ||  defined(__MSP430_HAS_SD24_B7__)
static __inline__ int32_t ADC32_4(void)
{
    return (int32_t) SD24BMEM4_32 >> 1;
}

static __inline__ int16_t ADC32_4_PENDING(void)
{
    return (SD24BIFG & SD24IFG4);
}

static __inline__ int32_t ADC32_5(void)
{
    return (int32_t) SD24BMEM5_32 >> 1;
}

static __inline__ int16_t ADC32_5_PENDING(void)
{
    return (SD24BIFG & SD24IFG5);
}
#endif

#if defined(__MSP430_HAS_SD24_B7__)
static __inline__ int32_t ADC32_6(void)
{
    return (int32_t) SD24BMEM6_32 >> 1;
}

static __inline__ int16_t ADC32_6_PENDING(void)
{
    return (SD24BIFG & SD24IFG6);
}
#endif
