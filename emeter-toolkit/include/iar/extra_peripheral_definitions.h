/*******************************************************************************
 *  extra_peripheral_definitions.h -
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

#if !defined(__EXTRA_PERPHERAL_DEFINITIONS)
#define __EXTRA_PERPHERAL_DEFINITIONS

#if defined(__MSP430_HAS_SD24_B__)  &&  defined(__MSP430F6779__)  &&  !defined(__MSP430_HAS_SD24_B7__)
#define __MSP430_HAS_SD24_B7__
#endif

#define __inline__ inline

#define DEFL(name, address) __no_init volatile unsigned long int name @ address;
#define DEFLL(name, address) __no_init volatile unsigned long long int name @ address;

#if !defined(DEFXC)
#define DEFXC  volatile unsigned char
#endif
#if !defined(DEFXW)
#define DEFXW  volatile unsigned short
#endif
#if !defined(DEFXA)
#if __REGISTER_MODEL__ == __REGISTER_MODEL_REG20__
#define DEFXA  void __data20 * volatile
#else
#define DEFXA  volatile unsigned short  /* only short access is allowed from C in small memory model */
#endif
#endif

#define DEFXL  volatile unsigned long int
#define DEFXLL volatile unsigned long long int

#if defined(__MSP430_HAS_MPY__)
/* Byte, 16 bit word and 32 bit word access to the result register of the 16 bit multiplier */

#if !defined(MPY_BASE)
#if defined(__MSP430_HAS_PMM__)
#define MPY_BASE    0x4C0
#else
#define MPY_BASE    0x130
#endif
#endif

#define RES16_32_           (MPY_BASE + 10)     /* 16x16 bit result */
#if !defined(__IAR_SYSTEMS_ICC__)
DEFL(   RES16_32          , RES16_32_)
#else
__no_init union
{
  DEFXC                               RES16_8[4];
  DEFXW                               RES16_16[2];
  DEFXL                               RES16_32;
} @ (MPY_BASE + 10);
#endif

#endif

#if defined(__MSP430_HAS_MPY32__)
/* Byte, 16 bit word and 32 bit word access to the result register of the 16 bit multiplier */

#if !defined(MPY32_BASE)
#if defined(__MSP430_HAS_PMM__)
#define MPY32_BASE  0x4C0
#else
#define MPY32_BASE  0x130
#endif
#endif

#define RES16_32_           (MPY32_BASE + 10)   /* 16x16 bit result */
#if !defined(__IAR_SYSTEMS_ICC__)
DEFL(   RES16_32          , RES16_32_)
#else
__no_init union
{
  DEFXC                               RES16_8[4];
  DEFXW                               RES16_16[2];
  DEFXL                               RES16_32;
} @ (MPY32_BASE + 10);
#endif

/* Byte, 16 bit word, 32 bit word and 64 bit word access to the registers of the 32 bit multiplier */

#define MPY32_              (MPY32_BASE + 16)   /* Multiply Unsigned Operand 1 */
#if !defined(__IAR_SYSTEMS_ICC__)
DEFL(   MPY32             , MPY32_)
#else
__no_init union
{
  DEFXC                               MPY8[4];
  DEFXW                               MPY16[2];
  DEFXL                               MPY32;
} @ (MPY32_BASE + 16);
#endif

#define MPYS32_             (MPY32_BASE + 20)   /* Multiply Signed Operand 1 */
#if !defined(__IAR_SYSTEMS_ICC__)
DEFL(   MPYS32            , MPYS32_)
#else
__no_init union
{
  DEFXC                               MPYS8[4];
  DEFXW                               MPYS16[2];
  DEFXL                               MPYS32;
} @ (MPY32_BASE + 20);
#endif

#define MAC32_              (MPY32_BASE + 24)   /* MAC Unsigned Operand 1 */
#if !defined(__IAR_SYSTEMS_ICC__)
DEFL(   MAC32             , MAC32_)
#else
__no_init union
{
  DEFXC                               MAC8[4];
  DEFXW                               MAC16[2];
  DEFXL                               MAC32;
} @ (MPY32_BASE + 24);
#endif

#define MACS32_             (MPY32_BASE + 28)   /* MAC Signed Operand 1 */
#if !defined(__IAR_SYSTEMS_ICC__)
DEFL(   MACS32            , MACS32_)
#else
__no_init union
{
  DEFXC                               MACS8[4];
  DEFXW                               MACS16[2];
  DEFXL                               MACS32;
} @ (MPY32_BASE + 28);
#endif

#define OP2_32_             (MPY32_BASE + 32)   /* Operand 2 */
#if !defined(__IAR_SYSTEMS_ICC__)
DEFL(   OP2_32            , OP2_32_)
#else
__no_init union
{
  DEFXC                               OP2_8[4];
  DEFXW                               OP2_16[2];
  DEFXL                               OP2_32X;
} @ (MPY32_BASE + 32);
#endif

#define RES64_              (MPY32_BASE + 36)   /* 32x32 bit result */
#if !defined(__IAR_SYSTEMS_ICC__)
DEFLL(  RES64             , RES64_)
#else
__no_init union
{
  DEFXC                               RES8[8];
  DEFXW                               RES16[4];
  DEFXL                               RES32[2];
  DEFXLL                              RES64;
} @ (MPY32_BASE + 36);
#endif

#endif

#if defined(__MSP430_HAS_SD24_B__)  ||  defined(__MSP430_HAS_SD24_B3__)  ||  defined(__MSP430_HAS_SD24_B4__)  ||  defined(__MSP430_HAS_SD24_B6__)  ||  defined(__MSP430_HAS_SD24_B7__)

#define SD24BMEM0_32_       (0x0850u)
#if !defined(__IAR_SYSTEMS_ICC__)
DEFL(   SD24BMEM0_32      , SD24BMEM0_32_)
#else
__no_init union
{
  DEFXC                               SD24BMEM0_8[4];
  DEFXW                               SD24BMEM0_16[2];
  DEFXL                               SD24BMEM0_32;
} @ (0x0850u);
#endif

#define SD24BMEM1_32_       (0x0854u)
#if !defined(__IAR_SYSTEMS_ICC__)
DEFL(   SD24BMEM1_32      , SD24BMEM1_32_)
#else
__no_init union
{
  DEFXC                               SD24BMEM1_8[4];
  DEFXW                               SD24BMEM1_16[2];
  DEFXL                               SD24BMEM1_32;
} @ (0x0854u);
#endif

#define SD24BMEM2_32_       (0x0858u)
#if !defined(__IAR_SYSTEMS_ICC__)
DEFL(   SD24BMEM2_32      , SD24BMEM2_32_)
#else
__no_init union
{
  DEFXC                               SD24BMEM2_8[4];
  DEFXW                               SD24BMEM2_16[2];
  DEFXL                               SD24BMEM2_32;
} @ (0x0858u);
#endif

#endif

#if defined(__MSP430_HAS_SD24_B4__)  ||  defined(__MSP430_HAS_SD24_B6__)  ||  defined(__MSP430_HAS_SD24_B7__)

#define SD24BMEM3_32_       (0x085Cu)
#if !defined(__IAR_SYSTEMS_ICC__)
DEFL(   SD24BMEM3_32      , SD24BMEM3_32_)
#else
__no_init union
{
  DEFXC                               SD24BMEM3_8[4];
  DEFXW                               SD24BMEM3_16[2];
  DEFXL                               SD24BMEM3_32;
} @ (0x085Cu);
#endif

#endif

#if defined(__MSP430_HAS_SD24_B6__)  ||  defined(__MSP430_HAS_SD24_B7__)

#define SD24BMEM4_32_       (0x0860u)
#if !defined(__IAR_SYSTEMS_ICC__)
DEFL(   SD24BMEM4_32      , SD24BMEM4_32_)
#else
__no_init union
{
  DEFXC                               SD24BMEM4_8[4];
  DEFXW                               SD24BMEM4_16[2];
  DEFXL                               SD24BMEM4_32;
} @ (0x0860u);
#endif

#define SD24BMEM5_32_       (0x0864u)
#if !defined(__IAR_SYSTEMS_ICC__)
DEFL(   SD24BMEM5_32      , SD24BMEM5_32_)
#else
__no_init union
{
  DEFXC                               SD24BMEM5_8[4];
  DEFXW                               SD24BMEM5_16[2];
  DEFXL                               SD24BMEM5_32;
} @ (0x0864u);
#endif

#endif

#if defined(__MSP430_HAS_SD24_B7__)

#define SD24BMEM6_32_       (0x0868u)
#if !defined(__IAR_SYSTEMS_ICC__)
DEFL(   SD24BMEM6_32      , SD24BMEM6_32_)
#else
__no_init union
{
  DEFXC                               SD24BMEM6_8[4];
  DEFXW                               SD24BMEM6_16[2];
  DEFXL                               SD24BMEM6_32;
} @ (0x0868u);
#endif

#endif

#endif
