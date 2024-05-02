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

#define __inline__ inline

#if 0
#if defined(__ASSEMBLER__)

#define sfrl(x,x_) x=x_
#define const_sfrl(x,x_) sfrl(x,x_)

#define sfrll(x,x_) x=x_
#define const_sfrll(x,x_) sfrll(x,x_)

#else

#define sfrl_(x,x_) volatile unsigned long int x __asm__("__" #x)
#define sfrl(x,x_) extern sfrl_(x,x_)
#define const_sfrl(x,x_) extern const sfrl_(x,x_)

#define sfrll_(x,x_) volatile unsigned long long int x __asm__("__" #x)
#define sfrll(x,x_) extern sfrll_(x,x_)
#define const_sfrll(x,x_) extern const sfrll_(x,x_)

#endif
#endif

#if defined(__MSP430_HAS_MPY__)
/* Byte, 16 bit word and 32 bit word access to the result register of the 16 bit multiplier */

#if !defined(MPY_BASE)
#if defined(__MSP430_HAS_PMM__)
#define MPY_BASE    0x4C0
#else
#define MPY_BASE    0x130
#endif
#endif

#define RES16_32_           0x13A //(MPY_BASE + 10)     /* 16x16 bit result */
sfrb(   RES16_8[4]        , RES16_32_)
sfrw(   RES16_16[2]       , RES16_32_)
sfrl(   RES16_32          , RES16_32_)

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

#define RES16_32_           0x13A //(MPY32_BASE + 10)   /* 16x16 bit result */
sfrb(   RES16_8[4]        , RES16_32_);
sfrw(   RES16_16[2]       , RES16_32_);
sfrl(   RES16_32          , RES16_32_);

/* Byte, 16 bit word, 32 bit word and 64 bit word access to the registers of the 32 bit multiplier */

#define MPY32_              0x140 //(MPY32_BASE + 16)   /* Multiply Unsigned Operand 1 */
sfrb(   MPY8[4]           , MPY32_);
sfrw(   MPY16[2]          , MPY32_);
sfrl(   MPY32             , MPY32_);

#define MPYS32_             0x144 //(MPY32_BASE + 20)   /* Multiply Signed Operand 1 */
sfrb(   MPYS8[4]          , MPYS32_);
sfrw(   MPYS16[2]         , MPYS32_);
sfrl(   MPYS32            , MPYS32_);

#define MAC32_              0x148 //(MPY32_BASE + 24)   /* MAC Unsigned Operand 1 */
sfrb(   MAC8[4]           , MAC32_);
sfrw(   MAC16[2]          , MAC32_);
sfrl(   MAC32             , MAC32_);

#define MACS32_             0x14C //(MPY32_BASE + 28)   /* MAC Signed Operand 1 */
sfrb(   MACS8[4]          , MACS32_);
sfrw(   MACS16[2]         , MACS32_);
sfrl(   MACS32            , MACS32_);

#define OP2_32X_            0x150 //(MPY32_BASE + 32)   /* Operand 2 */
sfrb(   OP2_8[4]          , OP2_32X_);
sfrw(   OP2_16[2]         , OP2_32X_);
sfrl(   OP2_32X           , OP2_32X_);

#define RES32_              0x154 //(MPY32_BASE + 36)   /* 32x32 bit */
sfrb(   RES8[8]           , RES32_);
sfrw(   RES16[4]          , RES32_);
sfrl(   RES32[2]          , RES32_);
sfrll(  RES64             , RES32_);

#endif

#if defined(__MSP430_HAS_SD24_B__)  ||  defined(__MSP430_HAS_SD24_B3__)  ||  defined(__MSP430_HAS_SD24_B4__)  ||  defined(__MSP430_HAS_SD24_B6__)  ||  defined(__MSP430_HAS_SD24_B7__)

#define SD24BMEM0_32_       (0x0850u)
sfrl(   SD24BMEM0_32      , SD24BMEM0_32_);

#define SD24BMEM1_32_       (0x0854u)
sfrl(   SD24BMEM1_32      , SD24BMEM1_32_);

#define SD24BMEM2_32_       (0x0858u)
sfrl(   SD24BMEM2_32      , SD24BMEM2_32_);

#endif

#if defined(__MSP430_HAS_SD24_B4__)  ||  defined(__MSP430_HAS_SD24_B6__)  ||  defined(__MSP430_HAS_SD24_B7__)

#define SD24BMEM3_32_       (0x085Cu)
sfrl(   SD24BMEM3_32      , SD24BMEM3_32_);

#endif

#if defined(__MSP430_HAS_SD24_B6__)  ||  defined(__MSP430_HAS_SD24_B7__)

#define SD24BMEM4_32_       (0x0860u)
sfrl(   SD24BMEM4_32      , SD24BMEM4_32_);

#define SD24BMEM5_32_       (0x0864u)
sfrl(   SD24BMEM5_32      , SD24BMEM5_32_);

#endif

#if defined(__MSP430_HAS_SD24_B7__)

#define SD24BMEM6_32_       (0x0868u)
sfrl(   SD24BMEM6_32      , SD24BMEM6_32_);

#endif

#endif
