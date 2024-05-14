/*******************************************************************************
 *  msp430_info_mem.h - Definitions of the information memory segments for the
 *                      various metering devices in the MSP430 range.
 *
 *  Copyright (C) 2012 Texas Instruments Incorporated - http://www.ti.com/ 
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

#if !defined(_MSP430_INFO_MEM_H_)
#define _MSP430_INFO_MEM_H_

#if     defined (__MSP430AFE221__) \
    ||  defined (__MSP430AFE231__) \
    ||  defined (__MSP430AFE251__) \
    ||  defined (__MSP430AFE222__) \
    ||  defined (__MSP430AFE232__) \
    ||  defined (__MSP430AFE252__) \
    ||  defined (__MSP430AFE223__) \
    ||  defined (__MSP430AFE233__) \
    ||  defined (__MSP430AFE253__)

#define __MSP430_INFO_MEM_SIZE__ 256
#define __MSP430_INFOD_MEM_SIZE__ 64
#define __MSP430_INFOC_MEM_SIZE__ 64
#define __MSP430_INFOB_MEM_SIZE__ 64
#define __MSP430_INFOA_MEM_SIZE__ 64
#define __MSP430_INFOA_MEM_IS_TLV__

#elif   defined (__MSP430F423__) \
    ||  defined (__MSP430F425__) \
    ||  defined (__MSP430F427__) \
    ||  defined (__MSP430F423A__) \
    ||  defined (__MSP430F425A__) \
    ||  defined (__MSP430F427A__) \
    ||  defined (__MSP430FE423__) \
    ||  defined (__MSP430FE425__) \
    ||  defined (__MSP430FE427__) \
    ||  defined (__MSP430FE423A__) \
    ||  defined (__MSP430FE425A__) \
    ||  defined (__MSP430FE427A__) \
    ||  defined (__MSP430FE4232__) \
    ||  defined (__MSP430FE4242__) \
    ||  defined (__MSP430FE4252__) \
    ||  defined (__MSP430FE4272__)

#define __MSP430_INFO_MEM_SIZE__ 256
#define __MSP430_INFOB_MEM_SIZE__ 128
#define __MSP430_INFOA_MEM_SIZE__ 128

#elif   defined (__MSP430F4783__) \
    ||  defined (__MSP430F4793__) \
    ||  defined (__MSP430F4784__) \
    ||  defined (__MSP430F4794__)

#define __MSP430_INFO_MEM_SIZE__ 256
#define __MSP430_INFOD_MEM_SIZE__ 64
#define __MSP430_INFOC_MEM_SIZE__ 64
#define __MSP430_INFOB_MEM_SIZE__ 64
#define __MSP430_INFOA_MEM_SIZE__ 64
#define __MSP430_INFOA_MEM_IS_TLV__

#elif   defined (__MSP430F47126__) \
    ||  defined (__MSP430F47127__) \
    ||  defined (__MSP430F47163__) \
    ||  defined (__MSP430F47173__) \
    ||  defined (__MSP430F47183__) \
    ||  defined (__MSP430F47193__) \
    ||  defined (__MSP430F47166__) \
    ||  defined (__MSP430F47176__) \
    ||  defined (__MSP430F47186__) \
    ||  defined (__MSP430F47196__) \
    ||  defined (__MSP430F47167__) \
    ||  defined (__MSP430F47177__) \
    ||  defined (__MSP430F47187__) \
    ||  defined (__MSP430F47197__)

#define __MSP430_INFO_MEM_SIZE__ 256
#define __MSP430_INFOD_MEM_SIZE__ 64
#define __MSP430_INFOC_MEM_SIZE__ 64
#define __MSP430_INFOB_MEM_SIZE__ 64
#define __MSP430_INFOA_MEM_SIZE__ 64
#define __MSP430_INFOA_MEM_IS_TLV__

#elif   defined(__MSP430F6700__) \
    ||  defined(__MSP430F6701__) \
    ||  defined(__MSP430F6702__) \
    ||  defined(__MSP430F6703__) \
    ||  defined(__MSP430F6720__) \
    ||  defined(__MSP430F6721__) \
    ||  defined(__MSP430F6722__) \
    ||  defined(__MSP430F6723__) \
    ||  defined(__MSP430F6730__) \
    ||  defined(__MSP430F6731__) \
    ||  defined(__MSP430F6732__) \
    ||  defined(__MSP430F6733__) \
    ||  defined(__MSP430F6736__) \
      ||  defined(__MSP430F67641__) \
    ||  defined(__MSP430F6749__) \
    ||  defined(__MSP430F67491__) \
    ||  defined(__MSP430F6779__) \
    ||  defined(__MSP430F67791__)

#define __MSP430_INFO_MEM_SIZE__ 512
#define __MSP430_INFOD_MEM_SIZE__ 128
#define __MSP430_INFOC_MEM_SIZE__ 128
#define __MSP430_INFOB_MEM_SIZE__ 128
#define __MSP430_INFOA_MEM_SIZE__ 128
#define __MSP430_INFOA_MEM_IS_TLV__


#elif defined (__MSP430i2040__)

#define __MSP430_INFO_MEM_SIZE__ 1024
#define __MSP430_INFOA_MEM_SIZE__ 1024
#define __MSP430_INFOA_MEM_IS_TLV__


#elif defined (__MSP430i2041__)

#define __MSP430_INFO_MEM_SIZE__ 1024
#define __MSP430_INFOA_MEM_SIZE__ 1024
#define __MSP430_INFOA_MEM_IS_TLV__


#elif defined (__MSP430i4020__)

#define __MSP430_INFO_MEM_SIZE__ 1024
#define __MSP430_INFOA_MEM_SIZE__ 1024
#define __MSP430_INFOA_MEM_IS_TLV__

#else
#error "Failed to match a device"
#endif

#if defined(__IAR_SYSTEMS_ICC__)
#define __infomem__                 _Pragma("location=\"INFO\"")
#define __infomem_uninitialized__   _Pragma("location=\"INFO\"") __no_init
    #if defined(__MSP430_INFOD_MEM_SIZE__)
#define __infod_mem__               _Pragma("location=\"INFOD\"")
#define __infod_mem_uninitialized__ _Pragma("location=\"INFOD\"") __no_init
    #endif
    #if defined(__MSP430_INFOC_MEM_SIZE__)
#define __infoc_mem__               _Pragma("location=\"INFOC\"")
#define __infoc_mem_uninitialized__ _Pragma("location=\"INFOC\"") __no_init
    #endif
    #if defined(__MSP430_INFOB_MEM_SIZE__)
#define __infob_mem__               _Pragma("location=\"INFOB\"")
#define __infob_mem_uninitialized__ _Pragma("location=\"INFOB\"") __no_init
    #endif
    #if defined(__MSP430_INFOA_MEM_SIZE__)
#define __infoa_mem__               _Pragma("location=\"INFOA\"")
#define __infoa_mem_uninitialized__ _Pragma("location=\"INFOA\"") __no_init
    #endif
#elif defined(__TI_COMPILER_VERSION__)
    #if defined(__MSP430_INFOD_MEM_SIZE__)
#define __infod_mem__               _Pragma("DATA_SECTION(nv_parms, \".infoD\")")
#define __infod_mem_uninitialized__ _Pragma("DATA_SECTION(nv_parms, \".infoD\")")
    #endif
    #if defined(__MSP430_INFOC_MEM_SIZE__)
#define __infoc_mem__               _Pragma("DATA_SECTION(nv_parms, \".infoC\")")
#define __infoc_mem_uninitialized__ _Pragma("DATA_SECTION(nv_parms, \".infoC\")")
    #endif
    #if defined(__MSP430_INFOB_MEM_SIZE__)
#define __infob_mem__               _Pragma("DATA_SECTION(nv_parms, \".infoB\")")
#define __infob_mem_uninitialized__ _Pragma("DATA_SECTION(nv_parms, \".infoB\")")
    #endif
    #if defined(__MSP430_INFOA_MEM_SIZE__)
#define __infoa_mem__               _Pragma("DATA_SECTION(nv_parms, \".infoA\")")
#define __infoa_mem_uninitialized__ _Pragma("DATA_SECTION(nv_parms, \".infoA\")")
    #endif
#elif defined(__GNUC__)
#define __infomem__                 __attribute__ ((section(".infomem")))
#define __infomem_uninitialized__   __attribute__ ((section(".infomemnobits")))
#else
#define __infomem__                 /**/
#define __infomem_uninitialized__   /**/
#endif

#endif
