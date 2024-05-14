;******************************************************************************
;  bin2bcd64.s43 (CCS version) - 64 bit binary to BCD conversion
;
;  Copyright (C) 2011 Texas Instruments Incorporated - http://www.ti.com/ 
; 
;  Redistribution and use in source and binary forms, with or without 
;  modification, are permitted provided that the following conditions 
;  are met:
;
;    Redistributions of source code must retain the above copyright 
;    notice, this list of conditions and the following disclaimer.
;
;    Redistributions in binary form must reproduce the above copyright
;    notice, this list of conditions and the following disclaimer in the 
;    documentation and/or other materials provided with the   
;    distribution.
;
;    Neither the name of Texas Instruments Incorporated nor the names of
;    its contributors may be used to endorse or promote products derived
;    from this software without specific prior written permission.
;
;  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
;  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
;  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
;  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
;  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
;  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
;  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
;  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
;  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
;  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
;  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
;
;******************************************************************************

    .cdecls C,LIST,"msp430.h"
    .include "if_macros.asm"

    ; Parameters
    .asg    R12,bcd
    ; The bin parameter is put on the stack

    ; Temporary variables
    .asg    R5,i
    .asg    R8,bin_0
    .asg    R9,bin_1
    .asg    R10,bin_2
    .asg    R11,bin_3
    .asg    R13,decimal_0
    .asg    R14,decimal_1
    .asg    R15,decimal_2
    .asg    R7,decimal_3
    .asg    R6,decimal_4

     .if $DEFINED(__LARGE_CODE_MODEL__) | $DEFINED(__LARGE_DATA_MODEL__)
STACK_USED .set 18
     .else
STACK_USED .set 9
     .endif

;void bin2bcd64(uint8_t bcd[10], uint64_t bin)
    .global bin2bcd64
    .text
    .align  2
bin2bcd64:  .asmfunc stack_usage(STACK_USED)
    pushmm  7,11
 .if $defined(__MSP430_HAS_MSP430X_CPU__)  |  $defined(__MSP430_HAS_MSP430XV2_CPU__)
    ; We just pushed 14 bytes, and the return address takes 4, so we will find our parameter 18 bytes up the stack
    mov.w   18(SP),bin_0
    mov.w   20(SP),bin_1
    mov.w   22(SP),bin_2
    mov.w   24(SP),bin_3
 .else
    ; We just pushed 14 bytes, and the return address takes 2, so we will find our parameter 16 bytes up the stack
    mov.w   16(SP),bin_0
    mov.w   18(SP),bin_1
    mov.w   20(SP),bin_2
    mov.w   22(SP),bin_3
 .endif

    clr.w   decimal_0
    clr.w   decimal_1
    clr.w   decimal_2
    clr.w   decimal_3
    clr.w   decimal_4

    mov.w   #16,i
bin2bcd64_1
    rla.w   bin_3
    dadd.w  decimal_0,decimal_0
    dadd.w  decimal_1,decimal_1
    dec.w   i
    jnz     bin2bcd64_1

    mov.w   #16,i
bin2bcd64_2
    rla.w   bin_2
    dadd.w  decimal_0,decimal_0
    dadd.w  decimal_1,decimal_1
    dadd.w  decimal_2,decimal_2
    dec.w   i
    jnz     bin2bcd64_2

    mov.w   #16,i
bin2bcd64_3
    rla.w   bin_1
    dadd.w  decimal_0,decimal_0
    dadd.w  decimal_1,decimal_1
    dadd.w  decimal_2,decimal_2
    dadd.w  decimal_3,decimal_3
    dec.w   i
    jnz     bin2bcd64_3

    mov.w   #16,i
bin2bcd64_4
    rla.w   bin_0
    dadd.w  decimal_0,decimal_0
    dadd.w  decimal_1,decimal_1
    dadd.w  decimal_2,decimal_2
    dadd.w  decimal_3,decimal_3
    dadd.w  decimal_4,decimal_4
    dec.w   i
    jnz     bin2bcd64_4

    ;We need to byte swap as we store the result
    mov.b   decimal_4,1(bcd)
    swpb    decimal_4
    mov.b   decimal_4,0(bcd)
    mov.b   decimal_3,3(bcd)
    swpb    decimal_3
    mov.b   decimal_3,2(bcd)
    mov.b   decimal_2,5(bcd)
    swpb    decimal_2
    mov.b   decimal_2,4(bcd)
    mov.b   decimal_1,7(bcd)
    swpb    decimal_1
    mov.b   decimal_1,6(bcd)
    mov.b   decimal_0,9(bcd)
    swpb    decimal_0
    mov.b   decimal_0,8(bcd)

    popmm   7,11
    xret
    .endasmfunc
    .end
