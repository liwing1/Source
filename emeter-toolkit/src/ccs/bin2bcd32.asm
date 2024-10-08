;******************************************************************************
;  bin2bcd32.asm (CCS version) - 32 bit binary to BCD conversion
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
    .asg    R13,bin_ls
    .asg    R14,bin_ms

    ; Temporary variables
    .asg    R8,i
    .asg    R9,decimal_0
    .asg    R10,decimal_1
    .asg    R11,decimal_2

     .if $DEFINED(__LARGE_CODE_MODEL__) | $DEFINED(__LARGE_DATA_MODEL__)
STACK_USED .set 12
     .else
STACK_USED .set 6
     .endif

;void bin2bcd32(uint8_t bcd[5], uint32_t bin)
    .global bin2bcd32
    .text
    .align  2
bin2bcd32:  .asmfunc stack_usage(STACK_USED)
    pushmm  4,11

    clr.w   decimal_0
    clr.w   decimal_1
    clr.w   decimal_2
        
    mov.w   #16,i
bin2bcd32_1
    rla.w   bin_ms
    dadd.w  decimal_0,decimal_0
    dadd.w  decimal_1,decimal_1
    dec.w   i
    jnz     bin2bcd32_1

    mov.w   #16,i
bin2bcd32_2
    rla.w   bin_ls
    dadd.w  decimal_0,decimal_0
    dadd.w  decimal_1,decimal_1
    dadd.w  decimal_2,decimal_2
    dec.w   i
    jnz     bin2bcd32_2

    ;We need to byte swap as we store the result
    mov.b   decimal_2,0(bcd)
    mov.b   decimal_1,2(bcd)
    swpb    decimal_1
    mov.b   decimal_1,1(bcd)
    mov.b   decimal_0,4(bcd)
    swpb    decimal_0
    mov.b   decimal_0,3(bcd)

    popmm   4,11
    xret
    .endasmfunc
    .end
