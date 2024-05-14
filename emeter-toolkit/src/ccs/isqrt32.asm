;******************************************************************************
;  isqrt32.asm (CCS version) - 
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
    .asg    R12,h_ls
    .asg    R13,h_ms

; Temporary variables
    .asg    R8,x_ls
    .asg    R9,x_ms
    .asg    R10,y_ls
    .asg    R11,y_ms
    .asg    R12,result_ls
    .asg    R13,result_ms
    .asg    R14,i

     .if $DEFINED(__LARGE_CODE_MODEL__) | $DEFINED(__LARGE_DATA_MODEL__)
STACK_USED .set 12
     .else
STACK_USED .set 6
     .endif

;uint32_t isqrt32(uint32_t h);
    .global isqrt32
    .text
    .align  2
isqrt32:    .asmfunc stack_usage(STACK_USED)
    pushmm  4,11
    mov.w   #0,x_ls
    mov.w   #0,x_ms
    mov.w   #0,y_ls
    mov.w   #0,y_ms
    mov.w   #32,i
isqrt32_1
    setc
    rlc.w   x_ls
    rlc.w   x_ms
    sub.w   x_ls,y_ls
    subc.w  x_ms,y_ms
    jhs     isqrt32_2
    add.w   x_ls,y_ls
    addc.w  x_ms,y_ms
    sub.w   #2,x_ls
isqrt32_2
    inc.w   x_ls
    rla.w   h_ls
    rlc.w   h_ms
    rlc.w   y_ls
    rlc.w   y_ms
    rla.w   h_ls
    rlc.w   h_ms
    rlc.w   y_ls
    rlc.w   y_ms
    dec.w   i
    jne     isqrt32_1
    mov.w   x_ls,result_ls
    mov.w   x_ms,result_ms
    popmm   4,11
    xret
    .endasmfunc
    .end
