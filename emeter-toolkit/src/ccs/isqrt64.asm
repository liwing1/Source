;******************************************************************************
;  isqrt64.asm (CCS version) - 
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
    .asg    R12,h_0
    .asg    R13,h_1
    .asg    R14,h_2
    .asg    R15,h_3

; Temporary variables
    .asg    R8,x_0
    .asg    R9,x_1
    .asg    R10,x_2
    .asg    R11,x_3
    .asg    R4,y_0
    .asg    R5,y_1
    .asg    R6,y_2
    .asg    R7,y_3

; Result
    .asg    R12,result_0
    .asg    R13,result_1
    .asg    R14,result_2
    .asg    R15,result_3

     .if $DEFINED(__LARGE_CODE_MODEL__) | $DEFINED(__LARGE_DATA_MODEL__)
STACK_USED .set 20
     .else
STACK_USED .set 10
     .endif

;uint64_t isqrt64(uint64_t h)
    .global isqrt64
    .text
    .align  2
isqrt64:    .asmfunc stack_usage(STACK_USED)
    pushmm  8,11

    ; Use a variable on the stack as the loop counter
    mov.w   #32,R11
    push    R11

    mov.w   #0,x_0
    mov.w   #0,x_1
    mov.w   #0,x_2
    mov.w   #0,x_3
    mov.w   #0,y_0
    mov.w   #0,y_1
    mov.w   #0,y_2
    mov.w   #0,y_3
isqrt64_1
    setc
    rlc.w   x_0
    rlc.w   x_1
    rlc.w   x_2
    rlc.w   x_3
    sub.w   x_0,y_0
    subc.w  x_1,y_1
    subc.w  x_2,y_2
    subc.w  x_3,y_3
    jhs     isqrt64_2
    add.w   x_0,y_0
    addc.w  x_1,y_1
    addc.w  x_2,y_2
    addc.w  x_3,y_3
    sub.w   #2,x_0
isqrt64_2
    add.w   #1,x_0
    ;
    rla.w   h_0
    rlc.w   h_1
    rlc.w   h_2
    rlc.w   h_3
    rlc.w   y_0
    rlc.w   y_1
    rlc.w   y_2
    rlc.w   y_3
    ;
    rla.w   h_0
    rlc.w   h_1
    rlc.w   h_2
    rlc.w   h_3
    rlc.w   y_0
    rlc.w   y_1
    rlc.w   y_2
    rlc.w   y_3
    ;
    dec     0(sp)
    jne     isqrt64_1

    mov.w   #32,0(sp)
isqrt64_3
    setc
    rlc.w   x_0
    rlc.w   x_1
    rlc.w   x_2
    rlc.w   x_3
    sub.w   x_0,y_0
    subc.w  x_1,y_1
    subc.w  x_2,y_2
    subc.w  x_3,y_3
    jhs     isqrt64_4
    add.w   x_0,y_0
    addc.w  x_1,y_1
    addc.w  x_2,y_2
    addc.w  x_3,y_3
    sub.w   #2,x_0
isqrt64_4
    inc.w   x_0
    ;
    rla.w   y_0
    rlc.w   y_1
    rlc.w   y_2
    rlc.w   y_3
    ;
    rla.w   y_0
    rlc.w   y_1
    rlc.w   y_2
    rlc.w   y_3
    ;
    dec     0(sp)
    jne     isqrt64_3

    mov.w   x_0,result_0
    mov.w   x_1,result_1
    mov.w   x_2,result_2
    mov.w   x_3,result_3

    pop     R11
    popmm   8,11
    xret
    .endasmfunc
    .end
