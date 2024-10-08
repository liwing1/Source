;******************************************************************************
;  shift48.asm (CCS version) - 
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
    .asg    R12, x
    .asg    R13, how_far

     .if $DEFINED(__LARGE_CODE_MODEL__) | $DEFINED(__LARGE_DATA_MODEL__)
STACK_USED .set 4
     .else
STACK_USED .set 2
     .endif

; void shift48(register int16_t x[3], int how_far);
    .global shift48
    .text
    .align  2
shift48:    .asmfunc stack_usage(STACK_USED)
    cmp     #0,how_far
    jeq     shift48_1
    jl      shift48_3
shift48_2
    rla.w   0(r12)
    rlc.w   2(r12) 
    rlc.w   4(r12) 
    dec     how_far
    jne     shift48_2
shift48_1
    xret
shift48_3
    rra.w   4(r12)
    rrc.w   2(r12) 
    rrc.w   0(r12) 
    inc     how_far
    jne     shift48_3
    xret
    .endasmfunc
    .end
