;******************************************************************************
;  isqrt16.s43 (CCS version) - 16 bit square root
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
    .asg    R12,h

; Temporary variables
	.asg	R13,x
	.asg	R14,y
	.asg	R12,result
	.asg	R15,i

     .if $DEFINED(__LARGE_CODE_MODEL__) | $DEFINED(__LARGE_DATA_MODEL__)
STACK_USED .set 4
     .else
STACK_USED .set 2
     .endif

;uint16_t isqrt16(uint16_t h);
    .global isqrt16
    .text
    .align  2
isqrt16:    .asmfunc stack_usage(STACK_USED)
    ;The answer is calculated as a 16 bit value, where the last
    ;8 bits are fractional.
    mov.w   #0,x
    mov.w   #0,y
    mov.w   #16,i
isqrt16_1
    setc
    rlc.w   x
    sub.w   x,y
    jhs     isqrt16_2
    add.w   x,y
    sub.w   #2,x
isqrt16_2
    inc.w   x
    rla.w   h
    rlc.w   y
    rla.w   h
    rlc.w   y
    dec.w   i
    jne     isqrt16_1
    mov.w   x,result
    xret
    .endasmfunc
    .end
