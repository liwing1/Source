;******************************************************************************
;  q1_15_mulr.asm (CCS version) - 
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
	.asg	R12,x
	.asg	R13,y

     .if $DEFINED(__LARGE_CODE_MODEL__) | $DEFINED(__LARGE_DATA_MODEL__)
STACK_USED .set 6
     .else
STACK_USED .set 3
     .endif

;Q1.15 style 16x16=>16 multiply with half bit rounding of the result.
; int16_t Q1_15_mulr(int16_t x, int16_t y);
    .global q1_15_mulr
    .text
    .align  2
q1_15_mulr: .asmfunc stack_usage(STACK_USED)
 .if ($defined(__MSP430_HAS_MPY__)  |  $defined(__MSP430_HAS_MPY32__))  &  !$defined(__TOOLKIT_USE_SOFT_MPY__)
	push    SR
	dint
	nop
	mov     x,&MPYS
	mov     y,&OP2
	mov     &RESHI,R13
	mov     &RESLO,x
	pop     SR
    ;Half bit round at the Q1.15 level
	add     #4000h,x
	addc    #0,R13
    ;Shift to Q1.15 format (i.e. the top 16 bits are returned)
	rla     x
	rlc     R13
	mov     R13,x
    xret
 .else
	pushmm  1,10
	pushmm  1,15
	mov     x,R15
	clr     R13
	mov     R13,x

	mov     R13,R10
	tst     R15
	jge     booth_2
	mov     #-1,R10
	jmp     booth_2

booth_6
	add     R15,x
	addc    R10,R13
booth_1
	rla     R15
	rlc     R10
booth_2
	rra     y
	jc      booth_5
	jne     booth_1
	jmp     booth_4

booth_5
	sub     R15,x
	subc    R10,R13
booth_3
	rla     R15
	rlc     R10
	rra     y
	jnc     booth_6
	cmp     #0xFFFF,y
	jne     booth_3

booth_4
    ;Half bit round at the Q1.15 level
	add     #4000h,x
	addc    #0,R13
    ;Shift to Q1.15 format (i.e. the top 16 bits are returned)
	rla     x
	rlc     R13
	mov     R13,x
	popmm   1,15
	popmm   1,10
    xret
 .endif
    .endasmfunc
    .end
