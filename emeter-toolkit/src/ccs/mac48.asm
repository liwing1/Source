;******************************************************************************
;  mac48.asm (CCS version) - 
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

	.asg	R12,x
	.asg	R13,y
	.asg	R14,z

     .if $DEFINED(__LARGE_CODE_MODEL__) | $DEFINED(__LARGE_DATA_MODEL__)
STACK_USED .set 4
     .else
STACK_USED .set 2
     .endif

; void mac48(register int16_t x[3], register int16_t y, register int16_t z)
    .global mac48_16
	.text
    .align  2
mac48_16:   .asmfunc stack_usage(STACK_USED)
 .if $defined(__MSP430_HAS_MPY__)  |  $defined(__MSP430_HAS_MPY32__)
    ; NB: This is not protected against interrupts, so only use it in an interrupt routine
    mov     y,&MPYS
    mov     z,&OP2
    nop
    add.w   &RESLO,0(x)
    addc.w  &RESHI,2(x)
    addc.w  &SUMEXT,4(x)
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
	popmm   1,15
    mov.w   R13,R10
    inv.w   R10
    rla.w   R10
    subc.w  R10,R10
    add.w   y,0(z)
    addc.w  R13,2(z)
    addc.w  R10,4(z)
	popmm   1,10
 .endif
    xret
    .endasmfunc
    .end
