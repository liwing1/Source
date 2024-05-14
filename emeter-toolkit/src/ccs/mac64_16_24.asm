;******************************************************************************
;  mac64_16_24.asm (CCS version) - 
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
	.asg	R12,z
	.asg	R13,x
	.asg	R14,y_lo
	.asg	R15,y_hi
	
	; Temporary variables
	.asg	R11,tmp

     .if $DEFINED(__LARGE_CODE_MODEL__) | $DEFINED(__LARGE_DATA_MODEL__)
STACK_USED .set 4
     .else
STACK_USED .set 2
     .endif

;void mac64_16_24(int64_t *z, int16_t x, int32_t y)
    .global mac64_16_24
    .text
    .align  2
mac64_16_24: .asmfunc stack_usage(STACK_USED)
    ; NB: This is not protected against interrupts, so only use it in an interrupt routine
 .if $defined(__MSP430_HAS_MPY32__)  &  !$defined(__TOOLKIT_USE_SOFT_MPY__)
    mov.w   y_lo,&MPYS32L
    mov.b   y_hi,&MPYS32H_B
    mov.w   x,&OP2
    add.w   &RES0,0(z)
    addc.w  &RES1,2(z)
    addc.w  &RES2,4(z)
    addc.w  &RES3,6(z)
 .elseif $defined(__MSP430_HAS_MPY__)  &  !$defined(__TOOLKIT_USE_SOFT_MPY__)
    ; We need to do the multiply in 2 16x16=>32 chunks. It isn't easy to
    ; do this with signed multiplies, so we take care of the signs first
    ; and then do 2 unsigned multiplies
    pushmm  1,11
    clr.w   tmp
    tst.w   y_hi
    jge     mac64_16_24_1
    ; The 24 bit y is negative
    xor.w   #1,tmp
    inv.w   y_hi
    inv.w   y_lo
    add.w   #1,y_lo
    addc.w  #0,y_hi
mac64_16_24_1
    tst.w   x
    jge     mac64_16_24_2
    ; The 16 bit x is negative
    xor.w   #1,tmp
    inv.w   x
    add.w   #1,x
mac64_16_24_2
    mov.w   x,&MPY
    tst.w   tmp
    jeq     mac64_16_24_3
    ; The answer to the real multiply will be negative, so subtract the unsigned one from the sum
    mov.w   y_hi,&OP2
    mov.w   #RESLO,tmp
    sub.w   @tmp+,2(z)
    subc.w  @tmp,4(z)
    subc.w  #0,6(z)
    ;
    mov.w   y_lo,&OP2
    sub.w   #2,tmp
    sub.w   @tmp+,0(z)
    subc.w  @tmp,2(z)
    subc.w  #0,4(z)
    subc.w  #0,6(z)
    ;
    pop     tmp
    xret
mac64_16_24_3
    ; The answer to the real multiply will be positive, so add the unsigned one to the sum
    mov.w   y_hi,&OP2
    mov.w   #RESLO,tmp
    add.w   @tmp+,2(z)
    addc.w  @tmp,4(z)
    addc.w  #0,6(z)
    ;
    mov.w   y_lo,&OP2
    sub.w   #2,tmp
    add.w   @tmp+,0(z)
    addc.w  @tmp,2(z)
    addc.w  #0,4(z)
    addc.w  #0,6(z)
    ;
    popmm   1,11
 .else
    ; TODO: software multiply version
 .endif
    xret
    .endasmfunc
    .end
