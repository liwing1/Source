;******************************************************************************
;  mul48_32_16.asm (CCS version) - 48 bit 16x32=>top 32 bits multiply
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
    .asg	R12, x_ls
    .asg	R13, x_ms
    .asg	R14, y

     .if $DEFINED(__LARGE_CODE_MODEL__) | $DEFINED(__LARGE_DATA_MODEL__)
STACK_USED .set 6
     .else
STACK_USED .set 3
     .endif

;mul48 is a 32x16=>top 32 bits multiply
;int32_t mul48_32_16(int32_t x, uint16_t y);
    .global mul48_32_16
	.text
    .align  2
mul48_32_16: .asmfunc stack_usage(STACK_USED)
 .if $defined(__MSP430_HAS_MPY32__)  &  !$defined(__TOOLKIT_USE_SOFT_MPY__)
    push    SR
    dint
    nop
    mov.w   x_ls,&MPYS32L
    mov.w   x_ms,&MPYS32H
    mov.w   y,&OP2L
    mov.w   #0,&OP2H
    nop
    nop
    mov.w   &RES1,x_ls
    mov.w   &RES2,x_ms
    pop     SR
 .elseif $defined(__MSP430_HAS_MPY__)  &  !$defined(__TOOLKIT_USE_SOFT_MPY__)
    push.w  SR
    dint
    nop
    tst.w   x_ms
    jge     mul48_32_16_1
    ; The 32 bit x is negative
    inv.w   x_ls
    inv.w   x_ms
    add.w   #1,x_ls
    addc.w  #0,x_ms
    mov.w   y,&MPY
    mov.w   x_ls,&OP2
    nop
    mov.w   &RESLO,y
    mov.w   &RESHI,x_ls
    mov.w   x_ms,&OP2
    mov.w   #0,x_ms
    add.w   &RESLO,x_ls
    addc.w  &RESHI,x_ms
    pop.w   SR
    ; Make the answer negative
    inv.w   y
    inv.w   x_ls
    inv.w   x_ms
    add.w   #1,y
    addc.w  #0,x_ls
    addc.w  #0,x_ms
    xret
mul48_32_16_1
    mov.w   y,&MPY
    mov.w   x_ls,&OP2
    nop
    nop
    mov.w   &RESHI,x_ls
    mov.w   x_ms,&OP2
    mov.w   #0,x_ms
    add.w   &RESLO,x_ls
    addc.w  &RESHI,x_ms
    pop.w   SR
 .endif
    xret
    .endasmfunc
    .end
