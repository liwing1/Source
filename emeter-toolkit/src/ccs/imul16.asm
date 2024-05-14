;******************************************************************************
;  imul16.asm (CCS version) - 16 bit signed multiply
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

;This routine implements a 16x16->32 2's complement multiplier. If a hardware
;multiplier is available it is used. If no hardware multiplier is available,
;Booth's algorithm is used to directly implement a signed multiplier in software.
;C does not normally express a 16x16->32 operation, and the speedup of achieving
;a 32-bit answer by a full 32x32 multiply is substantial.

    ; Parameters
    .asg    R12,x
    .asg    R13,y

    ; Result
    .asg    R12,ret_lo
    .asg    R13,ret_hi

     .if $DEFINED(__LARGE_CODE_MODEL__) | $DEFINED(__LARGE_DATA_MODEL__)
STACK_USED .set 6
     .else
STACK_USED .set 3
     .endif

;int32_t imul16(int16_t x, int16_t y)
    .global imul16
    .text
    .align  2
imul16:     .asmfunc stack_usage(STACK_USED)
 .if ($defined(__MSP430_HAS_MPY__)  |  $defined(__MSP430_HAS_MPY32__))  &  !$defined(__TOOLKIT_USE_SOFT_MPY__)
    push    SR
    dint
    nop
    mov.w   x,&MPYS
    mov.w   y,&OP2
    mov.w   &RESHI,ret_hi
    mov.w   &RESLO,ret_lo
    pop     SR
 .else
    pushmm  1,11
    mov.w   x,R15
    mov.w   y,R14
    clr.w   R13
    mov.w   R13,x

    mov.w   R13,R11
    tst.w   R15
    jge     booth_2
    mov.w   #-1,R11
    jmp     booth_2

booth_6
    add.w   R15,x
    addc.w  R11,R13
booth_1
    rla.w   R15
    rlc.w   R11
booth_2
    rra.w   R14
    jc      booth_5
    jne     booth_1
    jmp     booth_4

booth_5
    sub.w   R15,x
    subc.w  R11,R13
booth_3
    rla.w   R15
    rlc.w   R11
    rra.w   R14
    jnc     booth_6
    cmp.w   #0xFFFF,R14
    jne     booth_3

booth_4
    popmm   1,11
 .endif
    xret
    .endasmfunc
    .end
