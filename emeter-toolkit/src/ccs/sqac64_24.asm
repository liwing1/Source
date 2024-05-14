;******************************************************************************
;  sqac64_24.asm (CCS version) - 
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
    .asg    R12,z
    .asg    R14,x_lo
    .asg    R15,x_hi

    ; Temporary variables
    .asg    R13,tmp

     .if $DEFINED(__LARGE_CODE_MODEL__) | $DEFINED(__LARGE_DATA_MODEL__)
STACK_USED .set 4
     .else
STACK_USED .set 2
     .endif

; Square and accumulate x into z
;void sqac64_24(register int64_t *z, register int32_t x)
    .global sqac64_24
    .text
    .align  2
sqac64_24:  .asmfunc stack_usage(STACK_USED)
    ; NB: This is not protected against interrupts, so only use it in an interrupt routine
 .if $defined(__MSP430_HAS_MPY32__)  &  !$defined(__TOOLKIT_USE_SOFT_MPY__)
    mov.w   #RES0,tmp
    mov.w   x_lo,&MPYS32L
    mov.b   x_hi,&MPYS32H_B
    mov.w   x_lo,&OP2L
    mov.b   x_hi,&OP2H_B
    add.w   @tmp+,0(z)
    addc.w  @tmp+,2(z)
    addc.w  @tmp+,4(z)
    addc.w  @tmp,6(z)
 .elseif $defined(__MSP430_HAS_MPY__)  &  !$defined(__TOOLKIT_USE_SOFT_MPY__)
    ; We need to do the multiply in 4 chunks. It isn't easy to
    ; do this with signed multiplies, so we take care of the signs first
    ; and then do 2 unsigned multiplies
    tst.w   x_hi
    jge     sqac64_24_1
    ; The 24 bit x is negative
    inv.w   x_hi
    inv.w   x_lo
    add.w   #1,x_lo
    addc.w  #0,x_hi
sqac64_24_1
    mov.w   x_hi,&MPY
    mov.w   x_hi,&OP2
    mov.w   #RESLO,tmp
    add.w   @tmp+,4(z)
    addc.w  @tmp,6(z)
    ;
    mov.w   x_lo,&MPY
    mov.w   x_lo,&OP2
    sub.w   #2,tmp
    add.w   @tmp+,0(z)
    addc.w  @tmp,2(z)
    addc.w  #0,4(z)
    addc.w  #0,6(z)
    ;
    ; Note: It is this use of a shift which stops this routine being a full 32bit operation
    rla.w   x_hi
    mov.w   x_hi,&OP2
    sub.w   #2,tmp
    add.w   @tmp+,2(z)
    addc.w  @tmp,4(z)
    addc.w  #0,6(z)
    ;
 .else
    ; TODO: software multiply version
 .endif
    xret
    .endasmfunc
    .end
