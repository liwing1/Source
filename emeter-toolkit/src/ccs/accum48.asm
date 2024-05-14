;******************************************************************************
;  accum48.asm (CCS version) - 
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
    .asg    R13, y_ls
    .asg    R14, y_ms
    .asg    R13, y

     .if $DEFINED(__LARGE_CODE_MODEL__) | $DEFINED(__LARGE_DATA_MODEL__)
STACK_USED .set 6
     .else
STACK_USED .set 3
     .endif

;void accum48(int16_t x[3], int32_t y);
    .global accum48
    .text
    .align  2
accum48:    .asmfunc stack_usage(STACK_USED)
    pushmm  1,10
    mov.w   y_ms,R10
    inv.w   R10
    rla.w   R10
    subc.w  R10,R10
    add.w   y_ls,0(x)
    addc.w  y_ms,2(x)
    addc.w  R10,4(x)
    popmm   1,10
    xret
    .endasmfunc

;void accum48_48(int16_t x[3], int16_t y[3]);
    .global accum48_48
    .text
    .align  2
accum48_48:     .asmfunc stack_usage(STACK_USED)
    add.w   0(y),0(x)
    addc.w  2(y),2(x)
    addc.w  4(y),4(x)
    xret
    .endasmfunc

;void decum48_48(int16_t x[3], int16_t y[3]);
    .global decum48_48
    .text
    .align  2
decum48_48:     .asmfunc stack_usage(STACK_USED)
    sub.w   0(y),0(x)
    subc.w  2(y),2(x)
    subc.w  4(y),4(x)
    xret
    .endasmfunc
    .end
