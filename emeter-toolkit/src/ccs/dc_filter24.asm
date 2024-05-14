;******************************************************************************
;  dc_filter24.asm (CCS version) - DC estimation and removal for 24 bit signals
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
    .asg    R12,p
    .asg    R13,x_loo
    .asg    R14,x_hi

    ; Temporary variables
    .asg    R10,x_lo
    .asg    R15,x_ext
    .asg    R11,z

    .asg    R15,tmp

    ; Result
    .asg    R12,result_lo
    .asg    R13,result_hi

     .if $DEFINED(__LARGE_CODE_MODEL__) | $DEFINED(__LARGE_DATA_MODEL__)
STACK_USED .set 8
     .else
STACK_USED .set 4
     .endif

; A routine to filter away the DC content from an AC mains waveform
; signal. It does this my using a heavily damped single pole leaky integrator,
; with noise shaping, to estimate the DC level. The current DC level is then
; subtracted from the signal.
; The actual calculations are:
;   z += (((x << 16) - z) >> 14);
;   return  x - (z >> 16);
; where z is a 48 bit value, stored as a 3 element array of 16 bit integers.

;int32_t dc_filter24(int16_t p[3], int32_t x);
    .global dc_filter24
	.text
    .align  2
dc_filter24: .asmfunc stack_usage(STACK_USED)
    pushmm  2,11
    ; Move things out of the way, so the result registers are safe
    mov.w   p,z
    mov.w   x_loo,x_lo
    ;
    mov.w   x_lo,result_lo
    mov.w   x_hi,result_hi
    sub.w   2(z),x_lo
    subc.w  4(z),x_hi
    ; Shift, so our final effective shift is 14 bits.
    rla.w   x_lo
    rlc.w   x_hi
    subc.w  x_ext,x_ext
    inv.w   x_ext
    rla.w   x_lo
    rlc.w   x_hi
    rlc.w   x_ext
    ;
    add.w   x_lo,0(z)
    addc.w  2(z),x_hi
    addc.w  4(z),x_ext
    ; Store the updated *z
    mov.w   x_hi,2(z)
    mov.w   x_ext,4(z)
    ;
    sub.w   x_hi,result_lo
    subc.w  x_ext,result_hi
    ;
    popmm   2,11
    xret
    .endasmfunc

;void dc_filter24_init(int16_t p[3], int16_t x);
    .global dc_filter24_init
	.text
    .align  2
dc_filter24_init: .asmfunc stack_usage(STACK_USED)
    ; Put the 16 bit initial estimate where the high 16 bits of the 24 bit signal will sit when the filter is running
    mov.w   #0,0(p)
    mov.w   #0,2(p)
    mov.w   #0,4(p)
    mov.b   x_loo,3(p)
    swpb    x_loo
    sxt     x_loo
    mov.w   x_loo,4(p)
    xret
    .endasmfunc

;int32_t dc_filter24_estimate(int16_t p[3]);
    .global dc_filter24_estimate
	.text
    .align  2
dc_filter24_estimate: .asmfunc stack_usage(STACK_USED)
    mov.w   p,tmp
    mov.b   4(tmp),result_hi
    swpb    result_hi
    mov.b   3(tmp),result_lo
    bis.w   result_lo,result_hi
    mov.b   2(tmp),result_lo
    swpb    result_lo
    mov.b   1(tmp),tmp
    bis.w   tmp,result_lo
    xret
    .endasmfunc
    .end
