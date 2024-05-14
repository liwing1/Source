;******************************************************************************
;  dc_filter16.asm (CCS version) - DC estimation and removal for 16 bit signals
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
    .asg    R13,x

    ; Temporary variables
    .asg    R14,tmp_lo
    .asg    R15,tmp_hi

    .asg    R14,tmp

    ; Result
    .asg    R12,result

    .asg    R12,result_lo
    .asg    R13,result_hi

    .if $DEFINED(__LARGE_CODE_MODEL__) | $DEFINED(__LARGE_DATA_MODEL__)
STACK_USED .set 6
     .else
STACK_USED .set 3
     .endif

; A routine to filter away the DC content from an AC mains waveform
; signal. It does this my using a heavily damped integrator to estimate
; the DC level. The current DC level is then subtracted from the signal.

; One would like to estimate DC by something like:
;   z += ((((int32_t) x << 15) - z) >> 14);
;   return  x - (z >> 15);
; but this:
;   z += ((((int32_t) x << 16) - z) >> 14);
;   return  x - (z >> 16);
; is a bit faster, and the shift by 16 will never
; cause an overflow in this application. However,
; remember this is not a generic DC filter!

;int16_t dc_filter16(int32_t *p, int16_t x);
    .global dc_filter16
    .text
    .align  2
dc_filter16: .asmfunc stack_usage(STACK_USED)
    pushmm  1,11
    mov.w   0(p),tmp_lo      ; subtract (sample << 16) from filter_state 
    mov.w   2(p),tmp_hi
    sub.w   x,tmp_hi
    ;
    rla.w   tmp_lo           ; >> 14
    rlc.w   tmp_hi
    subc.w  R11,R11
    inv.w   R11
    rla.w   tmp_lo
    rlc.w   tmp_hi
    rlc.w   R11
    ;
    sub.w   tmp_hi,0(p)     ; + filter_state
    subc.w  R11,2(p)
    sub.w   2(p),x          ; sample - filter_state
    mov.w   x,result
    popmm   1,11
    xret
    .endasmfunc

;void dc_filter16_init(int32_t *p, int16_t x);
    .global dc_filter16_init
    .text
    .align  2
dc_filter16_init: .asmfunc stack_usage(STACK_USED)
    ; Put the 16 bit initial estimate where the 16 bit signal will sit when the filter is running
    mov.w   #0,0(p)
    mov.w   x,2(p)
    xret
    .endasmfunc

;int32_t dc_filter16_estimate(int32_t *p, int16_t x);
    .global dc_filter16_estimate
    .text
    .align  2
dc_filter16_estimate: .asmfunc stack_usage(STACK_USED)
    ; Shuffle the filter state so we return the estimate with 8 fractional bits
    mov.w   p,tmp
    mov.w   2(tmp),result_lo
    swpb    result_lo
    mov.b   result_lo,result_hi
    sxt     result_hi
    and.w   #0xFF00,result_lo
    mov.b   1(tmp),tmp
    bis.w   tmp,result_lo
    xret
    .endasmfunc
    .end
