;******************************************************************************
;  if_macros.asm (CCS version) - Macros to help interface assembly language
;                                with C
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

	; These macros and assignments abstract the large code/data
	; models out of the code so that the code can be written once
	; to handle all code/data models.

	; xadd and xmov are only ever used on registers.  On MSPX, for
	; register-only instructions, it is always safe to use the
	; A-size instruction, even in large code or large data model.
	.if $defined(.MSP430X)
	    .asg add.a,xadd
	    .asg mov.a,xmov
	.else
	    .asg add.w,xadd
	    .asg mov.w,xmov
	.endif

	; Certain instructions dealing with code addresses must have
	; exactly the right size.  For instance, CALL/RET push/pop a
	; different number of words to store RETA than CALLA/RETA do.
    .if $defined(__LARGE_CODE_MODEL__)
        ; The data size handled by each of these must match!
        .asg calla,xcall
        .asg reta,xret
        .asg cmpx,xcmp
        .asg bra,xbr
    .else
        ; The data size handled by each of these must match!
        .asg call,xcall
        .asg ret,xret
        .asg cmp,xcmp
        .asg br,xbr
    .endif

	; When storing/loading code addresses (RETA) to/from the
	; stack, the size must be exactly right.  We keep these moves
	; distinct from the moves which can be A-sized even in small
	; data and code models.
	;
	; We can't use xmovreta for moving data addresses between
	; registers, because in small code model xmovreta must be mov.w,
	; but in large data model a data address move must be mov.a.
    .if $defined(__LARGE_CODE_MODEL__)
        .asg mov.a,xmovreta
	.else
        .asg mov.w,xmovreta
    .endif

    ; Produce a pushm or multiple push instructions, as appropriate for the
    ; current core and model. Also, make the pushes of the right data width.
pushmm  .macro n, start
        .if $defined(.MSP430X)
            .if $defined(__LARGE_CODE_MODEL__) | $defined(__LARGE_DATA_MODEL__)
                pushm.a #n,R:start:
            .else
                pushm.w #n,R:start:
            .endif
        .else
            .loop n
                push.w R:start:
                .eval start-1, start
            .endloop
        .endif
    .endm

    ; Produce a popm or multiple pop instructions, as appropriate for the
    ; current core and model. Also, make the pops of the right data width.
popmm   .macro n, start
        .if $defined(.MSP430X)
            .if $defined(__LARGE_CODE_MODEL__) | $defined(__LARGE_DATA_MODEL__)
                popm.a #n,R:start:
            .else
                popm.w #n,R:start:
            .endif
        .else
            .eval start-n+1, start
            .loop n
                pop.w R:start:
                .eval start+1, start
            .endloop
        .endif
    .endm

	;
	; REG_SZ is the size of a saved register
	;
    .if $defined(__LARGE_CODE_MODEL__) | $defined(__LARGE_DATA_MODEL__)
        .asg 4,REG_SZ
    .else
        .asg 2,REG_SZ
    .endif

