#*******************************************************************************
#  Makefile -
#
#  Copyright (C) 2011 Texas Instruments Incorporated - http://www.ti.com/ 
# 
#  Redistribution and use in source and binary forms, with or without 
#  modification, are permitted provided that the following conditions 
#  are met:
#
#    Redistributions of source code must retain the above copyright 
#    notice, this list of conditions and the following disclaimer.
#
#    Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the 
#    documentation and/or other materials provided with the   
#    distribution.
#
#    Neither the name of Texas Instruments Incorporated nor the names of
#    its contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
#  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
#  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
#  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
#  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
#  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
#  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
#  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
#  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
#  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
#  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
#******************************************************************************

LIBNAME=libemeter-toolkit.a

CC=gcc
AR=ar
CFLAGS=-c -O2 -I. -I../include

objects=bin2bcd16.o \
        bin2bcd32.o \
        bin2bcd64.o \
        dc_filter16.o \
        dc_filter24.o \
        dds.o \
        div48.o \
        div_sh48.o \
        imul16.o \
        isqrt16.o \
        isqrt32.o \
        isqrt32i.o \
        isqrt64.o \
        isqrt64i.o \
        mac48.o \
        mac64_16_24.o \
        mul48_32_16.o \
        mul48u_32_16.o \
        q1_15_mul.o \
        q1_15_mulr.o \
        shift48.o \
        sqac48_16.o \
        sqac64_24.o
	
all: $(LIBNAME)

clean:
	rm -f *.obj *.asm *.map *.lst *.tmp *.exe *.o *.a *.hex *.qer *.asr *~

dist:
	rm -f *.obj *.asm *.map *.lst *.tmp *.exe *.o *.hex *.qer *.asr *~
	cd .. ; tar cvfz emeter-toolkit/emeter-toolkit.tgz emeter-toolkit

hostlib: $(LIBNAME)

$(LIBNAME): ${objects}
	$(AR) -r $(LIBNAME) ${objects}

$(objects): %.o: %.c
	$(CC) -c $(CFLAGS) $< -o $@

#automatic collection of dependencies in the source files.
#it's only updated the first time, after that it must be done maually
#with "make depend"
#the dependecies are included from a separate file:
-include dependencies.in
#target to update the file, it's removed first
depend: rmdepend dependencies.in
#remove the file
rmdepend:
	rm -f dependencies.in
#build the file that contains the dependencies. no deps in this rule.
#if there were deps it would be rebuilt every chnage, which is unneded:
dependencies.in:
	$(CC) -MM ${CFLAGS} $(patsubst %.o,%.c,$(objects)) >$@
