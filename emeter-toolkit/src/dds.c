/*******************************************************************************
 *  dds.c - Simple direct digital synthesis routines.
 *
 *  Copyright (C) 2011 Texas Instruments Incorporated - http://www.ti.com/ 
 * 
 *  Redistribution and use in source and binary forms, with or without 
 *  modification, are permitted provided that the following conditions 
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright 
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the 
 *    documentation and/or other materials provided with the   
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************/

#include <inttypes.h>
#if defined(__MSP430__)
#include <msp430.h>
#endif
#include "emeter-toolkit.h"

#define SLENK       8
#define DDS_STEPS   (1 << SLENK)
#define DDS_SHIFT   (32 - 2 - SLENK)

static const int16_t sine_table[DDS_STEPS + 1] =
{
         0,   201,   402,   603,   804,  1005,  1206,  1407,
      1608,  1809,  2009,  2210,  2410,  2611,  2811,  3012,
      3212,  3412,  3612,  3811,  4011,  4210,  4410,  4609,
      4808,  5007,  5205,  5404,  5602,  5800,  5998,  6195,
      6393,  6590,  6786,  6983,  7179,  7375,  7571,  7767,
      7962,  8157,  8351,  8545,  8739,  8933,  9126,  9319,
      9512,  9704,  9896, 10087, 10278, 10469, 10659, 10849,
     11039, 11228, 11417, 11605, 11793, 11980, 12167, 12353,
     12539, 12725, 12910, 13094, 13279, 13462, 13645, 13828,
     14010, 14191, 14372, 14553, 14732, 14912, 15090, 15269,
     15446, 15623, 15800, 15976, 16151, 16325, 16499, 16673,
     16846, 17018, 17189, 17360, 17530, 17700, 17869, 18037,
     18204, 18371, 18537, 18703, 18868, 19032, 19195, 19357,
     19519, 19680, 19841, 20000, 20159, 20317, 20475, 20631,
     20787, 20942, 21096, 21250, 21403, 21554, 21705, 21856,
     22005, 22154, 22301, 22448, 22594, 22739, 22884, 23027,
     23170, 23311, 23452, 23592, 23731, 23870, 24007, 24143,
     24279, 24413, 24547, 24680, 24811, 24942, 25072, 25201,
     25329, 25456, 25582, 25708, 25832, 25955, 26077, 26198,
     26319, 26438, 26556, 26674, 26790, 26905, 27019, 27133,
     27245, 27356, 27466, 27575, 27683, 27790, 27896, 28001,
     28105, 28208, 28310, 28411, 28510, 28609, 28706, 28803,
     28898, 28992, 29085, 29177, 29268, 29358, 29447, 29534,
     29621, 29706, 29791, 29874, 29956, 30037, 30117, 30195,
     30273, 30349, 30424, 30498, 30571, 30643, 30714, 30783,
     30852, 30919, 30985, 31050, 31113, 31176, 31237, 31297,
     31356, 31414, 31470, 31526, 31580, 31633, 31685, 31736,
     31785, 31833, 31880, 31926, 31971, 32014, 32057, 32098,
     32137, 32176, 32213, 32250, 32285, 32318, 32351, 32382,
     32412, 32441, 32469, 32495, 32521, 32545, 32567, 32589,
     32609, 32628, 32646, 32663, 32678, 32692, 32705, 32717,
     32728, 32737, 32745, 32752, 32757, 32761, 32765, 32766,
     32767
};

int16_t dds_lookup(uint32_t phase)
{
    int16_t amp;
    uint32_t step;

    phase >>= DDS_SHIFT;
    step = phase & (DDS_STEPS - 1);
    if ((phase & DDS_STEPS))
        step = DDS_STEPS - step;
    amp = sine_table[step];
    if ((phase & (2*DDS_STEPS)))
    	amp = -amp;
    return amp;
}

int16_t dds_interpolated_lookup(uint32_t phase)
{
    int16_t amp;
    uint16_t step;
    uint16_t stepx;
    uint16_t frac;

    /* An interpolated 4 quadrant lookup of the sine of the 32 bit phase angle supplied */
    frac = (phase >> (DDS_SHIFT - 8)) & 0xFF;
    phase >>= DDS_SHIFT;
    step = phase & (DDS_STEPS - 1);
    if ((phase & DDS_STEPS))
    {
        step = DDS_STEPS - step;
        stepx = step - 1;
    }
    else
    {
        stepx = step + 1;
    }
    amp = sine_table[step];
    amp += (((sine_table[stepx] - amp)*frac) >> 8);
    if ((phase & (2*DDS_STEPS)))
    	amp = -amp;
    return amp;
}

int16_t dds(uint32_t *phase_acc, int32_t phase_rate)
{
    int16_t amp;

    amp = dds_lookup(*phase_acc);
    *phase_acc += phase_rate;
    return amp;
}
