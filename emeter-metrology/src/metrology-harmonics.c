/*******************************************************************************
 *  emeter-harmonics.c -
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
#if !defined(__MSP430__)
#include <stdio.h>
#include <fcntl.h>
#include <stdlib.h>
#include <unistd.h>
#endif
#if defined(__GNUC__)
#include <signal.h>
#endif
#if defined(__MSP430__)
#include <msp430.h>
#endif
#include <emeter-toolkit.h>

#include "emeter-metrology.h"
#include "metrology-structs.h"

#if defined(HARMONICS_SUPPORT)
#if SAMPLES_PER_10_SECONDS == 32768
#if MAINS_NOMINAL_FREQUENCY == 50
const int16_t harm_factors[][4] =
{
    {   138,   2040,   1972,   1839},
    {   138,   2039,   1972,   1838},
    {   137,   2039,   1971,   1838},
    {   137,   2039,   1971,   1837},
    {   137,   2039,   1971,   1836},
    {   137,   2039,   1970,   1835},
    {   136,   2039,   1970,   1834},
    {   136,   2039,   1970,   1833},
    {   136,   2039,   1969,   1832},
    {   135,   2039,   1969,   1831},
    {   135,   2039,   1969,   1831},
    {   135,   2039,   1968,   1830},
    {   135,   2039,   1968,   1829},
    {   134,   2039,   1968,   1828},
    {   134,   2039,   1968,   1827},
    {   134,   2039,   1967,   1826},
    {   133,   2039,   1967,   1825},
    {   133,   2039,   1967,   1824},
    {   133,   2039,   1966,   1823},
    {   133,   2039,   1966,   1823},
    {   132,   2039,   1966,   1822},
    {   132,   2039,   1965,   1821},
    {   132,   2039,   1965,   1820},
    {   132,   2039,   1965,   1819},
    {   131,   2039,   1964,   1818},
    {   131,   2039,   1964,   1817},
    {   131,   2039,   1964,   1816},
    {   131,   2039,   1963,   1815},
    {   130,   2038,   1963,   1814},
    {   130,   2038,   1963,   1814},
    {   130,   2038,   1962,   1813},
    {   130,   2038,   1962,   1812},
    {   129,   2038,   1962,   1811},
    {   129,   2038,   1961,   1810},
    {   129,   2038,   1961,   1809},
    {   129,   2038,   1960,   1808},
    {   128,   2038,   1960,   1807},
    {   128,   2038,   1960,   1806},
    {   128,   2038,   1959,   1805},
    {   128,   2038,   1959,   1804},
    {   127,   2038,   1959,   1803},
    {   127,   2038,   1958,   1802},
    {   127,   2038,   1958,   1802},
    {   127,   2038,   1958,   1801},
    {   126,   2038,   1957,   1800},
    {   126,   2038,   1957,   1799},
    {   126,   2038,   1957,   1798},
    {   126,   2038,   1956,   1797},
    {   125,   2038,   1956,   1796},
    {   125,   2038,   1956,   1795},
    {   125,   2038,   1955,   1794},
};
#else
/* Assume 60Hz */
const int16_t harm_factors[][4] =
{
    {   114,   2036,   1937,   1745},
    {   114,   2036,   1937,   1744},
    {   114,   2035,   1936,   1743},
    {   113,   2035,   1936,   1742},
    {   113,   2035,   1935,   1740},
    {   113,   2035,   1935,   1739},
    {   113,   2035,   1935,   1738},
    {   113,   2035,   1934,   1737},
    {   112,   2035,   1934,   1736},
    {   112,   2035,   1934,   1735},
    {   112,   2035,   1933,   1734},
    {   112,   2035,   1933,   1733},
    {   112,   2035,   1932,   1732},
    {   111,   2035,   1932,   1731},
    {   111,   2035,   1932,   1730},
    {   111,   2035,   1931,   1729},
    {   111,   2035,   1931,   1728},
    {   111,   2035,   1930,   1727},
    {   111,   2035,   1930,   1726},
    {   110,   2035,   1930,   1725},
    {   110,   2035,   1929,   1724},
    {   110,   2035,   1929,   1723},
    {   110,   2035,   1928,   1722},
    {   110,   2035,   1928,   1721},
    {   109,   2035,   1928,   1719},
    {   109,   2034,   1927,   1718},
    {   109,   2034,   1927,   1717},
    {   109,   2034,   1926,   1716},
    {   109,   2034,   1926,   1715},
    {   109,   2034,   1926,   1714},
    {   108,   2034,   1925,   1713},
    {   108,   2034,   1925,   1712},
    {   108,   2034,   1924,   1711},
    {   108,   2034,   1924,   1710},
    {   108,   2034,   1924,   1709},
    {   107,   2034,   1923,   1708},
    {   107,   2034,   1923,   1707},
    {   107,   2034,   1922,   1705},
    {   107,   2034,   1922,   1704},
    {   107,   2034,   1922,   1703},
    {   107,   2034,   1921,   1702},
    {   106,   2034,   1921,   1701},
    {   106,   2034,   1920,   1700},
    {   106,   2034,   1920,   1699},
    {   106,   2034,   1920,   1698},
    {   106,   2034,   1919,   1697},
    {   106,   2033,   1919,   1696},
    {   105,   2033,   1918,   1695},
    {   105,   2033,   1918,   1693},
    {   105,   2033,   1917,   1692},
    {   105,   2033,   1917,   1691},
};
#endif
#else
/* Assume 4096.0/s sampling */
#if MAINS_NOMINAL_FREQUENCY == 50
const int16_t harm_factors[][4] =
{
    {   172,   2043,   1999,   1914},
    {   172,   2043,   1999,   1913},
    {   172,   2043,   1999,   1912},
    {   171,   2042,   1999,   1912},
    {   171,   2042,   1998,   1911},
    {   171,   2042,   1998,   1911},
    {   170,   2042,   1998,   1910},
    {   170,   2042,   1998,   1910},
    {   170,   2042,   1998,   1909},
    {   169,   2042,   1997,   1908},
    {   169,   2042,   1997,   1908},
    {   169,   2042,   1997,   1907},
    {   168,   2042,   1997,   1907},
    {   168,   2042,   1997,   1906},
    {   168,   2042,   1996,   1906},
    {   167,   2042,   1996,   1905},
    {   167,   2042,   1996,   1904},
    {   167,   2042,   1996,   1904},
    {   166,   2042,   1996,   1903},
    {   166,   2042,   1995,   1903},
    {   165,   2042,   1995,   1902},
    {   165,   2042,   1995,   1902},
    {   165,   2042,   1995,   1901},
    {   164,   2042,   1994,   1900},
    {   164,   2042,   1994,   1900},
    {   164,   2042,   1994,   1899},
    {   164,   2042,   1994,   1899},
    {   163,   2042,   1994,   1898},
    {   163,   2042,   1993,   1897},
    {   163,   2042,   1993,   1897},
    {   162,   2042,   1993,   1896},
    {   162,   2042,   1993,   1896},
    {   162,   2042,   1993,   1895},
    {   161,   2042,   1992,   1895},
    {   161,   2042,   1992,   1894},
    {   161,   2042,   1992,   1893},
    {   160,   2042,   1992,   1893},
    {   160,   2042,   1991,   1892},
    {   160,   2042,   1991,   1892},
    {   159,   2042,   1991,   1891},
    {   159,   2042,   1991,   1890},
    {   159,   2042,   1991,   1890},
    {   158,   2042,   1990,   1889},
    {   158,   2042,   1990,   1888},
    {   158,   2042,   1990,   1888},
    {   158,   2041,   1990,   1887},
    {   157,   2041,   1989,   1887},
    {   157,   2041,   1989,   1886},
    {   157,   2041,   1989,   1885},
    {   156,   2041,   1989,   1885},
    {   156,   2041,   1989,   1884},
};
#else
/* Assume 60Hz */
const int16_t harm_factors[][4] =
{
    {   142,   2040,   1977,   1852},
    {   142,   2040,   1976,   1851},
    {   142,   2040,   1976,   1851},
    {   142,   2040,   1976,   1850},
    {   141,   2040,   1976,   1849},
    {   141,   2040,   1975,   1849},
    {   141,   2040,   1975,   1848},
    {   141,   2040,   1975,   1847},
    {   141,   2040,   1975,   1847},
    {   140,   2040,   1974,   1846},
    {   140,   2040,   1974,   1845},
    {   140,   2040,   1974,   1845},
    {   140,   2040,   1974,   1844},
    {   139,   2040,   1973,   1843},
    {   139,   2040,   1973,   1843},
    {   139,   2040,   1973,   1842},
    {   139,   2040,   1973,   1841},
    {   138,   2040,   1972,   1840},
    {   138,   2040,   1972,   1840},
    {   138,   2040,   1972,   1839},
    {   138,   2039,   1972,   1838},
    {   137,   2039,   1971,   1838},
    {   137,   2039,   1971,   1837},
    {   137,   2039,   1971,   1836},
    {   137,   2039,   1971,   1836},
    {   137,   2039,   1970,   1835},
    {   136,   2039,   1970,   1834},
    {   136,   2039,   1970,   1834},
    {   136,   2039,   1970,   1833},
    {   136,   2039,   1969,   1832},
    {   135,   2039,   1969,   1831},
    {   135,   2039,   1969,   1831},
    {   135,   2039,   1969,   1830},
    {   135,   2039,   1968,   1829},
    {   135,   2039,   1968,   1829},
    {   134,   2039,   1968,   1828},
    {   134,   2039,   1968,   1827},
    {   134,   2039,   1967,   1826},
    {   134,   2039,   1967,   1826},
    {   133,   2039,   1967,   1825},
    {   133,   2039,   1967,   1824},
    {   133,   2039,   1966,   1824},
    {   133,   2039,   1966,   1823},
    {   133,   2039,   1966,   1822},
    {   132,   2039,   1965,   1821},
    {   132,   2039,   1965,   1821},
    {   132,   2039,   1965,   1820},
    {   132,   2039,   1965,   1819},
    {   131,   2039,   1964,   1819},
    {   131,   2039,   1964,   1818},
    {   131,   2039,   1964,   1817},
};
#endif
#endif

void goertzel_init(goertzel_state_t *s)
{
    s->v2 =
    s->v3 = 0;
}

void goertzel_update(goertzel_state_t *s, int16_t x, int16_t fac)
{
    int32_t v1;

    v1 = s->v2;
    s->v2 = s->v3;
    s->v3 = ((fac*s->v2) >> 10) - v1 + x;
}

int32_t goertzel_result(goertzel_state_t *s, int16_t fac)
{
    int32_t v1;
    int32_t v2;
    int32_t v3;

    /* Push a zero through the process to finish things off. */
    v1 = s->v2;
    s->v2 = s->v3;
    s->v3 = ((fac*s->v2) >> 10) - v1;
    
    /* Now calculate the non-recursive side of the filter. */
    /* The result here is not scaled down to allow for the magnification
       effect of the filter (the usual DFT magnification effect). */
    v2 = s->v2 >> 5;
    v3 = s->v3 >> 5;
    return v3*v3 + v2*v2 - ((v2*v3) >> 10)*fac;
}

#if NUM_PHASES == 1
static __inline__ void init_harmonics()
#else
static __inline__ void init_harmonics(struct phase_parms_s *phase)
#endif
{
        goertzel_init(&(phase->current.harm_1));
        goertzel_init(&(phase->current.harm_3));
        goertzel_init(&(phase->current.harm_5));
        phase->current.harmonic_step = phase->frequency/10 - 475;
        phase->current.harmonic_samples = harm_factors[phase->current.harmonic_step][0];
}

#if NUM_PHASES == 1
static __inline__ void harmonics(int16_t iamp)
#else
static __inline__ void harmonics(struct phase_parms_s *phase, int16_t iamp)
#endif
{
    int step;
    
    step = phase->current.harmonic_step;

    goertzel_update(&(phase->current.harm_1), iamp, harm_factors[step][1]);
    goertzel_update(&(phase->current.harm_3), iamp, harm_factors[step][2]);
    goertzel_update(&(phase->current.harm_5), iamp, harm_factors[step][3]);
    if (--phase->current.harmonic_samples <= 0)
    {
        phase->current.pow_1 = goertzel_result(&(phase->current.harm_1), harm_factors[step][1]);
        phase->current.pow_3 = goertzel_result(&(phase->current.harm_3), harm_factors[step][2]);
        phase->current.pow_5 = goertzel_result(&(phase->current.harm_5), harm_factors[step][3]);
#if NUM_PHASES == 1
        init_harmonics();
#else
        init_harmonics(phase);
#endif
    }
}
#endif

int main(int argc, char *argv[])
{
    float phasex;
    float amp;
    int16_t iamp;
    int i;
    int loops;
    int freq;
    int samples;
    int32_t res31;
    int32_t res51;
    struct phase_parms_s phases[3];
    
    phasex = 0.0;
    freq = 525;
#if NUM_PHASES == 1
    init_harmonics();
#else
    init_harmonics(&phases[0]);
#endif
    for (i = 0;  i < 8000;  i++)
    {
        amp = cos(phasex)*1.0 + cos(3.0*phasex)*0.10 + cos(5.0*phasex)*0.75;
        phasex += (freq/10.0)*2.0*3.1415926/3276.8;
        iamp = amp*1900.0;
#if NUM_PHASES == 1
        harmonics(iamp);
#else
        harmonics(&phases[0], iamp);
#endif
        res31 = isqrt32(phases[0].current.pow_3)/(isqrt32(phases[0].current.pow_1) >> 16);
        res51 = isqrt32(phases[0].current.pow_5)/(isqrt32(phases[0].current.pow_1) >> 16);
    }
    return  0;
}

































#if defined(HARMONICS_SUPPORT)
/* Coeffs are 2048*cos(2*PI*freq/sample_rate) */
/* The length of the Goertzel transform is set to the nearest approx. to a whole number of mains
   cycles to minimise ripple across successive outputs. The coefficient is tuned to be close to
   the current exact mains frequency to keep the frequency right in the centre of the DFT bin,
   where the transform's gain is known. */
/* These values are for 50Hz and 3276.8 samples/second */
const int16_t harm_factors[][4] =
{
    {   138,   2040,   1972,   1839},
    {   138,   2039,   1972,   1838},
    {   137,   2039,   1971,   1838},
    {   137,   2039,   1971,   1837},
    {   137,   2039,   1971,   1836},
    {   137,   2039,   1970,   1835},
    {   136,   2039,   1970,   1834},
    {   136,   2039,   1970,   1833},
    {   136,   2039,   1969,   1832},
    {   135,   2039,   1969,   1831},
    {   135,   2039,   1969,   1831},
    {   135,   2039,   1968,   1830},
    {   135,   2039,   1968,   1829},
    {   134,   2039,   1968,   1828},
    {   134,   2039,   1968,   1827},
    {   134,   2039,   1967,   1826},
    {   133,   2039,   1967,   1825},
    {   133,   2039,   1967,   1824},
    {   133,   2039,   1966,   1823},
    {   133,   2039,   1966,   1823},
    {   132,   2039,   1966,   1822},
    {   132,   2039,   1965,   1821},
    {   132,   2039,   1965,   1820},
    {   132,   2039,   1965,   1819},
    {   131,   2039,   1964,   1818},
    {   131,   2039,   1964,   1817},       /* Centre frequency */
    {   131,   2039,   1964,   1816},
    {   131,   2039,   1963,   1815},
    {   130,   2038,   1963,   1814},
    {   130,   2038,   1963,   1814},
    {   130,   2038,   1962,   1813},
    {   130,   2038,   1962,   1812},
    {   129,   2038,   1962,   1811},
    {   129,   2038,   1961,   1810},
    {   129,   2038,   1961,   1809},
    {   129,   2038,   1960,   1808},
    {   128,   2038,   1960,   1807},
    {   128,   2038,   1960,   1806},
    {   128,   2038,   1959,   1805},
    {   128,   2038,   1959,   1804},
    {   127,   2038,   1959,   1803},
    {   127,   2038,   1958,   1802},
    {   127,   2038,   1958,   1802},
    {   127,   2038,   1958,   1801},
    {   126,   2038,   1957,   1800},
    {   126,   2038,   1957,   1799},
    {   126,   2038,   1957,   1798},
    {   126,   2038,   1956,   1797},
    {   125,   2038,   1956,   1796},
    {   125,   2038,   1956,   1795},
    {   125,   2038,   1955,   1794},
};

void goertzel_init(goertzel_state_t *s)
{
    s->v2 =
    s->v3 = 0;
}

void goertzel_update(goertzel_state_t *s, int16_t x, int16_t fac)
{
    int32_t v1;

    v1 = s->v2;
    s->v2 = s->v3;
    s->v3 = ((fac*s->v2) >> 10) - v1 + x;
}

int32_t goertzel_result(goertzel_state_t *s, int16_t fac)
{
    int32_t v1;
    int32_t v2;
    int32_t v3;

    /* Push a zero through the process to finish things off. */
    v1 = s->v2;
    s->v2 = s->v3;
    s->v3 = ((fac*s->v2) >> 10) - v1;

    /* Now calculate the non-recursive side of the filter. */
    /* The result here is not scaled down to allow for the magnification
       effect of the filter (the usual DFT magnification effect). */
    v2 = s->v2 >> 5;
    v3 = s->v3 >> 5;
    return v3*v3 + v2*v2 - ((v2*v3) >> 10)*fac;
}

    #if NUM_PHASES == 1
static __inline__ void init_harmonics()
    #else
static __inline__ void init_harmonics(struct phase_parms_s *phase)
    #endif
{
        goertzel_init(&(phase->metrology.current.harm_1));
        goertzel_init(&(phase->metrology.current.harm_3));
        goertzel_init(&(phase->metrology.current.harm_5));
        phase->metrology.current.harmonic_step = phase->frequency/10 - 475;
        phase->metrology.current.harmonic_samples = harm_factors[phase->metrology.current.harmonic_step][0];
}

    #if NUM_PHASES == 1
static __inline__ void harmonics(int16_t iamp)
    #else
static __inline__ void harmonics(struct phase_parms_s *phase, int16_t iamp)
    #endif
{
    int step;

    step = phase->metrology.current.harmonic_step;

    goertzel_update(&(phase->metrology.current.harm_1), iamp, harm_factors[step][1]);
    goertzel_update(&(phase->metrology.current.harm_3), iamp, harm_factors[step][2]);
    goertzel_update(&(phase->metrology.current.harm_5), iamp, harm_factors[step][3]);
    if (--phase->metrology.current.harmonic_samples <= 0)
    {
        phase->metrology.current.pow_1 = goertzel_result(&(phase->metrology.current.harm_1), harm_factors[step][1]);
        phase->metrology.current.pow_3 = goertzel_result(&(phase->metrology.current.harm_3), harm_factors[step][2]);
        phase->metrology.current.pow_5 = goertzel_result(&(phase->metrology.current.harm_5), harm_factors[step][3]);
    #if NUM_PHASES == 1
        init_harmonics();
    #else
        init_harmonics(phase);
    #endif
    }
}
#endif
