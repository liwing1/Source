/*******************************************************************************
 *  emeter-toolkit.h -
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

/*! \file */

#if !defined(_EMETER_TOOLKIT_H_)
#define _EMETER_TOOLKIT_H_

#if defined(__GNUC__)
#include <intrinsics.h>
#endif

#if defined(__TI_COMPILER_VERSION__)
typedef unsigned short istate_t;
#elif defined(__GNUC__)
typedef __istate_t istate_t;
#endif

#if defined(__MSP430__)
#include <msp430.h>
#include <isr_compat.h>
#endif

#if defined(__IAR_SYSTEMS_ICC__)
#include <iar/extra_peripheral_definitions.h>
#endif
#if (defined(__TI_COMPILER_VERSION__)  &&  __TI_COMPILER_VERSION__ >= 4000000)
#include <ccs/extra_peripheral_definitions.h>
#endif
#if defined(__GNUC__)
#include <gcc/extra_peripheral_definitions.h>
#endif

#if defined(__GNUC__)  ||  defined(__IAR_SYSTEMS_ICC__)  ||  (defined(__TI_COMPILER_VERSION__)  &&  __TI_COMPILER_VERSION__ >= 4000000)
#define EMETER_TOOLKIT_SUPPORT_64BIT
#endif

#if !defined(__MSP430__)
/*! Attribute to push data into the info memory */
#define __infomem__                 /**/
/*! Attribute to push data into an eraseable segment of memory */
#define __erasablemem__             /**/
/*! Attribute to push data into an uninitialised segment of memory. This is
    useful for data which is required to persist through a reset. */
#define __uninitialized__           /**/
#define _EINT()                     /**/
#define _DINT()                     /**/
#elif defined(__IAR_SYSTEMS_ICC__)
/*! Attribute to push data into an uninitialised segment of memory. This is
    useful for data which is required to persist through a reset. */
#define __uninitialized__           /**/
/*! Attribute to push data into an eraseable segment of memory */
#define __erasablemem__             /**/
#define __inline__                  inline
#elif defined(__TI_COMPILER_VERSION__)
/*! Attribute to push data into an uninitialised segment of memory. This is
    useful for data which is required to persist through a reset. */
#define __uninitialized__           /**/
/*! Attribute to push data into an eraseable segment of memory */
#define __erasablemem__             /**/
#define __inline__                  inline
#elif defined(__GNUC__)
/*! Attribute to push data into an uninitialised segment of memory. This is
    useful for data which is required to persist through a reset. */
#define __uninitialized__           __attribute__ ((section(".noinit")))
/*! Attribute to push data into an eraseable segment of memory */
#define __erasablemem__             __attribute__ ((section(".erasabletext")))
#else
/*! Attribute to push data into the info memory */
#define __infomem__                 /**/
/*! Attribute to push data into the info memory, as uninitialised data */
#define __infomem_uninitialized__   /**/
/*! Attribute to push data into an uninitialised segment of memory. This is
    useful for data which is required to persist through a reset. */
#define __uninitialized__           /**/
/*! Attribute to push data into an eraseable segment of memory */
#define __erasablemem__             /**/
#endif

/*!
    \brief 16 x 16 => 16 multiply in Q1.15 format
    \param x
    \param y
    \return Result
 */
#if (defined(__GNUC__)  ||  defined(__IAR_SYSTEMS_ICC__)  ||  defined(__TI_COMPILER_VERSION__))  &&  defined(__MSP430_HAS_MPY32__)  &&  !defined(__TOOLKIT_USE_SOFT_MPY__)
static __inline__ int16_t q1_15_mul(int16_t x, int16_t y)
{
	int32_t res;
    istate_t istate;

    istate = __get_interrupt_state();
    __disable_interrupt();
    MPYS = y;
    OP2 = x;
    res = RES32[0] << 1;
    __set_interrupt_state(istate);
    return res >> 16;
}
#else
extern int16_t q1_15_mul(int16_t x, int16_t y);
#endif

/*!
    \brief 16 x 16 => 16 multiply in Q1.15 format with half bit rounding of the result
    \param x
    \param y
    \return Result
 */
#if (defined(__GNUC__)  ||  defined(__IAR_SYSTEMS_ICC__)  ||  defined(__TI_COMPILER_VERSION__))  &&  defined(__MSP430_HAS_MPY32__)  &&  !defined(__TOOLKIT_USE_SOFT_MPY__)
static __inline__ int16_t q1_15_mulr(int16_t x, int16_t y)
{
	int32_t res;
    istate_t istate;

    istate = __get_interrupt_state();
    __disable_interrupt();
    MPYS = y;
    OP2 = x;
    res = (RES32[0] + 0x4000L) << 1;
    __set_interrupt_state(istate);
    return res >> 16;
}
#else
extern int16_t q1_15_mulr(int16_t x, int16_t y);
#endif

/*!
    \brief 16 x 16 => 32 signed multiply
    \param x
    \param y
    \return x*y
 */
#if (defined(__GNUC__)  ||  defined(__IAR_SYSTEMS_ICC__)  ||  defined(__TI_COMPILER_VERSION__))  &&  defined(__MSP430_HAS_MPY32__)  &&  !defined(__TOOLKIT_USE_SOFT_MPY__)
static __inline__ int32_t imul16(int16_t x, int16_t y)
{
	int32_t res;
    istate_t istate;

    istate = __get_interrupt_state();
    __disable_interrupt();
    MPYS = y;
    OP2 = x;
    res = RES32[0];
    __set_interrupt_state(istate);
    return res;
}
#else
int32_t imul16(int16_t x, int16_t y);
#endif

/*! Multiply 2 unsigned 16bit numbers, and return the 32 bit result.
    \brief 16 bit unsigned x 16 bit unsigned => 32 bit unsigned multiply
    \param x First 16 bit unsigned operand
    \param y Second 16 bit unsigned operand
    \return The 32 bit unsigned result
 */
#if (defined(__GNUC__)  ||  defined(__IAR_SYSTEMS_ICC__)  ||  defined(__TI_COMPILER_VERSION__))  &&  defined(__MSP430_HAS_MPY32__)  &&  !defined(__TOOLKIT_USE_SOFT_MPY__)
static __inline__ uint32_t imul16u(uint16_t x, uint16_t y)
{
	uint32_t res;
    istate_t istate;

    istate = __get_interrupt_state();
    __disable_interrupt();
    MPY = y;
    OP2 = x;
    res = RES32[0];
    __set_interrupt_state(istate);
    return res;
}
#else
uint32_t imul16u(uint16_t x, uint16_t y);
#endif

/*! The result is a 8.8 bit fractional integer.
    \brief Evaluate the square root of a 16 bit unsigned integer.
    \param x The value for which the square root is required.
    \return The square root in the form 8 integer bits : 8 fractional bits.
*/
uint16_t isqrt16(uint16_t x);

/*! The result is a 16.16 bit fractional integer.
    \brief Evaluate the square root of a 32 bit unsigned integer.
    \param x The value for which the square root is required.
    \return The square root in the form 16 integer bits : 16 fractional bits.
*/
uint32_t isqrt32(uint32_t x);

/*! The result is a 16 bit unsigned integer.
    \brief Evaluate the square root of a 32 bit unsigned integer.
    \param x The value for which the square root is required.
    \return The square root, as a 16 bit unsigned integer.
*/
uint16_t isqrt32i(uint32_t x);

#if defined(EMETER_TOOLKIT_SUPPORT_64BIT)
/*! The result is a 32.32 bit fractional integer.
    \brief Evaluate the square root of a 64 bit unsigned integer.
    \param x The value for which the square root is required.
    \return The square root in the form 32 integer bits : 32 fractional bits.
*/
uint64_t isqrt64(uint64_t x);

/*! The result is a 32 bit unsigned integer.
    \brief Evaluate the square root of a 64 bit unsigned integer.
    \param x The value for which the square root is required.
    \return The square root, as a 32 bit unsigned integer.
*/
uint32_t isqrt64i(uint64_t x);
#endif

/*! This is intended for the removal of the DC content from 16 bit 50Hz/60Hz mains
    signals, sampled at 4096 samples/second. All samples should be passed through
    the filter, in sequence, to obtain a signal free of DC content. The estimation
    is based on a noise shaped single pole LPF. The cutoff frequency of this filter
    is set very low, so its gain is essentially flat from 45Hz upwards. This means
    the filter can take several seconds to stabilise when the signal is initially
    applied. This can be mitigated by priming the filter with an initial state close
    to the expected stable state. The filter state variable should be initialised,
    using dc_filter16_init(), before the filter is used.
    \brief Estimate the DC content in a signal, and remove the DC content from the signal.
    \param p A pointer to the filter state variable.
    \param x The next signal sample.
    \return The signal free of DC content.
    \see dc_filter16_no_update
    \see dc_filter16_init
    \see dc_filter16_estimate
    \see dc_filter24
    \see dc_filter24_no_update
    \see dc_filter24_init
    \see dc_filter24_estimate
*/
int16_t dc_filter16(int32_t *p, int16_t x);

/*! \brief Remove the estimated DC content from a signal sample, without updating the DC estimate.
    \param p A pointer to the filter state variable.
    \param x A signal sample to be filtered.
    \return The signal free of DC content.
    \see dc_filter16
    \see dc_filter16_init
    \see dc_filter16_estimate
    \see dc_filter24
    \see dc_filter24_no_update
    \see dc_filter24_init
    \see dc_filter24_estimate
*/
int16_t dc_filter16_no_update(const int32_t *p, int16_t x);

/*! \brief Initialise a DC estimator/filter for a 16 bit signal.
    \param p A pointer to the filter state variable.
    \param x An initial estimate of the DC, to prime the estimator.
    \see dc_filter16
    \see dc_filter16_no_update
    \see dc_filter16_estimate
    \see dc_filter24
    \see dc_filter24_no_update
    \see dc_filter24_init
    \see dc_filter24_estimate
*/
void dc_filter16_init(int32_t *p, int16_t x);

/*! \brief Get the current DC estimate for a 16 bit signal.
    \param p A pointer to the filter state variable.
    \return The actual DC estimate.
    \see dc_filter16
    \see dc_filter16_no_update
    \see dc_filter16_init
    \see dc_filter24
    \see dc_filter24_no_update
    \see dc_filter24_init
    \see dc_filter24_estimate
*/
int32_t dc_filter16_estimate(const int32_t *p);

/*! This is intended for the removal of the DC content from 24 bit 50Hz/60Hz mains
    signals, sampled at 4096 samples/second. All samples should be passed through
    the filter, in sequence, to obtain a signal free of DC content. The estimation
    is based on a noise shaped single pole LPF. The cutoff frequency of this filter
    is set very low, so its gain is essentially flat from 45Hz upwards. This means
    the filter can take several seconds to stabilise when the signal is initially
    applied. This can be mitigated by priming the filter with an initial state close
    to the expected stable state. The filter state variable should be initialised,
    using dc_filter16_init(), before the filter is used.
    \brief Estimate the DC content in a signal, and remove the DC content from the signal.
    \param p A pointer to the filter state variable.
    \param x The next signal sample.
    \return The signal free of DC content.
    \see dc_filter16
    \see dc_filter16_no_update
    \see dc_filter16_init
    \see dc_filter16_estimate
    \see dc_filter24_no_update
    \see dc_filter24_init
    \see dc_filter24_estimate
*/
int32_t dc_filter24(int16_t p[3], int32_t x);

/*! \brief Remove the estimated DC content from a signal sample, without updating the DC estimate.
    \param p A pointer to the filter state variable.
    \param x A signal sample to be filtered.
    \return The signal free of DC content.
    \see dc_filter16
    \see dc_filter16_no_update
    \see dc_filter16_init
    \see dc_filter16_estimate
    \see dc_filter24
    \see dc_filter24_init
    \see dc_filter24_estimate
*/
int32_t dc_filter24_no_update(const int16_t p[3], int32_t x);

/*! \brief Initialise a DC estimator/filter for a 24 bit signal.
    \param p A pointer to the filter state variable.
    \param x An initial estimate of the DC, to prime the estimator.
    \see dc_filter16
    \see dc_filter16_no_update
    \see dc_filter16_init
    \see dc_filter16_estimate
    \see dc_filter24
    \see dc_filter24_no_update
    \see dc_filter24_estimate
*/
void dc_filter24_init(int16_t p[3], int16_t x);

/*! \brief Get the current DC estimate for a 24 bit signal.
    \param p A pointer to the filter state variable.
    \return The actual DC estimate.
    \see dc_filter16
    \see dc_filter16_no_update
    \see dc_filter16_init
    \see dc_filter16_estimate
    \see dc_filter24
    \see dc_filter24_no_update
    \see dc_filter24_init
*/
int32_t dc_filter24_estimate(const int16_t p[3]);

/*! The maximum possible result is 5 digits long, but to keep things byte aligned
    the result is returned as a 6 nibble value packed into a 3 byte buffer. The first
    nibble is always zero.
    \brief Convert a 16 bit unsigned integer to a nibble packed BCD string.
    \param bcd The buffer which will contain the BCD result.
    \param bin The 16 bit unsigned integer value to be converted.
    \see bin2bcd32
    \see bin2bcd64
*/
void bin2bcd16(uint8_t bcd[3], uint16_t bin);

/*! The result is returned as a 10 nibble value packed into a 5 byte buffer.
    \brief Convert a 32 bit unsigned integer to a nibble packed BCD string.
    \param bcd The buffer which will contain the BCD result.
    \param bin The 32 bit unsigned integer value to be converted.
    \see bin2bcd16
    \see bin2bcd64
*/
void bin2bcd32(uint8_t bcd[5], uint32_t bin);

#if defined(EMETER_TOOLKIT_SUPPORT_64BIT)
/*! The result is returned as a 20 nibble value packed into a 10 byte buffer.
    \brief Convert a 64 bit unsigned integer to a nibble packed BCD string.
    \param bcd The buffer which will contain the BCD result.
    \param bin The 64 bit unsigned integer value to be converted.
    \see bin2bcd16
    \see bin2bcd32
*/
void bin2bcd64(uint8_t bcd[10], uint64_t bin);
#endif

/*! The uses a linear congruel technique, resulting in the LSB having no randomness
    at all, and the MSB being very random. If you need less than 16 bits shift the
    returned value down to the size you need to maintain the maximum possible
    randomness.
    \brief Obtain an evenly distributed random 16 bit integer.
    \return The 16 bit signed random value.
*/
int16_t rand16(void);

/*! This version is very fast. It is based on a table of 256 steps per quadrant (i.e.
    step 0.352 degrees apart). This limits the accuracy of individual samples, but the
    bias over many samples is small. 
    \brief Look up the amplitude of a sine wave at a specified phase angle.
    \param phase The phase angle, on a scale where 0 represents zero degrees,
                 and 0xFFFFFFFF represents 359.999999 degrees.
    \return The amplitude of a sine wave at the specified phase.
*/
extern int16_t dds_lookup(uint32_t phase);

/*! This version is a little slower than dds_lookup(), but achieves better accuracy (about 15 bits).
    It works by interpolating between steps in a 256 steps per quadrant table.
    \brief Look up the amplitude of a sine wave at a specified phase angle.
    \param phase The phase angle, on a scale where 0 represents zero degrees,
                 and 0xFFFFFFFF represents 359.999999 degrees.
    \return The amplitude of a sine wave at the specified phase.
*/
extern int16_t dds_interpolated_lookup(uint32_t phase);

/*! \brief Increment a phase accumulator by a specified amount of phase, and return the
    amplitude of a sine wave at the new phase angle.
    \param phase_acc The current phase accumulator, on a scale where 0 represents zero degrees,
                     and 0xFFFFFFFF represents 359.999999 degrees.
    \param phase_rate The amount of phase to add, on a scale where 0 represents zero degrees,
                      and 0xFFFFFFFF represents 359.999999 degrees.
    \return The amplitude of a sine wave at the specified phase.
*/
extern int16_t dds(uint32_t *phase_acc, int32_t phase_rate);

#if defined(BCSCTL1_)  &&  defined(TACCR0_)
extern void set_dco(int freq);
#endif

/*! Divide a 48 bit integer (stored as an array of three 16 bit integers) by a 16 bit integer,
    and return the 32 bit result.
    \brief 48 bit / 16 bit => 32 bit.
    \param x The 48 bit integer dividend, stored as an array of three 16 bit integers.
    \param y The divisor.
    \return x/y. */
int32_t div48(int16_t x[3], int16_t y);

/*! Divide a shifted 48 bit integer (stored as an array of three 16 bit integers) by a 16 bit integer,
    and return the 32 bit result.
    \brief 48 bit / 16 bit => 32 bit.
    \param x The 48 bit integer dividend, stored as an array of three 16 bit integers.
    \param sh The number of bits to upward preshift x by, before the division.
    \param y The divisor.
    \return x/y. */
int32_t div_sh48(int16_t x[3], int sh, int16_t y);

/*! Multiply a signed 32 bit integer by an unsigned 16 bit integer, and return the top
    32 bits of the 48 bit result as a signed 32 bit integer. This is useful for applying
    unsigned 16 bit scaling factors (e.g. calibration factors) to signed 32 bit numbers.
    \brief Evaluate ((x*y) >> 16)
    \param x The signed parameter.
    \param y The unsigned parameter.
    \return The signed result.
 */
#if (defined(__GNUC__)  ||  defined(__IAR_SYSTEMS_ICC__)  ||  defined(__TI_COMPILER_VERSION__))  &&  defined(__MSP430_HAS_MPY32__)  &&  !defined(__TOOLKIT_USE_SOFT_MPY__)
static __inline__ int32_t mul48_32_16(int32_t x, uint16_t y)
{
    int32_t res;
    istate_t istate;

    istate = __get_interrupt_state();
    __disable_interrupt();
    MPYS32 = x;
    OP2_16[0] = y;
    OP2_16[1] = 0;
    _NOP();
    _NOP();
    res = *((int32_t *) &RES16[1]);
    __set_interrupt_state(istate);
    return res;
}
#else
int32_t mul48_32_16(int32_t x, uint16_t y);
#endif

/*! Multiply an unsigned 32 bit integer by an unsigned 16 bit integer, and return the top
    32 bits of the 48 bit result as an unsigned 32 bit integer. This is useful for applying
    unsigned 16 bit scaling factors (e.g. calibration factors) to unsigned 32 bit numbers.
    \brief Evaluate ((x*y) >> 16)
    \param x The signed parameter.
    \param y The unsigned parameter.
    \return The signed result.
 */
#if (defined(__GNUC__)  ||  defined(__IAR_SYSTEMS_ICC__)  ||  defined(__TI_COMPILER_VERSION__))  &&  defined(__MSP430_HAS_MPY32__)  &&  !defined(__TOOLKIT_USE_SOFT_MPY__)
static __inline__ uint32_t mul48u_32_16(uint32_t x, uint16_t y)
{
    uint32_t res;
    istate_t istate;

    istate = __get_interrupt_state();
    __disable_interrupt();
    MPY32 = x;
    OP2 = y;
    _NOP();
    _NOP();
    res = *((uint32_t *) &RES16[1]);
    __set_interrupt_state(istate);
    return res;
}
#else
uint32_t mul48u_32_16(uint32_t x, uint16_t y);
#endif

void shift48(int16_t x[3], int how_far);

#if defined(EMETER_TOOLKIT_SUPPORT_64BIT)
/*!
    Assign the value of a 48 bit integer (stored as an array of three 16 bit integers) to a 64 bit
    integer. This operation is performed without managing interrupts, so it is only suitable for
    use where interrupt code could not see a half completed operation.
    \param y The destination 64 bit value, which is zeroed
    \param x A pointer to the source 48 bit value.
 */
static __inline__ int64_t int48_to_64(int16_t x[3])
{
    int64_t y;

    y = x[2];
    y <<= 16;
    y |= (uint16_t) x[1];
    y <<= 16;
    y |= (uint16_t) x[0];
    return  y;
}

/*!
    Assign the value of a 64 bit signed integer to a 48 bit integer (stored as an array of three 16 bit
    integers). This operation is performed without managing interrupts, so it is only suitable for
    use where interrupt code could not see a half completed operation.
    \param y A pointer to the destination 48 bit value.
    \param x The source 64 bit value, which is zeroed
 */
static __inline__ void int64_to_48(int16_t y[3], int64_t x)
{
    y[2] = x >> 32;
    y[1] = x >> 16;
    y[0] = x;
}

/*!
    Assign the value of a 64 bit signed integer to another integer of the same type, and clear (zero)
    the original. This operation is performed without managing interrupts, so it is only suitable for
    use where interrupt code could not see a half completed operation.
    \param y The destination 64 bit value.
    \param x The source 64 bit value, which is zeroed
 */
#define transfer64(y,x) (y = x, x = 0)

/*!
    Assign the value of a 64 bit signed integer to another integer of the same type. This operation is
    performed without managing interrupts, so it is only suitable for use where interrupt code could
    not see a half completed assignment.
    \param y The destination 64 bit value.
    \param x The source 64 bit value.
 */
#define assign64(y,x)   (y = x)
#endif

/*!
    Assign the value of a 48 bit signed integer (stored as an array of three 16 bit integers) to
    another integer of the same type, and clear (zero) the original. This operation is performed
    without managing interrupts, so it is only suitable for use where interrupt code could not see
    a half completed operation.
    \param y A pointer to the destination 48 bit value.
    \param x A pointer to the source 48 bit value, which is zeroed
 */
static __inline__ void transfer48(int16_t y[3], int16_t x[3])
{
    y[2] = x[2];
    y[1] = x[1];
    y[0] = x[0];
    x[0] =
    x[1] =
    x[2] = 0;
}

/*!
    Assign the value of a 48 bit signed integer (stored as an array of three 16 bit integers) to
    another integer of the same type. This operation is performed without managing interrupts, so
    it is only suitable for use where interrupt code could not see a half completed assignment.
    \param y A pointer to the destination 48 bit value.
    \param x A pointer to the source 48 bit value.
 */
static __inline__ void assign48(int16_t y[3], const int16_t x[3])
{
    y[2] = x[2];
    y[1] = x[1];
    y[0] = x[0];
}

/*!
    Clear (zero) a 48 bit signed integer (stored as an array of three 16 bit integers). This operation
    is performed without managing interrupts, so it is only suitable for operations performed when
    interrupts are already disabled.
    \param z A pointer to the 48 bit accumulator.
 */
static __inline__ void clear48(int16_t z[3])
{
    z[2] = 0;
    z[1] = 0;
    z[0] = 0;
}

/*!
    Multiply two 16 bit signed integers, and accumulate the result into a 48 bit signed integer (stored
    as an array of three 16 bit integers). This operation is performed without managing interrupts,
    so it is only suitable for operations performed when interrupts are already disabled.
    \param z A pointer to the 48 bit accumulator.
    \param x A 16 bit number.
    \param y A 16 bit number.
 */
void mac48_16(int16_t z[3], int16_t x, int16_t y);

/*!
    Square a 16 bit signed integer, and accumulate the result into a 48 bit signed integer (stored
    as an array of three 16 bit integers). This operation is performed without managing interrupts,
    so it is only suitable for operations performed when interrupts are already disabled.
    \param z A pointer to the 48 bit accumulator.
    \param x The 16 bit number.
 */
void sqac48_16(int16_t z[3], int16_t x);

/*!
    Square a 24 bit signed integer (stored as a 32 bit integer), and accumulate the result into a
    64 bit signed integer. This operation is performed without managing interrupts,
    so it is only suitable for operations performed when interrupts are already disabled.
    \param z A pointer to the 64 bit accumulator.
    \param x The 24 bit number.
 */
#if (defined(__GNUC__)  ||  defined(__IAR_SYSTEMS_ICC__)  ||  defined(__TI_COMPILER_VERSION__))  &&  defined(__MSP430_HAS_MPY32__)  &&  !defined(__TOOLKIT_USE_SOFT_MPY__)
static __inline__ void sqac64_24(int64_t *z, int32_t x)
{
    /* This does not protect against interrupts. It is only for use within an interrupt routine */
    MPYS32 = x;
    OP2_32X = x;
    *z += RES64;
}
#else
void sqac64_24(int64_t *z, int32_t x);
#endif

/*!
    Multiply a 16 bit signed integer by a 24 bit signed integer (stored as a 32 bit integer), and accumulate
    the result into a 64 bit signed integer. This operation is performed without managing interrupts,
    so it is only suitable for operations performed when interrupts are already disabled.
    \param z A pointer to the 64 bit accumulator.
    \param x The 16 bit number.
    \param y The 32 bit number.
 */
#if (defined(__GNUC__)  ||  defined(__IAR_SYSTEMS_ICC__)  ||  defined(__TI_COMPILER_VERSION__))  &&  defined(__MSP430_HAS_MPY32__)  &&  !defined(__TOOLKIT_USE_SOFT_MPY__)
static __inline__ void mac64_16_24(int64_t *z, int16_t x, int32_t y)
{
    /* This does not protect against interrupts. It is only for use within an interrupt routine */
    MPYS32 = y;
    OP2 = x;
    *z += RES64;
}
#else
void mac64_16_24(int64_t *z, int16_t x, int32_t y);
#endif

void accum48_48(int16_t z[3], int16_t y[3]);

void decum48_48(int16_t z[3], int16_t y[3]);

#if defined(__IAR_SYSTEMS_ICC__)  ||  defined(__TI_COMPILER_VERSION__)
void accum48(int16_t z[3], int32_t y);
#else
static __inline__ void accum48(int16_t x[3], int32_t y)
{
    /* Accumulate a 32 bit integer value into a 48 bit one represented
       by a 3 element int16_t array */
#if defined(__MSP430__)  &&  defined(__GNUC__)
    int16_t y_ex;

    __asm__ __volatile__ (
        " mov   %B[y],%[y_ex] \n"
        " rla   %[y_ex] \n"
        " subc  %[y_ex],%[y_ex] \n"
        " inv   %[y_ex] \n"
        " add   %A[y],0(%[x]) \n"
        " addc  %B[y],2(%[x]) \n"
        " addc  %[y_ex],4(%[x]) \n"
        : 
        : [x] "r"(x), [y] "r"(y), [y_ex] "r"(y_ex));
#else
    int64_t acc;

    acc = (uint16_t) x[2];
    acc <<= 16;
    acc |= (uint16_t) x[1];
    acc <<= 16;
    acc |= (uint16_t) x[0];
    acc += y;
    x[0] = acc;
    acc >>= 16;
    x[1] = acc;
    acc >>= 16;
    x[2] = acc;
#endif
}
#endif

/*! The actual delay produced by this function will depend on the clock speed
    of the core. It should only be used when "a few cycles" of delay are required.
    \brief Briefly pause the software
    \param n A value which determines how long the brief pause will be.
 */
static void __inline__ brief_pause(unsigned int n)
{
    while (n > 0)
    {
        n--;
        __no_operation();
    }
}

/*! \brief Restart the watchdog counter.
 */
static void __inline__ restart_watchdog(void)
{
#if defined(__MSP430__)
    WDTCTL = (WDTCTL & 0xFF) | WDTPW | WDTCNTCL;
#endif
}

#endif
