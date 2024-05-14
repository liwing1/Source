/*******************************************************************************
 *  iiceeprom.c -
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
#include <stdbool.h>
#include "emeter-toolkit.h"

// These routines support a single I2C EEPROM or FRAM attached to
// two pins of a port on an MSP430.
//
//Please note:
//
// These routines have been tested with an Atmel 2401. They
// are intended to work with a range of I2C interface serial
// EEPROMs, but should be thoroughly tested against any
// other EEPROM before serious use.
//
// Since the EEPROM will be operating at a low voltage, when
// connected to the MSP430, this code is designed to operate
// at the worst case timing for the Atmel chips, at their
// lowest recommended supply voltage, and with the MSP430
// running at 8MHz. They could be tuned for faster operation
// in other environments.

#if defined(SUPPORT_IICEEPROM)

#define MAX_IIC_TRIES 200

#if defined(EEPROM_IS_REALLY_FRAM)
/* No delay is needed for FRAMs */
#define delay(x) /**/
#else
/* This delay is needed for EEPROMs */
static void delay(int i)
{
    for (  ;  i;  i--)
    {
        __no_operation();
        __no_operation();
        __no_operation();
        __no_operation();
    }
}
#endif

static void iic_start(void)
{
    /* Send a start to the EEPROM */
    /* A start is the data bit falling while the clock is high */
    IICEEPROM_PORT_DIR &= ~IICEEPROM_SDA_BIT;
    delay(2);   /* At least ??? */
    IICEEPROM_PORT_DIR &= ~IICEEPROM_SCL_BIT;
    delay(5);   /* At least 4.7us */
    IICEEPROM_PORT_DIR |= IICEEPROM_SDA_BIT;
    delay(5);   /* At least 4.7us */
    IICEEPROM_PORT_DIR |= IICEEPROM_SCL_BIT;
}

static void iic_stop(void)
{
    /* Send a stop to the EEPROM */
    /* A stop is the data bit rising while the clock is high */
    IICEEPROM_PORT_DIR |= IICEEPROM_SDA_BIT;
    delay(2);   /* At least ??? */
    IICEEPROM_PORT_DIR &= ~IICEEPROM_SCL_BIT;
    delay(5);   /* At least 4.7us */
    IICEEPROM_PORT_DIR &= ~IICEEPROM_SDA_BIT;
    delay(5);   /* At least 4.7us */
    IICEEPROM_PORT_DIR |= IICEEPROM_SCL_BIT;
}

static int iic_send(uint8_t data)
{
    int i;
    
    /* Send 8 bits of data */
    IICEEPROM_PORT_DIR |= IICEEPROM_SCL_BIT;
    for (i = 8;  i > 0;  i--)
    {
        /* The data can now change without delay */
        if (data & 0x80)
            IICEEPROM_PORT_DIR &= ~IICEEPROM_SDA_BIT;
        else
            IICEEPROM_PORT_DIR |= IICEEPROM_SDA_BIT;
        data <<= 1;
        /* Pulse the clock high while the data bit is steady */
        delay(5);   /* At least 4.7us */
        IICEEPROM_PORT_DIR &= ~IICEEPROM_SCL_BIT;
        delay(5);   /* At least 4.7us */
        IICEEPROM_PORT_DIR |= IICEEPROM_SCL_BIT;
    }
    /* Check the acknowledgement from the EEPROM */
    /* Pulse the clock high, and see what the device says while the clock is high */
    delay(5);
    IICEEPROM_PORT_DIR &= ~IICEEPROM_SDA_BIT;
    delay(5);
    IICEEPROM_PORT_DIR &= ~IICEEPROM_SCL_BIT;
    delay(5);
    i = IICEEPROM_PORT_IN & IICEEPROM_SDA_BIT;
    IICEEPROM_PORT_DIR |= IICEEPROM_SCL_BIT;
    delay(4);
    return i;
}

static uint8_t iic_receive(int ok)
{
    uint8_t data;
    int i;

    /* Get 8 bits of data */
    IICEEPROM_PORT_DIR &= ~IICEEPROM_SDA_BIT;     /* Input */
    data = 0;
    for (i = 8;  i > 0;  i--)
    {
        IICEEPROM_PORT_DIR &= ~IICEEPROM_SCL_BIT;
        delay(5);   /* At least 4.7us */
        data <<= 1;
        if (IICEEPROM_PORT_IN & IICEEPROM_SDA_BIT)
            data |= 0x01;
        IICEEPROM_PORT_DIR |= IICEEPROM_SCL_BIT;
        delay(5);   /* At least 4.7us */
    }
    /* Send the acknowledgement to the EEPROM */
    if (ok)
        IICEEPROM_PORT_DIR |= IICEEPROM_SDA_BIT;
    else
        IICEEPROM_PORT_DIR &= ~IICEEPROM_SDA_BIT;
    delay(4);
    IICEEPROM_PORT_DIR &= ~IICEEPROM_SCL_BIT;
    delay(4);
    IICEEPROM_PORT_DIR |= IICEEPROM_SCL_BIT;
    IICEEPROM_PORT_DIR &= ~IICEEPROM_SDA_BIT;
    return data;
}

static int test_SDA(void)
{
    int i;
  
    iic_stop();
    IICEEPROM_PORT_DIR &= ~IICEEPROM_SDA_BIT;
    delay(4);
    for (i = 16;  i > 0;  i--)
    {
        delay(5);
        if (!(IICEEPROM_PORT_IN & IICEEPROM_SDA_BIT))
            break;
    }
    return i;
}

int iicEEPROM_read(uint16_t addr, void *dat, int len)
{
    int i;
    int j;
    uint8_t *p;

    for (i = 0;  i < MAX_IIC_TRIES;  ++i)
    {
        if (i)
        {
            /* Read false, retry */
            if (test_SDA())
                continue;
        }
        iic_start();
#if EEPROM_PAGE_SIZE == 32
        if (iic_send(0xA0)  ||  iic_send(addr/0x100)  ||  iic_send(addr))
            continue;
#else
        if (iic_send(0xA0 | ((uint8_t)(addr/0x100)*2))  ||  iic_send(addr))
            continue;
#endif
        p = (uint8_t *) dat;

        iic_start();
#if EEPROM_PAGE_SIZE == 32
        if (iic_send(0xA1))
            continue;
#else
        if (iic_send(0xA1 | ((uint8_t)(addr/0x100)*2)))
            continue;
#endif
        for (j = len;  j > 0;  j--)
            *p++ = iic_receive(true);
        *p = iic_receive(false);
        iic_stop();
        return true;
    }
    iic_stop();
    return false;
}

int iicEEPROM_write(uint16_t addr, void *dat, int len)
{
    int i;
    int j;
    int section_len;
    uint8_t *p;
    uint8_t *q;

    /* If the write spreads across pages in the EEPROM, we need to split the write
       into sections. */
    q = (uint8_t *) dat;
    while (len > 0)
    {
        if (addr + len > ((addr + EEPROM_PAGE_SIZE) & ~(EEPROM_PAGE_SIZE - 1)))
            section_len = ((addr + EEPROM_PAGE_SIZE) & ~(EEPROM_PAGE_SIZE - 1)) - addr;
        else
            section_len = len;
        for (i = 0;  i < MAX_IIC_TRIES;  ++i)
        {
            if (i)
            {
                /* Write false, retry */
                if (test_SDA())
                    continue;
            }

            iic_start();
#if EEPROM_PAGE_SIZE == 32
            if (iic_send(0xA0)  ||  iic_send(addr/0x100)  ||  iic_send(addr))
                continue;
#else
            if (iic_send(0xA0 | ((uint8_t)(addr/0x100)*2))  ||  iic_send(addr))
                continue;
#endif
            p = q;
            for (j = section_len;  j > 0;  j--)
            {
                if (iic_send(*p++))
                    break;
            }
            if (j == 0)
                break;
            iic_stop();
        }
        iic_stop();
        if (i >= MAX_IIC_TRIES)
            return false;
        len -= section_len;
        addr += section_len;
        q += section_len;
    }
    return true;
}

int iicEEPROM_init(void)
{
    int i;

    /* While idling, the EEPROM clock should be low */
    IICEEPROM_PORT_DIR |= IICEEPROM_SCL_BIT;
    /* If we happen to have restarted in the middle of a read from
       the EEPROM/FRAM, we need to regain control of the device. If we
       give it enough clocks, and do no acknowledge it we should get out
       of any odd conditions. Then we do a stop, and we should be OK. If
       the device was mid write when the restart occurred we cannot really
       act in a clean way. */
    delay(5);   /* At least 4.7us */
    for (i = 0;  i < 16;  i++)
    {
        IICEEPROM_PORT_DIR &= ~IICEEPROM_SCL_BIT;
        delay(5);   /* At least 4.7us */
        IICEEPROM_PORT_DIR |= IICEEPROM_SCL_BIT;
        delay(5);   /* At least 4.7us */
    }
    iic_stop();
    return 0;
}
#endif
