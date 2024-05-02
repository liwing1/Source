/*******************************************************************************
 *  emeter-iec62056.c -
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

/*! \file emeter-structs.h */

#include <inttypes.h>
#include <stdlib.h>
#include <stdbool.h>
#if !defined(__MSP430__)
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#endif
#if defined(__GNUC__)
#include <signal.h>
#endif
#include <math.h>
#if defined(__MSP430__)
#include <msp430.h>
#endif
#define __MAIN_PROGRAM__

#include "emeter-template.h"

#include <emeter-toolkit.h>
#include <emeter-metrology.h>

#include <uart_comms.h>
#include <iec62056_46_user.h>
#include <iec62056_46_link.h>
#include <iec62056_config.h>
#include <packing.h>

#include "emeter-app.h"
#include "emeter-rtc.h"
#include "emeter-iec62056.h"

iec62056_46_link_t links[MAX_UART_PORT];

async_hdlc_rx_t rx[MAX_UART_PORT];
async_hdlc_tx_t tx[MAX_UART_PORT];
static struct msg_info_s msg_info[MAX_UART_PORT];

const uint16_t chunk_size = CHUNK_SIZE;

void exchange_date_time(iec62056_46_link_t *link, void *data, int direction)
{
    uint8_t *x;

    if (direction == ATTR_READ)
    {
        x = (uint8_t *) data;
        x[0] = 12;
        unpack16(&x[1], rtc.year);
        x[3] = rtc.month;
        x[4] = rtc.day;
        x[5] = weekday(&rtc);
        x[6] = rtc.hour;
        x[7] = rtc.minute;
        x[8] = rtc.second;
        x[9] = 0xFF;
        unpack16(&x[10], 120);
        x[12] = 0x00;
    }
    else
    {
        x = (uint8_t *) data;
        rtc.year = pack16(&x[1]);
        rtc.month = x[3];
        rtc.day = x[4];
        rtc.hour = x[6];
        rtc.minute = x[7];
        rtc.second = x[8];
    }
}
/*- End of function --------------------------------------------------------*/

void comms_setup(void)
{
}
/*- End of function --------------------------------------------------------*/

void iec62056_init(void)
{
    int i;

    //aes_encrypt(challenge_s2c, aes_key);
    /* Configure all available USCI UART ports as DLMS ports */
    for (i = 0;  i < MAX_UART_PORT;  i++)
    {
        serial_configure(i, 1, BIT_RATE);
        serial_configure_timeouts(i, TIMEOUT_3_INTRA_FRAME, TIMEOUT_2_INACTIVITY);

        iec62056_46_link_init(&links[i], i, chosen_local_msap);
        hdlc_async_rx_init(&rx[i]);
        links[i].rx_frame_pending = 0;
        links[i].msg_info = &msg_info[i];
    }
}
/*- End of function --------------------------------------------------------*/

void iec62056_service(void)
{
    int i;

    for (i = 0;  i < MAX_UART_PORT;  i++)
    {
        if (links[i].rx_frame_pending)
        {
            process_rx_frame(&links[i], &rx[i]);
            rx[i].state = 0;
            links[i].rx_frame_pending = 0;
        }
    }
}
/*- End of function --------------------------------------------------------*/

int dl_connect_indication(iec62056_46_link_t *linkp)
{
    uint16_t i;

#if 0
    if (linkp->port == PREFERRED_PORT)
    { 
        for (i = 0;  i < MAX_SERIAL_PORT;  i++)
        { 
            if (i == linkp->port)
                continue;
            if (links[i].configured == true)
            {
                links[i].disconnected = true;
                links[i].configured = false;
            }
        }
        return DL_CONNECT_RESPONSE_OK;
    }
    else
#endif
    {
        for (i = 0;  i < MAX_UART_PORT;  i++)
        { 
            if (i == linkp->port)
                continue;
            if (links[i].configured == true)
            {
                linkp->disconnected = true;
                linkp->configured = false;
                return DL_CONNECT_RESPONSE_NOK;  
            }
        }
        return DL_CONNECT_RESPONSE_OK;  
    }
}
/*- End of function --------------------------------------------------------*/

void dl_connect_confirm(iec62056_46_link_t *link, int response)
{
}
/*- End of function --------------------------------------------------------*/

int dl_disconnect_indication(iec62056_46_link_t *link, int reason)
{
    return DL_DISCONNECT_RESPONSE_OK;
}
/*- End of function --------------------------------------------------------*/
/*- End of file ------------------------------------------------------------*/
