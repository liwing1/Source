/*******************************************************************************
 *  emeter-communication.h -
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

/* We either start with a simple MAX_UART_PORT value, and map this to the
   individual enabled UARTs, or we start with the individual enabled UART
   defines, and work out the value of MAX_UART_PORT. */
#if !defined(MAX_UART_PORT)
    #if defined(UART_3_SUPPORT)
        #define MAX_UART_PORT 4
    #elif defined(UART_2_SUPPORT)
        #define MAX_UART_PORT 3
    #elif defined(UART_1_SUPPORT)
        #define MAX_UART_PORT 2
    #elif defined(UART_0_SUPPORT)
        #define MAX_UART_PORT 1
    #endif
#endif

#define MAX_SERIAL_MESSAGE_LEN      82

typedef union
{
    uint8_t uint8[MAX_SERIAL_MESSAGE_LEN];
    uint16_t uint16[MAX_SERIAL_MESSAGE_LEN >> 1];
} serial_msg_t;

/* Incoming or outgoing serial message buffer */
typedef struct
{
    serial_msg_t buf;
    uint8_t ptr;
    uint8_t len;
    unsigned long inter_char_timeout;
} serial_msg_buf_t;

struct uart_port_s
{
#if defined(COMMON_RX_TX_BUFFER_SUPPORT)
    union
    {
        serial_msg_buf_t tx_msg;
        serial_msg_buf_t rx_msg;
    };
#else
    serial_msg_buf_t tx_msg;
    serial_msg_buf_t rx_msg;
#endif
    int8_t tx_in_progress;
    int8_t rx_frame_pending;
};

extern struct uart_port_s ports[MAX_UART_PORT];


int serial_configure(int port, int mode, uint32_t bit_rate);



void serial_write(int port, const uint8_t buf[], int len);
void RS485_sendBuf(int port, uint8_t* buf, uint16_t len);

int is_calibration_enabled(void);

#if defined(IHD430_SUPPORT)
void IHD430_UART_configure(void);
void send_reading_to_CC2530_for_IHD430 (power_t total_active_power);
#endif
