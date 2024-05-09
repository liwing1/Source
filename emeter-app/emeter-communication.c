/*******************************************************************************
 *  emeter-communication.c -
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
#include <stdlib.h>
#include <stdbool.h>
#if defined(__MSP430__)
#include <msp430.h>
#endif

#include "emeter-template.h"

#include <emeter-toolkit.h>
#include <emeter-metrology.h>

#include "emeter-communication.h"
#include "emeter-dlt645.h"

/* We either start with a simple MAX_UART_PORT value, and map this to the
   individual enabled UARTs, or we start with the individual enabled UART
   defines, and work out the value of MAX_UART_PORT. */
#if defined(MAX_UART_PORT)
    #if MAX_UART_PORT >= 1
        #define UART_0_SUPPORT
    #endif
    #if MAX_UART_PORT >= 2
        #define UART_1_SUPPORT
    #endif
    #if MAX_UART_PORT >= 3
        #define UART_2_SUPPORT
    #endif
    #if MAX_UART_PORT >= 4
        #define UART_3_SUPPORT
    #endif
#endif

#if defined(__MSP430__)  &&  defined(IEC62056_21_SUPPORT)
uint8_t iec62056_21_address[] = "Node 1";
uint8_t iec62056_21_address_len = 6;
#endif

#if defined(UART_0_DMA_SUPPORT)
uint8_t dma_rx_buffer0[128];
#endif
#if defined(UART_1_DMA_SUPPORT)
uint8_t dma_rx_buffer1[128];
#endif

struct uart_port_s ports[MAX_UART_PORT];

#if defined(__MSP430_HAS_UART0__)  ||  defined(__MSP430_HAS_EUSCI_A0__)
/* Dither/modulation patterns to get close to an exact bit rate */
static const uint8_t mod_patterns[] =
{
    0x00, 0x01, 0x11, 0x51, 0x55, 0x5D, 0xDD, 0xDF
};
#endif

int serial_configure(int port, int mode, uint32_t bit_rate)
{
    int32_t bitrate_divider;
    uint16_t ctl0;
    uint16_t ctl1;
#if !defined(__MSP430_HAS_EUSCI_A0__)
    /* If the device has EUSCI ports the bit rate control register can be accessed as
       a single 16 bit operation. The other serial modules need to be accessed with two
       8 bit operations. */
    uint8_t br0;
    uint8_t br1;
#endif
    uint8_t mctl;

#if !defined(COMMON_RX_TX_BUFFER_SUPPORT)
    ports[port].tx_msg.len = 0;
    ports[port].tx_msg.ptr = 0;
#endif
    ports[port].rx_msg.ptr = 0;
    ports[port].rx_frame_pending = false;
    ports[port].tx_in_progress = false;
    //ports[port].next_inactivity_timeout = 0;
    //ports[port].next_octet_timeout = 0;

    /* Set some defaults */
    //serial_configure_timeouts(port, 1, 1);

    /* We only need two modes for the serial characters in the work we have done to date.
       Just support those for the time being */
    if (mode == 0)
    {
        /* 7-bit character, even parity */
#if defined(__MSP430_HAS_UART0__)
        ctl0 = PENA | PEV | SWRST;
#else
        ctl0 = UC7BIT | UCPEN;
#endif
    }
    else
    {
        /* 8-bit character, no parity */
#if defined(__MSP430_HAS_UART0__)
        ctl0 = UCPEN | UCPAR | CHAR | SWRST;
#else
        ctl0 = 0;
#endif
    }
    /* Use ACLK for slow bit rates. Use SMCLK for higher bit rates */
    if (bit_rate <= 4800L)
    {
        bitrate_divider = (32768L*16L)/bit_rate;
#if defined(__MSP430_HAS_UART0__)
        ctl1 = SSEL0;               /* ACLK */
#else
        ctl1 = UCSSEL_1 | UCSWRST;  /* ACLK */
#endif
    }
    else
    {
#if defined(__MSP430F6736__)  || defined(__MSP430F67641__)  ||  defined(__MSP430F6749__)  ||  defined(__MSP430F67491__)  ||  defined(__MSP430F6779__)  ||  defined(__MSP430F67791__)
        /* 24MHz clock */
        bitrate_divider = (32768L*768L*16L)/bit_rate;
#elif defined(__MSP430F4794__)  ||  defined(__MSP430F47197__)
        /* 16MHz clock */
        bitrate_divider = (32768L*512L*16L)/bit_rate;
#elif defined(__MSP430AFE251__)  ||  defined(__MSP430AFE252__)  ||  defined(__MSP430AFE253__)
        /* Exactly 8MHz clock */
        bitrate_divider = (8000000L*16L)/bit_rate;
#else
        /* 8MHz clock */
        bitrate_divider = (32768L*256L*16L)/bit_rate; // -> LI: REAL = 24MHZ
        // bitrate_divider = (32768L*768L*16L)/bit_rate;
#endif
#if defined(__MSP430_HAS_UART0__)
        ctl1 = SSEL1;               /* SMCLK */
#else
        ctl1 = UCSSEL_2 | UCSWRST;  /* SMCLK */
#endif
    }
    bitrate_divider++;
#if defined(__MSP430_HAS_EUSCI_A0__)  ||  defined(__MSP430_HAS_UART0__)
    mctl = (bitrate_divider & 0x0E) >> 1;
    mctl = mod_patterns[mctl];
#else
    mctl = bitrate_divider & 0x0E;
#endif
    bitrate_divider >>= 4;
#if !defined(__MSP430_HAS_EUSCI_A0__)
    br1 = bitrate_divider >> 8;
    br0 = bitrate_divider;
#endif

    switch (port)
    {
#if defined(UART_0_SUPPORT)
    case 0:
    #if defined(__MSP430_HAS_USCI_AB0__)  ||  defined(__MSP430_HAS_USCI_A0__)  ||  defined(__MSP430_HAS_EUSCI_A0__)
        /* Configure the port with the reset bit held high */
        UCA0CTL1 |= UCSWRST;
        //UCA0CTL0 = UCPEN | UCPAR | ctl0;
        UCA0CTL0 = 0xC0 | ctl0;
        UCA0CTL1 = ctl1;
        UCA0CTLW1 = 0x03;
        #if defined(__MSP430_HAS_EUSCI_A0__)
        UCA0BRW = bitrate_divider;
        UCA0MCTLW = ((uint16_t) mctl << 8);
        #else
        UCA0BR1 = br1;
        UCA0BR0 = br0;
        UCA0MCTL = mctl;
        UCA0STAT = 0;
        #endif
        UCA0TXBUF = 0;
        UCA0CTL1 &= ~UCSWRST;
        #if defined(UART_0_DMA_SUPPORT)
        //DMACTL0 = DMA1TSEL_3 | DMA2TSEL_4;
        #endif
        #if defined(__MSP430_HAS_USCI_AB0__)
        UC0IE |= UCA0RXIE;
        #else
        UCA0IE |= UCRXIE;
        #endif
    #elif defined(__MSP430_HAS_UART0__)
        UCTL0 = ctl0;
        UTCTL0 = ctl1;
        UBR10 = br1;
        UBR00 = br0;
        UMCTL0 = mctl;
        UCTL0 &= ~SWRST;
        /* Enable USART0 TXD/RXD */
        U0ME |= (UTXE0 | URXE0);
        U0IE |= URXIE0;
        /* If you do not initially kick the Tx port the TXEPT bit is not set. */
        TXBUF0 = 0;
    #endif
        // LI:
        if (bit_rate == 19200)
        {
          UCA0BRW = 0x0051;
          UCA0MCTLW = 0xBBE1;
        }
        else //9600
        {
          UCA0BRW = 0x00A3;
          UCA0MCTLW = 0x55D1;
        }
        UCA0IE = 0X09;
        return 0;
#endif
#if defined(UART_1_SUPPORT)
    case 1:
    #if defined(__MSP430_HAS_USCI_AB1__)  ||  defined(__MSP430_HAS_USCI_A1__)  ||  defined(__MSP430_HAS_EUSCI_A1__)
        /* Configure the port with the reset bit held high */
        UCA1CTL1 |= UCSWRST;
        //UCA1CTL0 = UCPEN | UCPAR | ctl0;
        UCA1CTL0 = 0xC0 | ctl0;
        UCA1CTL1 = ctl1;
        UCA1CTLW1 = 0x03;
        #if defined(__MSP430_HAS_EUSCI_A1__)
        UCA1BRW = bitrate_divider;
        UCA1MCTLW = ((uint16_t) mctl << 8);
        #else
        UCA1BR1 = br1;
        UCA1BR0 = br0;
        UCA1MCTL = mctl;
        UCA1STAT = 0;
        #endif
        UCA1TXBUF = 0;
        UCA1CTL1 &= ~UCSWRST;
        #if defined(UART_1_DMA_SUPPORT)
        DMACTL0 = DMA1TSEL_9 | DMA2TSEL_10;
        #endif
        #if defined(__MSP430_HAS_USCI_AB1__)
        UC1IE |= UCA1RXIE;
        #else
        UCA1IE |= UCRXIE;
        #endif
    #elif defined(__MSP430_HAS_UART1__)
        UCTL1 = ctl0;
        UTCTL1 = ctl1;
        UBR11 = br1;
        UBR01 = br0;
        UMCTL1 = mctl;
        UCTL1 &= ~SWRST;
        /* Enable USART1 TXD/RXD */
        ME2 |= (UTXE1 | URXE1);
        IE2 |= URXIE1;
        /* If you do not initially kick the Tx port, the TXEPT bit is not set. */
        TXBUF1 = 0;
    #endif
        // LI:
        if (bit_rate == 19200)
        {
          UCA0BRW = 0x0051;
          UCA0MCTLW = 0xBBE1;
        }
        else //9600
        {
          UCA0BRW = 0x00A3;
          UCA0MCTLW = 0x55D1;
        }
        UCA1IE = 0X09;
        return 0;
#endif
#if defined(UART_2_SUPPORT)
    case 2:
    #if defined(__MSP430_HAS_USCI_AB2__)  ||  defined(__MSP430_HAS_USCI_A2__)  ||  defined(__MSP430_HAS_EUSCI_A2__)
        /* Configure the port with the reset bit held high */
        UCA2CTL1 |= UCSWRST;
        UCA2CTL0 = ctl0;
        UCA2CTL1 = ctl1;
        #if defined(__MSP430_HAS_EUSCI_A2__)
        UCA2BRW = bitrate_divider;
        UCA2MCTLW = ((uint16_t) mctl << 8);
        #else
        UCA2BR1 = br1;
        UCA2BR0 = br0;
        UCA2MCTL = mctl;
        UCA2STAT = 0;
        #endif
        UCA2TXBUF = 0;
        UCA2CTL1 &= ~UCSWRST;
        #if defined(__MSP430_HAS_USCI_AB2__)
        UC2IE |= UCA2RXIE;
        #else
        UCA2IE |= UCRXIE;
        #endif
    #endif
        return 0;
#endif
#if defined(UART_3_SUPPORT)
    case 3:
    #if defined(__MSP430_HAS_USCI_AB3__)  ||  defined(__MSP430_HAS_USCI_A3__)  ||  defined(__MSP430_HAS_EUSCI_A3__)
        /* Configure the port with the reset bit held high */
        UCA3CTL1 |= UCSWRST;
        UCA3CTL0 = ctl0;
        UCA3CTL1 = ctl1;
        #if defined(__MSP430_HAS_EUSCI_A3__)
        UCA3BRW = bitrate_divider;
        UCA3MCTLW = ((uint16_t) mctl << 8);
        #else
        UCA3BR1 = br1;
        UCA3BR0 = br0;
        UCA3MCTL = mctl;
        UCA3STAT = 0;
        #endif
        UCA3TXBUF = 0;
        UCA3CTL1 &= ~UCSWRST;
        #if defined(__MSP430_HAS_USCI_AB3__)
        UC3IE |= UCA3RXIE;
        #else
        UCA3IE |= UCRXIE;
        #endif
    #endif
        return 0;
#endif
    }
    return -1;
}
/*- End of function --------------------------------------------------------*/

void serial_write(int port, const uint8_t buf[], int len)
{
    ports[port].tx_msg.ptr = 0;
    ports[port].tx_msg.len = len;
    switch (port)
    {
    #if defined(UART_0_SUPPORT)
    case 0:
        //#if defined(UART_0_DMA_SUPPORT)
        //__data20_write_long((uint32_t) &DMA2SA, (uint32_t) buf);
        //__data20_write_long((uint32_t) &DMA2DA, (uint32_t) &UCA0TXBUF);
        //DMA2SZ = len;
        ///* Enable, source address incremented, single transfer. */
        //DMA2CTL = DMADT_0 | DMASRCINCR_3 | DMASRCBYTE | DMADSTBYTE | DMAEN;
        ///* Kick things, so the DMA starts rolling */
        //UC0IFG &= ~UCA0TXIFG;
        //UC0IFG |= UCA0TXIFG;
        //#elif defined(__MSP430_HAS_UART0__)
        //U0IE |= UTXIE0;
        //#elif defined(__MSP430_HAS_USCI_AB0__)
        //UC0IE |= UCA0TXIE;
        //#else
        //UCA0IE |= UCTXIE;
        //#endif
        
        for(int idx = 0; idx < len; idx++) {
          while(!(UCA0IFG & UCTXIFG));  
          UCA0TXBUF = buf[idx];
        }
        break;
    #endif
    #if defined(UART_1_SUPPORT)
    case 1:
        //#if defined(UART_1_DMA_SUPPORT)
        //__data20_write_long((uint32_t) &DMA2SA, (uint32_t) buf);
        //__data20_write_long((uint32_t) &DMA2DA, (uint32_t) &UCA1TXBUF);
        //DMA2SZ = len;
        /* Enable, source address incremented, single transfer. */
        //DMA2CTL = DMADT_0 | DMASRCINCR_3 | DMASRCBYTE | DMADSTBYTE | DMAEN;
        /* Kick things, so the DMA starts rolling */
        //UCA1IFG &= ~UCTXIFG;
        //UCA1IFG |= UCTXIFG;
        //#elif defined(__MSP430_HAS_UART1__)
        //U1IE |= UTXIE1;
        //#elif defined(__MSP430_HAS_USCI_AB1__)
        //UC1IE |= UCA1TXIE;
        //#else
        //UCA1IE |= UCTXIE;
        //#endif
      
      // LI: uart new implementation
        for(int idx = 0; idx < len; idx++) {
          while(!(UCA1IFG & UCTXIFG));  
          UCA1TXBUF = buf[idx];
        }
        break;
    #endif
    #if defined(UART_2_SUPPORT)
    case 2:
        #if defined(__MSP430_HAS_USCI_AB2__)
        UC2IE |= UCA2TXIE;
        #else
        UCA2IE |= UCTXIE;
        #endif
        break;
    #endif
    #if defined(UART_3_SUPPORT)
    case 3:
        #if defined(__MSP430_HAS_USCI_AB3__)
        UC3IE |= UCA3TXIE;
        #else
        UCA3IE |= UCTXIE;
        #endif
        break;
    #endif
    }
}
/*- End of function --------------------------------------------------------*/

void RS485_sendBuf(int port, uint8_t* buf, uint16_t len){
    ports[port].tx_in_progress = false;

    P4OUT |= BIT0;
    __delay_cycles(500);
    
    serial_write(port, buf, len);
    
    while(!(UCA0IFG & UCTXIFG));
    
    ports[port].tx_in_progress = true;

}

void uart_rx_core(int port, uint8_t rx_char)
{
    #if defined(UART_0_IEC62056_21_SUPPORT)
    if (inter_char_timeout == 0)
        iec62056_21_rx_restart(port);
    inter_char_timeout = SAMPLES_PER_10_SECONDS/200;
    if (!ports[port].tx_in_progress)
        iec62056_21_rx_byte(rx_char & 0x7F);
    #endif
    #if defined(COMMON_RX_TX_BUFFER_SUPPORT)
    dlt645_rx_byte(port, rx_char);
    #else
    dlt645_rx_byte(port, rx_char);
    #endif
}
/*- End of function --------------------------------------------------------*/

uint16_t uart_tx_core(int port)
{
    uint8_t tx;

    if (!ports[port].tx_in_progress)
        return 0x8000;
    
    
    tx = ports[port].tx_msg.buf.uint8[ports[port].tx_msg.ptr++];
    if (ports[port].tx_msg.ptr >= ports[port].tx_msg.len)
    {
#if defined(COMMON_RX_TX_BUFFER_SUPPORT)
        /* Let receive mode have access to the buffer */
        ports[port].rx_msg.ptr = 0;
        ports[port].rx_msg.len = 0;
#else
        ports[port].tx_msg.ptr = 0;
        ports[port].tx_msg.len = 0;
#endif
        //ports[port].tx_in_progress = false;
        //serial_tx_complete(port);
        return 0x8000 | tx;
    }
   
    return tx;
    
}
/*- End of function --------------------------------------------------------*/

/* We need a separate set of interrupt routines for the 4xx and 5xx devices. Although the body of the
   USCI module is functionally similar in the 4xx and 5xx devices, the interrupt scheme is quite different. */
#if defined(UART_0_SUPPORT)
    #if defined(__MSP430_HAS_UART0__)
ISR(USART0RX, USART0_rx_isr)
{
    uart_rx_core(0, RXBUF0);
}
/*- End of function --------------------------------------------------------*/

ISR(USART0TX, USART0_tx_isr)
{
    uint16_t tx;

    tx = uart_tx_core(0);
    TXBUF0 = tx & 0xFF;
    if (tx & 0x8000)
        U0IE &= ~UTXIE0;
}
/*- End of function --------------------------------------------------------*/
    #elif defined(__MSP430_HAS_USCI_AB0__)
ISR(USCIAB0RX, USCI_AB0_rx_isr)
{
    uart_rx_core(0, UCA0RXBUF);
}
/*- End of function --------------------------------------------------------*/

        #if !defined(UART_0_DMA_SUPPORT)
ISR(USCIAB0TX, USCI_AB0_tx_isr)
{
    uint16_t tx;

    tx = uart_tx_core(0);
    UCA0TXBUF = tx & 0xFF;
    if (tx & 0x8000)
        UC0IE &= ~UCA0TXIE;
}
/*- End of function --------------------------------------------------------*/
        #endif
    #elif defined(__MSP430_HAS_USCI_A0__)
ISR(USCI_A0, USCI_A0_isr)
{
    uint16_t tx;

    switch (__even_in_range(UCA0IV, USCI_UART_UCTXIFG))
    {
    case USCI_NONE:
        break;
    case USCI_UART_UCRXIFG:
        uart_rx_core(0, UCA0RXBUF);
        break;
    case USCI_UART_UCTXIFG:
        tx = uart_tx_core(0);
        UCA0TXBUF = tx & 0xFF;
        if (tx & 0x8000)
            UCA0IE &= ~UCTXIE;
        break;
    }
}
/*- End of function --------------------------------------------------------*/
    #elif defined(__MSP430_HAS_EUSCI_A0__)
ISR(USCI_A0, USCI_A0_isr)
{
    switch (__even_in_range(UCA0IV, USCI_UART_UCTXCPTIFG))
    {
    case USCI_NONE:
        break;
    case USCI_UART_UCRXIFG:
        ports[0].rx_msg.buf.uint8[ports[0].rx_msg.ptr++] = UCA0RXBUF;
        
        if (ports[0].rx_msg.buf.uint8[0] == 0x68)
        {
          while(!(UCA0IFG & UCRXIFG));
          ports[0].rx_msg.buf.uint8[ports[0].rx_msg.ptr++] = UCA0RXBUF;
            
          switch (ports[0].rx_msg.buf.uint8[1])
          {
          // Espera 8bytes
          case 0x02: //Read Input Status
          case 0x03: //Read Holding Register
          case 0x04: //Read Input Register
          case 0x05: //Force Single Coil
          case 0x06: //Preset Single Register
            ports[0].rx_msg.len = 8;
            break;
            
          //Espera 4bytes
          case 0x07: //Read Exception Status
          case 0x11: //Report Slave ID
            ports[0].rx_msg.len = 4;
            break;
            
          //Preset Multiple Register (nbytes)
          case 0x10:
            // Wait rx nbytes
            if(ports[0].rx_msg.ptr < 7)
              return;
              
            ports[0].rx_msg.len = ports[0].rx_msg.buf.uint8[6] + 9;
            break;
          
          default: 
            // Got invalid function
            ports[0].rx_msg.ptr = 0;
            return;
          }
          
          while(ports[0].rx_msg.ptr < ports[0].rx_msg.len)
          {
            while(!(UCA0IFG & UCRXIFG));
            ports[0].rx_msg.buf.uint8[ports[0].rx_msg.ptr++] = UCA0RXBUF;
          }
          
          ports[0].rx_frame_pending = true;
        }
        
        ports[0].rx_msg.ptr = 0;
        
        //uart_rx_core(1, UCA0RXBUF);
        break;
    case USCI_UART_UCTXIFG:
        break;
    case USCI_UART_UCSTTIFG:
        break;
    case USCI_UART_UCTXCPTIFG:
        if(ports[0].tx_in_progress)
        {
          ports[0].tx_in_progress = false;
          P4OUT &= ~BIT0;
        }
        break;
    }
}
/*- End of function --------------------------------------------------------*/
    #else
        #error Device does not have a UART port 0
    #endif
#endif

#if defined(UART_1_SUPPORT)
    #if defined(__MSP430_HAS_UART1__)
ISR(USART1RX, USART1_rx_isr)
{
    uart_rx_core(1, RXBUF1);
}
/*- End of function --------------------------------------------------------*/

ISR(USART1TX, USART1_tx_isr)
{
    uint16_t tx;

    tx = uart_tx_core(1);
    TXBUF1 = tx & 0xFF;
    if (tx & 0x8000)
        U1IE &= ~UTXIE1;
}
/*- End of function --------------------------------------------------------*/
    #elif defined(__MSP430_HAS_USCI_AB1__)
ISR(USCIAB1RX, USCI_AB1_rx_isr)
{
    uart_rx_core(1, UCA1RXBUF);
}
/*- End of function --------------------------------------------------------*/

        #if !defined(UART_1_DMA_SUPPORT)
ISR(USCIAB1TX, USCI_AB1_tx_isr)
{
    uint16_t tx;

    tx = uart_tx_core(1);
    UCA1TXBUF = tx & 0xFF;
    if (tx & 0x8000)
        UC1IE &= ~UCA1TXIE;
}
/*- End of function --------------------------------------------------------*/
        #endif
    #elif defined(__MSP430_HAS_USCI_A1__)
ISR(USCI_A1, USCI_A1_isr)
{
    uint16_t tx;

    switch (__even_in_range(UCA1IV, USCI_UART_UCRXIFG))
    {
    case USCI_NONE:
        break;
    case USCI_UART_UCRXIFG:
        uart_rx_core(1, UCA1RXBUF);
        break;
    case USCI_UART_UCTXIFG:
        tx = uart_tx_core(1);
        UCA1TXBUF = tx & 0xFF;
        if (tx & 0x8000)
            UCA1IE &= ~UCTXIE;
        break;
    }
}
/*- End of function --------------------------------------------------------*/
    #elif defined(__MSP430_HAS_EUSCI_A1__)
ISR(USCI_A1, USCI_A1_isr)
{
    switch (__even_in_range(UCA1IV, USCI_UART_UCTXCPTIFG))
    {
    case USCI_NONE:
        break;
    case USCI_UART_UCRXIFG:
        ports[1].rx_msg.buf.uint8[ports[1].rx_msg.ptr++] = UCA1RXBUF;
        
        if (ports[1].rx_msg.buf.uint8[0] == 0x68)
        {
          while(!(UCA1IFG & UCRXIFG));
          ports[1].rx_msg.buf.uint8[ports[1].rx_msg.ptr++] = UCA1RXBUF;
            
          switch (ports[1].rx_msg.buf.uint8[1])
          {
          // Espera 8bytes
          case 0x02: //Read Input Status
          case 0x03: //Read Holding Register
          case 0x04: //Read Input Register
          case 0x05: //Force Single Coil
          case 0x06: //Preset Single Register
            ports[1].rx_msg.len = 8;
            break;
            
          //Espera 4bytes
          case 0x07: //Read Exception Status
          case 0x11: //Report Slave ID
            ports[1].rx_msg.len = 4;
            break;
            
          //Preset Multiple Register (nbytes)
          case 0x10:
            // Wait rx nbytes
            if(ports[1].rx_msg.ptr < 7)
              return;
              
            ports[1].rx_msg.len = ports[1].rx_msg.buf.uint8[6] + 9;
            break;
          
          default: 
            // Got invalid function
            ports[1].rx_msg.ptr = 0;
            return;
          }
          
          while(ports[1].rx_msg.ptr < ports[1].rx_msg.len)
          {
            while(!(UCA1IFG & UCRXIFG));
            ports[1].rx_msg.buf.uint8[ports[1].rx_msg.ptr++] = UCA1RXBUF;
          }
          
          ports[1].rx_frame_pending = true;
        }
        
        ports[1].rx_msg.ptr = 0;
        
        //uart_rx_core(1, UCA1RXBUF);
        break;
    case USCI_UART_UCTXIFG:
        //tx = uart_tx_core(1);
        //UCA1TXBUF = tx & 0xFF;
        //if (tx & 0x8000)
        //   UCA1IE &= ~UCTXIE;
        break;
    case USCI_UART_UCSTTIFG:
        break;
    case USCI_UART_UCTXCPTIFG:
        break;
    }
}
/*- End of function --------------------------------------------------------*/
    #else
        #error Device does not have a UART port 1
    #endif
#endif

#if defined(UART_2_SUPPORT)
    #if defined(__MSP430_HAS_USCI_AB2__)
ISR(USCIAB2RX, USCI_AB2_rx_isr)
{
    uart_rx_core(2, UCA2RXBUF);
}
/*- End of function --------------------------------------------------------*/

        #if !defined(UART_2_DMA_SUPPORT)
ISR(USCIAB2TX, USCI_AB2_tx_isr)
{
    uint16_t tx;

    tx = uart_tx_core(2);
    UCA2TXBUF = tx & 0xFF;
    if (tx & 0x8000)
        UC2IE &= ~UCA2TXIE;
}
/*- End of function --------------------------------------------------------*/
        #endif
    #elif defined(__MSP430_HAS_USCI_A2__)
ISR(USCI_A2, USCI_A2_isr)
{
    uint16_t tx;

    now = current_time();
    switch (__even_in_range(UCA2IV, USCI_UART_UCTXIFG))
    {
    case USCI_NONE:
        break;
    case USCI_UART_UCRXIFG:
        uart_rx_core(2, UCA2RXBUF);
        break;
    case USCI_UART_UCTXIFG:
        tx = uart_tx_core(2);
        UCA2TXBUF = tx & 0xFF;
        if (tx & 0x8000)
            UCA2IE &= ~UCTXIE;
        break;
    default:
        break;
    }
}
/*- End of function --------------------------------------------------------*/
    #elif defined(__MSP430_HAS_EUSCI_A2__)
ISR(USCI_A2, USCI_A2_isr)
{
    uint16_t tx;

    switch (__even_in_range(UCA2IV, USCI_UART_UCTXCPTIFG))
    {
    case USCI_NONE:
        break;
    case USCI_UART_UCRXIFG:
        uart_rx_core(2, UCA2RXBUF);
        break;
    case USCI_UART_UCTXIFG:
        tx = uart_tx_core(2);
        UCA2TXBUF = tx & 0xFF;
        if (tx & 0x8000)
            UCA2IE &= ~UCTXIE;
        break;
    case USCI_UART_UCSTTIFG:
        break;
    case USCI_UART_UCTXCPTIFG:
        break;
    }
}
/*- End of function --------------------------------------------------------*/
    #else
        #error Device does not have a UART port 2
    #endif
#endif

#if defined(UART_3_SUPPORT)
    #if defined(__MSP430_HAS_USCI_AB3__)
ISR(USCIAB3RX, USCI_AB3_rx_isr)
{
    uart_rx_core(3, UCA3RXBUF);
}
/*- End of function --------------------------------------------------------*/

        #if !defined(UART_3_DMA_SUPPORT)
ISR(USCIAB3TX, USCI_AB3_tx_isr)
{
    uint16_t tx;

    tx = uart_tx_core(3);
    UCA3TXBUF = tx & 0xFF;
    if (tx & 0x8000)
        UC3IE &= ~UCA3TXIE;
}
/*- End of function --------------------------------------------------------*/
        #endif
    #elif defined(__MSP430_HAS_USCI_A3__)
ISR(USCI_A3, USCI_A3_isr)
{
    uint16_t tx;

    now = current_time();
    switch (__even_in_range(UCA3IV, USCI_UART_UCTXIFG))
    {
    case USCI_NONE:
        break;
    case USCI_UART_UCRXIFG:
        uart_rx_core(3, UCA3RXBUF);
        break;
    case USCI_UART_UCTXIFG:
        tx = uart_tx_core(3);
        UCA3TXBUF = tx & 0xFF;
        if (tx & 0x8000)
            UCA3IE &= ~UCTXIE;
        break;
    default:
        break;
    }
}
/*- End of function --------------------------------------------------------*/
    #elif defined(__MSP430_HAS_EUSCI_A3__)
ISR(USCI_A3, USCI_A3_isr)
{
    uint16_t tx;

    switch (__even_in_range(UCA3IV, USCI_UART_UCTXCPTIFG))
    {
    case USCI_NONE:
        break;
    case USCI_UART_UCRXIFG:
        uart_rx_core(3, UCA3RXBUF);
        break;
    case USCI_UART_UCTXIFG:
        tx = uart_tx_core(3);
        UCA3TXBUF = tx & 0xFF;
        if (tx & 0x8000)
            UCA3IE &= ~UCTXIE;
        break;
    case USCI_UART_UCSTTIFG:
        break;
    case USCI_UART_UCTXCPTIFG:
        break;
    }
}
/*- End of function --------------------------------------------------------*/
    #else
        #error Device does not have a UART port 3
    #endif
#endif

#if defined(IHD430_SUPPORT)
void send_reading_to_CC2530_for_IHD430 (power_t total_active_power)
{   
     int i;
     static unsigned char RF_Tx[17]={0xFE,0x0C,0x29,0x00,0x09,0x00,0x00,0x00,0x00,0x00,0x05,0x01};
     if(total_active_power < 0) total_active_power*=-1;
     RF_Tx[12]=total_active_power;
     RF_Tx[13]= (total_active_power & 0xFF00)>>8;
     RF_Tx[14]=(total_active_power & 0xFF0000)>>16;
     RF_Tx[15]=(total_active_power & 0xFF000000)>>24; 
     RF_Tx[16] =0x28 ^ RF_Tx[12] ^ RF_Tx[13] ^ RF_Tx[14] ^ RF_Tx[15];
     for (i=0; i<17; i++)
     {
        UCA2TXBUF=RF_Tx[i];
        while(!(UCA2IFG & UCTXIFG));
     }               
}
void IHD430_UART_configure()
{
    UCA2CTL1 |= UCSWRST; 
    UCA2CTL1 |= UCSSEL_2;                     // SMCLK
    UCA2BRW = 0x000D;
    UCA2MCTLW = 0x55A1;                         
    UCA2CTL1 &= ~UCSWRST; 
}
#endif
  

//#if defined(UART_0_DMA_SUPPORT)  ||  defined(UART_1_DMA_SUPPORT)
//ISR(DMA, dma_interrupt)
//{
//    switch (__even_in_range(DMAIV, DMAIV_DMA2IFG))
//    {
//    case DMAIV_DMA0IFG:
//        DMA0CTL &= ~ DMAIFG;
//        break;
//    case DMAIV_DMA1IFG:
//        DMA1CTL &= ~ DMAIFG;
//        break;
//    case DMAIV_DMA2IFG:
//        DMA2CTL &= ~ DMAIFG;
//    //#if defined(UART_0_DMA_SUPPORT)
//    //    uart_tx_dma_core(0);
//    //#endif
//    //#if defined(UART_1_DMA_SUPPORT)
//    //    uart_tx_dma_core(1);
//    //#endif
//        break;
//    }
//}
/*- End of function --------------------------------------------------------*/
//#endif
/*- End of file ------------------------------------------------------------*/
