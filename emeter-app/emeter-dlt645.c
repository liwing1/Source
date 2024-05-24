/*******************************************************************************
 *  emeter-dlt645.c -
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
#include <string.h>
#include <stdbool.h>
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

#include "emeter-template.h"

#include <emeter-toolkit.h>
#include <emeter-metrology.h>

#include "emeter-main.h"
#include "emeter-app.h"
#include "emeter-lcd.h"
#include "emeter-rtc.h"
#if defined(MULTI_RATE_SUPPORT)
#include "emeter-multirate.h"
#endif
#include "emeter-communication.h"
#include "emeter-dlt645.h"
#include "dlt645-decs.h"

#include "metrology-calibration.h"

#if !defined(NULL)
#define NULL    (void *) 0
#endif

uint16_t CalculateCRC(uint8_t *data, uint16_t length) {
  uint16_t CRC16 = 0xffff;
  uint16_t poly =  0xA001;

  for (int i=0; i<length; i++)
  {
      CRC16 = (uint16_t)data[i] ^ CRC16;
      for (int j=0; j<8; j++)
      {
        if(CRC16 & 0x0001 == 1) {
          CRC16 >>= 1;
          CRC16 ^= poly;
        } else {
          CRC16 >>= 1;
        }
      }
  }
  
  return CRC16;
}

static uint8_t* get_ptr_from_input_register_addr(uint16_t reg_addr) 
{
  uint8_t byte_addr = 2 * reg_addr;
  if(reg_addr <= 65)
    return (&(input_registers.byte[byte_addr]));
  
  else if (reg_addr >= 200 && reg_addr <= 225)
    return (&(input_registers.byte[byte_addr - 268]));
  
  else if (reg_addr >= 300 && reg_addr <= 339)
    return (&(input_registers.byte[byte_addr - 416]));
  
  else if (reg_addr >= 1200 && reg_addr <= 1229)
    return (&(input_registers.byte[byte_addr - 2136]));
  
  else 
    return NULL;
}

uint8_t process_preset_single_reg(int port, uint16_t write_reg, uint16_t write_data)
{
  switch (write_reg)
  {
  case 0:
    holding_registers.addr[write_reg] = write_data;
    set_cfg_baud_rate(holding_registers.addr[write_reg]);
  break;

  // CONFIG VRMS SCALE
  case 1: set_V_rms_scaling(0, 0, write_data); break;
  case 2: set_V_rms_scaling(1, 0, write_data); break;
  case 3: set_V_rms_scaling(2, 0, write_data); break;
  
  // CONFIG IRMS SCALE
  case 4: set_I_rms_scaling(0, 0, write_data); break;
  case 5: set_I_rms_scaling(1, 0, write_data); break;
  case 6: set_I_rms_scaling(2, 0, write_data); break;
  
  // CONFIG P SCALE
  case 7: set_P_scaling(0, write_data); break;
  case 8: set_P_scaling(1, write_data); break;
  case 9: set_P_scaling(2, write_data); break;
  
  // CONFIG PHASE
  case 10: set_phase_corr(0, write_data); break;
  case 11: set_phase_corr(1, write_data); break;  
  case 12: set_phase_corr(2, write_data); break;
  
  default: return 0; break;
  }

  RS485_sendBuf(port, ports[port].rx_msg.buf.uint8, 8); // header + data 
  return 1;
}

uint8_t process_read_inp_reg(int port, uint16_t first_reg, uint16_t n_reg)
{
  if (n_reg > 18)
    return 0;
  
  ports[port].tx_msg.buf.uint8[0] = 0x68;
  ports[port].tx_msg.buf.uint8[1] = 0x04;
  ports[port].tx_msg.buf.uint8[2] = n_reg * 2; //byte count
  
  uint8_t* map_ptr = get_ptr_from_input_register_addr(first_reg);
  
  if(map_ptr == NULL)
    return 0;
  
  memcpy(&(ports[port].tx_msg.buf.uint8[3]), map_ptr, n_reg * 2);
    
  uint16_t crc = CalculateCRC(ports[port].tx_msg.buf.uint8, 3 + n_reg * 2);
  
  ports[port].tx_msg.buf.uint8[4 + n_reg * 2] = (uint8_t)(crc>>8);
  ports[port].tx_msg.buf.uint8[3 + n_reg * 2] = (uint8_t)(crc&0x00FF);
  
  RS485_sendBuf(port, ports[port].tx_msg.buf.uint8, 5 + n_reg * 2); // header + data
  return 1;
}

/* This routine is called regularly from the main polling loop, to check for completed incoming
   DLT-645 messages, and to service them. */
void dlt645_service(void)
{
  const int port = 0; //LI: UART A1
  
  if (ports[port].rx_frame_pending)
  {
    uint16_t crc = CalculateCRC(ports[port].rx_msg.buf.uint8, ports[port].rx_msg.len-2);
    uint16_t first_reg, n_reg;
    uint16_t write_reg, write_data;
    
    // Verifica CRC
    if (ports[port].rx_msg.buf.uint8[ports[port].rx_msg.len-2] == (uint8_t)(crc&0x00FF) &&
        ports[port].rx_msg.buf.uint8[ports[port].rx_msg.len-1] == (uint8_t)(crc>>8))
    {
      uint8_t function = ports[port].rx_msg.buf.uint8[1];
      
      switch (function)
      {
      // Espera 8bytes
      case 0x02: //Read Input Status
      case 0x03: //Read Holding Register
        break;
        
      case 0x04: //Read Input Register
        first_reg = ((uint16_t)ports[port].rx_msg.buf.uint8[2])<<8 | ports[port].rx_msg.buf.uint8[3];
        n_reg = ((uint16_t)ports[port].rx_msg.buf.uint8[4])<<8 | ports[port].rx_msg.buf.uint8[5];
        
        process_read_inp_reg(port, first_reg, n_reg);
        
        break;
        
      case 0x05: break;//Force Single Coil
      
      case 0x06: //Preset Single Register
        write_reg = ((uint16_t)ports[port].rx_msg.buf.uint8[2])<<8 | ports[port].rx_msg.buf.uint8[3];
        write_data = ((uint16_t)ports[port].rx_msg.buf.uint8[4])<<8 | ports[port].rx_msg.buf.uint8[5];

        process_preset_single_reg(port, write_reg, write_data);
        
        break;
        
      //Espera 4bytes
      case 0x07: //Read Exception Status
      case 0x11: //Report Slave ID
        break;
        
      //Preset Multiple Register (nbytes)
      case 0x10: break;

      /*Funcoes especiais*/
      /*
      case 0x42: //Config Address
      case 0x71: //Read Address
      case 0x75: //Read Partidas
      case 0x76: //Report Slave Id Kron
        break;
      */
      default: 
        // Got invalid function
        ports[port].rx_msg.ptr = 0;
        ports[port].rx_frame_pending = false;
        break;
      }
    }
    ports[port].rx_frame_pending = false;
  }
  
  // Manage timeout
  if (Contador4096 - ports[port].rx_msg.inter_char_timeout > 10000)
  {
    ports[port].rx_msg.inter_char_timeout = Contador4096;
    
    if (ports[port].rx_msg.ptr != 0)
    {
      memset(ports[port].rx_msg.buf.uint8, 0, ports[port].rx_msg.ptr);
      ports[port].rx_msg.ptr = 0;
    }
  }
}

/* This routine is called from within UART port interrupts, so it must be kept lean and mean. */
void dlt645_rx_byte(int port, uint8_t c)
{
/*
  // update timeout
  ports[port].rx_msg.inter_char_timeout = Contador4096;

  // Add new byte
  //ports[port].rx_msg.buf.uint8[ports[port].rx_msg.ptr++] = c;
  
  // Espera slave address + func
  if (ports[port].rx_msg.ptr <= 2)
  {
    if (ports[port].rx_msg.buf.uint8[0] != 0x68)
    {
      ports[port].rx_msg.ptr = 0;
      return; 
    }
  }
  else
  {
    switch (ports[port].rx_msg.buf.uint8[1])
    {
    // Espera 8bytes
    case 0x02: //Read Input Status
    case 0x03: //Read Holding Register
    case 0x04: //Read Input Register
    case 0x05: //Force Single Coil
    case 0x06: //Preset Single Register
      ports[port].rx_msg.len = 8;
      break;
      
    //Espera 4bytes
    case 0x07: //Read Exception Status
    case 0x11: //Report Slave ID
      ports[port].rx_msg.len = 4;
      break;
      
    //Preset Multiple Register (nbytes)
    case 0x10:
      // Wait rx nbytes
      if(ports[port].rx_msg.ptr < 7)
        return;
        
      ports[port].rx_msg.len = ports[port].rx_msg.buf.uint8[6] + 9;
      break;
    
    default: 
      // Got invalid function
      ports[port].rx_msg.ptr = 0;
      ports[port].rx_frame_pending = false;
      break;
    }
    
    // Espera completar frame
    if (ports[port].rx_msg.ptr >= ports[port].rx_msg.len)
    {
      ports[port].rx_msg.ptr = 0;
    }
  }
*/
}
