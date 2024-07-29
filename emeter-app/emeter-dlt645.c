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

uint16_t word1;
uint16_t word2;
int port = 0;

enum holding_register_address 
{
  HOLD_CFG_BAUD_RATE = 0,
  HOLD_CFG_ADDRESS,
  HOLD_V_A_SCALE,
  HOLD_V_B_SCALE,
  HOLD_V_C_SCALE,
  HOLD_I_A_SCALE,
  HOLD_I_B_SCALE,
  HOLD_I_C_SCALE,
  HOLD_P_A_SCALE,
  HOLD_P_B_SCALE,
  HOLD_P_C_SCALE,
  HOLD_A_PHASE,
  HOLD_B_PHASE,
  HOLD_C_PHASE,
};

enum coils
{
  COIL_RST_DEFAULT_CAL_CFG = 0,
  COIL_RST_DEVICE,
};

uint16_t CalculateCRC(uint8_t *data, uint16_t length) 
{
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

void update_holding_registers(void)
{
  const struct calibration_data_s* temp_cal_data = get_cal_info();
  const struct configuration_data_s* temp_cfg_data = get_cfg_info();
  
  holding_registers.configs.baud_rate = get_cfg_baud_rate();
  holding_registers.configs.modbus_addr = get_cfg_mb_address();
  
  holding_registers.configs.v_a_scale = temp_cal_data->phases[0].V_rms_scale_factor[0];
  holding_registers.configs.v_b_scale = temp_cal_data->phases[1].V_rms_scale_factor[0];
  holding_registers.configs.v_c_scale = temp_cal_data->phases[2].V_rms_scale_factor[0];
  
  holding_registers.configs.i_a_scale = temp_cal_data->phases[0].current[0].I_rms_scale_factor[0];
  holding_registers.configs.i_b_scale = temp_cal_data->phases[1].current[0].I_rms_scale_factor[0];
  holding_registers.configs.i_c_scale = temp_cal_data->phases[2].current[0].I_rms_scale_factor[0];
  
  holding_registers.configs.p_a_scale = temp_cal_data->phases[0].current[0].P_scale_factor;
  holding_registers.configs.p_b_scale = temp_cal_data->phases[1].current[0].P_scale_factor;
  holding_registers.configs.p_c_scale = temp_cal_data->phases[2].current[0].P_scale_factor;
  
  holding_registers.configs.a_phase  = temp_cal_data->phases[0].current[0].phase_correction;
  holding_registers.configs.b_phase  = temp_cal_data->phases[1].current[0].phase_correction;
  holding_registers.configs.c_phase  = temp_cal_data->phases[2].current[0].phase_correction;
}

static uint8_t* get_ptr_from_input_register_addr(uint16_t reg_addr) 
{
  uint16_t byte_addr = 2 * reg_addr;
  if(reg_addr <= 65)
    return (&(input_registers.byte[byte_addr]));
  
  else if (reg_addr >= 200 && reg_addr <= 225)
    return (&(input_registers.byte[byte_addr - 268]));
  
  else if (reg_addr >= 300 && reg_addr <= 339)
    return (&(input_registers.byte[byte_addr - 416]));
  
  else if (reg_addr >= 1224 && reg_addr <= 1235)
    return (&(input_registers.byte[byte_addr - 2184]));

  else if (reg_addr == 3900)
    return ((uint8_t*)&(input_registers.grandezas.status));
  
  else 
    return NULL;
}

void process_force_single_coil(void)
{
  word1 = (uint16_t)ports[port].rx_msg.buf.uint8[2];
  switch (word1)
  {
  case COIL_RST_DEFAULT_CAL_CFG:
    clear_calibration_data(); 
    write_calibration_data(CALIBRATION_DATA_DEFAULT, CONFIGURATION_DATA_DEFAULT);
  break;

  case COIL_RST_DEVICE:
    while(1); // Lock device until WDT trigger
  break;
  
  default: break;
  }
}

void process_preset_single_reg(void)
{
  word1 = ((uint16_t)ports[port].rx_msg.buf.uint8[2])<<8 | ports[port].rx_msg.buf.uint8[3];
  word2 = ((uint16_t)ports[port].rx_msg.buf.uint8[4])<<8 | ports[port].rx_msg.buf.uint8[5];

  struct calibration_data_s temp_cal_data;
  struct configuration_data_s temp_cfg_data;

  memcpy(&temp_cal_data, (void*)get_cal_info(), sizeof(temp_cal_data));
  memcpy(&temp_cfg_data, (void*)get_cfg_info(), sizeof(temp_cfg_data));

  switch (word1)
  {
  case HOLD_CFG_BAUD_RATE: temp_cfg_data.baud_rate = word2; break;
  case HOLD_CFG_ADDRESS: temp_cfg_data.mb_address = word2; break;
  
  case HOLD_V_A_SCALE: temp_cal_data.phases[0].V_rms_scale_factor[0] = word2; break;
  case HOLD_V_B_SCALE: temp_cal_data.phases[1].V_rms_scale_factor[0] = word2; break;
  case HOLD_V_C_SCALE: temp_cal_data.phases[2].V_rms_scale_factor[0] = word2; break;
  
  case HOLD_I_A_SCALE: temp_cal_data.phases[0].current[0].I_rms_scale_factor[0] = word2; break;
  case HOLD_I_B_SCALE: temp_cal_data.phases[1].current[0].I_rms_scale_factor[0] = word2; break;
  case HOLD_I_C_SCALE: temp_cal_data.phases[2].current[0].I_rms_scale_factor[0] = word2; break;
  
  case HOLD_P_A_SCALE: temp_cal_data.phases[0].current[0].P_scale_factor = word2; break;
  case HOLD_P_B_SCALE: temp_cal_data.phases[1].current[0].P_scale_factor = word2; break;
  case HOLD_P_C_SCALE: temp_cal_data.phases[2].current[0].P_scale_factor = word2; break;
  
  case HOLD_A_PHASE: temp_cal_data.phases[0].current[0].phase_correction = word2; break;
  case HOLD_B_PHASE: temp_cal_data.phases[1].current[0].phase_correction = word2; break;
  case HOLD_C_PHASE: temp_cal_data.phases[2].current[0].phase_correction = word2; break;
  
  default: return; break;
  }
  
  // Rewrite old data with adjust
  clear_calibration_data();
  write_calibration_data(&temp_cal_data, &temp_cfg_data);

  RS485_sendBuf(port, ports[port].rx_msg.buf.uint8, 8); // header + data 
  return;
}

void process_read_hold_reg(void)
{
  word1 = ((uint16_t)ports[port].rx_msg.buf.uint8[2])<<8 | ports[port].rx_msg.buf.uint8[3];
  word2 = ((uint16_t)ports[port].rx_msg.buf.uint8[4])<<8 | ports[port].rx_msg.buf.uint8[5];

  if (word2 > 18)
    return;
  
  ports[port].tx_msg.buf.uint8[0] = get_cfg_mb_address();
  ports[port].tx_msg.buf.uint8[1] = 0x03;
  ports[port].tx_msg.buf.uint8[2] = word2 * 2; //byte count
  
  uint16_t* map_ptr = &holding_registers.addr[word1];
  
  if(map_ptr == NULL)
    return;
  
  memcpy(&(ports[port].tx_msg.buf.uint8[3]), map_ptr, word2 * 2);
    
  uint16_t crc = CalculateCRC(ports[port].tx_msg.buf.uint8, 3 + word2 * 2);
  
  ports[port].tx_msg.buf.uint8[4 + word2 * 2] = (uint8_t)(crc>>8);
  ports[port].tx_msg.buf.uint8[3 + word2 * 2] = (uint8_t)(crc&0x00FF);
  
  RS485_sendBuf(port, ports[port].tx_msg.buf.uint8, 5 + word2 * 2); // header + data
  return;
}

void process_read_inp_reg(void)
{
  word1 = ((uint16_t)ports[port].rx_msg.buf.uint8[2])<<8 | ports[port].rx_msg.buf.uint8[3];
  word2 = ((uint16_t)ports[port].rx_msg.buf.uint8[4])<<8 | ports[port].rx_msg.buf.uint8[5];
  if (word2 > 18)
    return;
  
  ports[port].tx_msg.buf.uint8[0] = get_cfg_mb_address();
  ports[port].tx_msg.buf.uint8[1] = 0x04;
  ports[port].tx_msg.buf.uint8[2] = word2 * 2; //byte count
  
  uint8_t* map_ptr = get_ptr_from_input_register_addr(word1);
  
  if(map_ptr == NULL)
    return;
  
  memcpy(&(ports[port].tx_msg.buf.uint8[3]), map_ptr, word2 * 2);
    
  uint16_t crc = CalculateCRC(ports[port].tx_msg.buf.uint8, 3 + word2 * 2);
  
  ports[port].tx_msg.buf.uint8[4 + word2 * 2] = (uint8_t)(crc>>8);
  ports[port].tx_msg.buf.uint8[3 + word2 * 2] = (uint8_t)(crc&0x00FF);
  
  RS485_sendBuf(port, ports[port].tx_msg.buf.uint8, 5 + word2 * 2); // header + data
  return;
}

/* This routine is called regularly from the main polling loop, to check for completed incoming
   DLT-645 messages, and to service them. */
void dlt645_service(void)
{   
  if (ports[port].rx_frame_pending)
  {
    uint16_t crc = CalculateCRC(ports[port].rx_msg.buf.uint8, ports[port].rx_msg.len-2);
    
    // Verifica CRC
    if (ports[port].rx_msg.buf.uint8[ports[port].rx_msg.len-2] == (uint8_t)(crc&0x00FF) &&
        ports[port].rx_msg.buf.uint8[ports[port].rx_msg.len-1] == (uint8_t)(crc>>8))
    {
      switch (ports[port].rx_msg.buf.uint8[1])
      {
      // Espera 8bytes
      case 0x02: //Read Input Status
        break;
        
      case 0x03: //Read Holding Register
        process_read_hold_reg();
        break;
        
      case 0x04: //Read Input Register
        process_read_inp_reg();
        break;
        
      case 0x05:  //Force Single Coil
        process_force_single_coil();
        update_holding_registers();
        break;
      
      case 0x06: //Preset Single Register
        process_preset_single_reg();
        update_holding_registers();
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
        ports[port].rx_msg.len = 0;
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
    
    if (ports[port].rx_msg.ptr != 0 || ports[port].rx_msg.buf.uint8[0] != 0)
    {
      //memset(ports[port].rx_msg.buf.uint8, 0, ports[port].rx_msg.ptr);
      memset(ports[port].rx_msg.buf.uint8, 0, ports[port].rx_msg.len);
      ports[port].rx_msg.ptr = 0;
    }
  }
  
  if(port++ == 2) port = 0;
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
