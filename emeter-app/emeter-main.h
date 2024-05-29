/*******************************************************************************
 *  emeter-main.h -
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

#if !defined(_METER_MAIN_H_)
#define _METER_MAIN_H_

#if !defined(NO_ENERGY_ACCUMULATION)
enum
{
#if defined(ACTIVE_ENERGY_SUPPORT)  ||  defined(TOTAL_ACTIVE_ENERGY_SUPPORT)
    APP_ACTIVE_ENERGY_IMPORTED,
    APP_ACTIVE_ENERGY_EXPORTED,
#endif
#if defined(FUNDAMENTAL_ACTIVE_ENERGY_SUPPORT)  ||  defined(TOTAL_FUNDAMENTAL_ACTIVE_ENERGY_SUPPORT)
    APP_FUNDAMENTAL_ACTIVE_ENERGY_IMPORTED,
    APP_FUNDAMENTAL_ACTIVE_ENERGY_EXPORTED,
#endif
#if defined(REACTIVE_ENERGY_SUPPORT)  ||  defined(TOTAL_REACTIVE_ENERGY_SUPPORT)
    APP_REACTIVE_ENERGY_QUADRANT_I,
    APP_REACTIVE_ENERGY_QUADRANT_II,
    APP_REACTIVE_ENERGY_QUADRANT_III,
    APP_REACTIVE_ENERGY_QUADRANT_IV,
#endif
#if defined(FUNDAMENTAL_REACTIVE_ENERGY_SUPPORT)  ||  defined(TOTAL_FUNDAMENTAL_REACTIVE_ENERGY_SUPPORT)
    APP_FUNDAMENTAL_REACTIVE_ENERGY_QUADRANT_I,
    APP_FUNDAMENTAL_REACTIVE_ENERGY_QUADRANT_II,
    APP_FUNDAMENTAL_REACTIVE_ENERGY_QUADRANT_III,
    APP_FUNDAMENTAL_REACTIVE_ENERGY_QUADRANT_IV,
#endif
#if defined(APPARENT_ENERGY_SUPPORT)  ||  defined(TOTAL_APPARENT_ENERGY_SUPPORT)
    APP_APPARENT_ENERGY_IMPORTED,
    APP_APPARENT_ENERGY_EXPORTED,
#endif
#if defined(INTEGRATED_V2_SUPPORT)  ||  defined(TOTAL_INTEGRATED_V2_SUPPORT)
    APP_INTEGRATED_V2,
#endif
#if defined(INTEGRATED_I2_SUPPORT)  ||  defined(TOTAL_INTEGRATED_I2_SUPPORT)
    APP_INTEGRATED_I2,
#endif
    APP_TOTAL_ENERGY_BINS
};

/* These are all the possible types of consumed energies */
#if NUM_PHASES == 1
extern energy_t energy_consumed[1][APP_TOTAL_ENERGY_BINS];
#else
extern energy_t energy_consumed[NUM_PHASES + 1][APP_TOTAL_ENERGY_BINS];
#endif
#endif

#if defined(SAG_SWELL_SUPPORT)
extern uint16_t sag_events[NUM_PHASES];
extern uint32_t sag_duration[NUM_PHASES]; 
extern uint16_t swell_events[NUM_PHASES];
extern uint32_t swell_duration[NUM_PHASES]; 
extern volatile unsigned long Contador4096;
#endif

#define NOMINAL_VOLTAGE 220.0
#define UNDER_VOLTAGE_THRESHOLD (NOMINAL_VOLTAGE * 0.90)
#define OVER_VOLTAGE_THRESHOLD (NOMINAL_VOLTAGE * 1.10)
#define IMBALANCE_THRESHOLD 0.02 // 2% imbalance

#define PHASE_A_OVERVOLTAGE BIT0
#define PHASE_B_OVERVOLTAGE BIT1
#define PHASE_C_OVERVOLTAGE BIT2
#define PHASE_A_UNDERVOLTAGE BIT3
#define PHASE_B_UNDERVOLTAGE BIT4
#define PHASE_C_UNDERVOLTAGE BIT5
#define PHASE_IMBALANCE BIT6
#define PHASE_OUT_OF_SEQ  BIT7


typedef struct {
  union {
    struct {
      // 0 - 65
      uint32_t num_serial;
      float tensao_tri;
      float tensao_fase_a_b;
      float tensao_fase_b_c;
      float tensao_fase_c_a;
      float tensao_linha_1;
      float tensao_linha_2;
      float tensao_linha_3;
      float corrente_tri;
      float reservado1;
      float corrente_linha_1;
      float corrente_linha_2;
      float corrente_linha_3;
      float freq_1;
      float freq_2;
      float freq_3;
      float reservado2;
      float potencia_ativa_tri;
      float potencia_ativa_1;
      float potencia_ativa_2;
      float potencia_ativa_3;
      float potencia_reativa_tri;
      float potencia_reativa_1;
      float potencia_reativa_2;
      float potencia_reativa_3;
      float potencia_aparente_tri;
      float potencia_aparente_1;
      float potencia_aparente_2;
      float potencia_aparente_3;
      float fator_potencia_tri;
      float fator_potencia_1;
      float fator_potencia_2;
      float fator_potencia_3;
      
      // Energias 200 - 225
      float energia_atv_pos; //i = 132
      float energia_rtv_pos;
      float energia_atv_neg;
      float energia_rtv_neg;
      float max_dem_atv;
      float dem_atv;
      float max_dem_apr;
      float dem_apr;
      float max_dem_rtv;
      float dem_rtv;
      float max_dem_cor;
      float dem_cor;
      float energia_apr;
      
      // Delta 300 - 339
      // float delta_ene_atv_pos; //i = 184
      // float delta_ene_rtv_pos;
      // float delta_ene_atv_neg;
      // float delta_ene_rtv_neg;
      // float delta_ene_apr;
      // float delta_ene_atv_pos_fase_1;
      // float delta_ene_rtv_pos_fase_1;
      // float delta_ene_atv_neg_fase_1;
      // float delta_ene_rtv_neg_fase_1;
      // float delta_ene_atv_pos_fase_2;
      // float delta_ene_rtv_pos_fase_2;
      // float delta_ene_atv_neg_fase_2;
      // float delta_ene_rtv_neg_fase_2;
      // float delta_ene_atv_pos_fase_3;
      // float delta_ene_rtv_pos_fase_3;
      // float delta_ene_atv_neg_fase_3;
      // float delta_ene_rtv_neg_fase_3;
      // float delta_ene_apr_fase_1;
      // float delta_ene_apr_fase_2;
      // float delta_ene_apr_fase_3;
      
      // Energia/fase 1200 - 1229
      float energia_atv_pos_fase_1; // i = 264
      float energia_rtv_pos_fase_1;
      float energia_atv_neg_fase_1;
      float energia_rtv_neg_fase_1;
      float energia_atv_pos_fase_2;
      float energia_rtv_pos_fase_2;
      float energia_atv_neg_fase_2;
      float energia_rtv_neg_fase_2;      
      float energia_atv_pos_fase_3;
      float energia_rtv_pos_fase_3;
      float energia_atv_neg_fase_3;
      float energia_rtv_neg_fase_3;
      float energia_apr_fas_fase_1;
      float energia_apr_fas_fase_2;
      float energia_apr_fas_fase_3;

      uint16_t status;
    } grandezas;
    
    uint16_t addr[164];
    uint8_t byte[328];
  };
} input_registers_t;

typedef struct {
  union {
    struct {
        uint16_t baud_rate;
        uint16_t mb_address;
    } configs;

    uint16_t addr[2];
    uint8_t byte[4];
  };
} holding_registers_t;

extern input_registers_t input_registers;
extern holding_registers_t holding_registers;

void update_input_registers(input_registers_t* _input_registers);
#endif
