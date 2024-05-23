/*******************************************************************************
 *  emeter-main.c -
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
#if !defined(__TI_COMPILER_VERSION__)  &&  defined(__GNUC__)
#include <signal.h>
#endif
#include <math.h>
#if defined(__MSP430__)
#include <msp430.h>
#endif
#define __MAIN_PROGRAM__


int ModoDisplay=0;

volatile unsigned long Contador4096=0;
unsigned long int TempoBotao=0;
unsigned long int TempoDisplayOled=0;
extern volatile int oled_step;
unsigned long TempoMapaMemoria=0;


#include "emeter-template.h"

#include <emeter-toolkit.h>
#include <emeter-metrology.h>

#if defined(IEC62056_SUPPORT)
#include <uart_comms.h>
#include <iec62056_46_user.h>
#include "emeter-iec62056.h"
#endif
#if defined(DLT645_SUPPORT)
#include "emeter-dlt645.h"
#endif

#if defined(TRNG_PURITY_TESTS)
#include "fips_tests.h"
#endif

#include "emeter-main.h"
#include "emeter-app.h"
#include "emeter-rtc.h"
#include "emeter-lcd.h"

#include "emeter-keypad.h"
#include "emeter-autocal.h"

#if defined(OLED_DISPLAY_SUPPORT)  
#include "emeter-basic-oled.h"
#endif
#if defined(LCD_DISPLAY_SUPPORT)
#include "emeter-basic-display.h"
#endif

#if defined(IHD430_SUPPORT)
#include "emeter-communication.h"
#endif

#if defined(TEMPERATURE_SUPPORT)  &&  defined(TRNG_SUPPORT)  &&  defined(IEC62056_SUPPORT)
/* 16 byte salt for DLMS authentication exchanges */
uint8_t salt[16];
int salt_counter = 0;
#endif


#if !defined(NO_ENERGY_ACCUMULATION)
/* These are all the possible types of consumed energies */
#if NUM_PHASES == 1
energy_t energy_consumed[1][APP_TOTAL_ENERGY_BINS] = {0};
#else
energy_t energy_consumed[NUM_PHASES + 1][APP_TOTAL_ENERGY_BINS] = {0};
#endif
#endif

#if defined(SAG_SWELL_SUPPORT)
uint16_t sag_events[NUM_PHASES];
uint32_t sag_duration[NUM_PHASES]; 
uint16_t swell_events[NUM_PHASES];
uint32_t swell_duration[NUM_PHASES]; 
#endif


static __inline__ int32_t abs32(int32_t x)
{
    return (x < 0)  ?  -x  :  x;
}

#if defined(BATTERY_MONITOR_SUPPORT)
void test_battery(void)
{
    P3DIR |= (BIT1);
    P3OUT &= ~(BIT1);
    battery_countdown = 1000;
}
#endif

void update_input_registers(input_registers_t* _input_registers)
{
  _input_registers->grandezas.tensao_linha_1 = rms_voltage(0);
  _input_registers->grandezas.tensao_linha_2 = rms_voltage(1);
  _input_registers->grandezas.tensao_linha_3 = rms_voltage(2);
  _input_registers->grandezas.corrente_linha_1 = rms_current(0);
  _input_registers->grandezas.corrente_linha_2 = rms_current(1);
  _input_registers->grandezas.corrente_linha_3 = rms_current(2);
  _input_registers->grandezas.potencia_ativa_tri = active_power(FAKE_PHASE_TOTAL);
  _input_registers->grandezas.potencia_ativa_1 = active_power(0);
  _input_registers->grandezas.potencia_ativa_2 = active_power(1);
  _input_registers->grandezas.potencia_ativa_3 = active_power(2);
  _input_registers->grandezas.potencia_reativa_tri = reactive_power(FAKE_PHASE_TOTAL);
  _input_registers->grandezas.potencia_reativa_1 = reactive_power(0);
  _input_registers->grandezas.potencia_reativa_2 = reactive_power(1);
  _input_registers->grandezas.potencia_reativa_3 = reactive_power(2);
  _input_registers->grandezas.potencia_aparente_tri = apparent_power(FAKE_PHASE_TOTAL);
  _input_registers->grandezas.potencia_aparente_1 = apparent_power(0);
  _input_registers->grandezas.potencia_aparente_2 = apparent_power(1);
  _input_registers->grandezas.potencia_aparente_3 = apparent_power(2);
  _input_registers->grandezas.fator_potencia_1 = power_factor(0);
  _input_registers->grandezas.fator_potencia_2 = power_factor(1);
  _input_registers->grandezas.fator_potencia_3 = power_factor(2);
  _input_registers->grandezas.freq_1 = mains_frequency(0);
  _input_registers->grandezas.freq_2 = mains_frequency(1);
  _input_registers->grandezas.freq_3 = mains_frequency(2);

  _input_registers->grandezas.energia_atv_pos = energy_consumed[FAKE_PHASE_TOTAL][APP_ACTIVE_ENERGY_IMPORTED];
  _input_registers->grandezas.energia_atv_pos_fase_1 = energy_consumed[0][APP_ACTIVE_ENERGY_IMPORTED];
  _input_registers->grandezas.energia_atv_pos_fase_2 = energy_consumed[1][APP_ACTIVE_ENERGY_IMPORTED];
  _input_registers->grandezas.energia_atv_pos_fase_3 = energy_consumed[2][APP_ACTIVE_ENERGY_IMPORTED];

  _input_registers->grandezas.energia_rtv_pos = (long) energy_consumed[FAKE_PHASE_TOTAL][APP_REACTIVE_ENERGY_QUADRANT_I] + (long) energy_consumed[FAKE_PHASE_TOTAL][APP_REACTIVE_ENERGY_QUADRANT_IV];
  _input_registers->grandezas.energia_rtv_pos_fase_1 = (long) energy_consumed[0][APP_REACTIVE_ENERGY_QUADRANT_I] + (long) energy_consumed[0][APP_REACTIVE_ENERGY_QUADRANT_IV];
  _input_registers->grandezas.energia_rtv_pos_fase_1 = (long) energy_consumed[1][APP_REACTIVE_ENERGY_QUADRANT_I] +(long)  energy_consumed[1][APP_REACTIVE_ENERGY_QUADRANT_IV];
  _input_registers->grandezas.energia_rtv_pos_fase_1 = (long) energy_consumed[2][APP_REACTIVE_ENERGY_QUADRANT_I] + (long) energy_consumed[2][APP_REACTIVE_ENERGY_QUADRANT_IV];
}

input_registers_t input_registers;
holding_registers_t holding_registers;

#if defined(__IAR_SYSTEMS_ICC__)  ||  defined(__TI_COMPILER_VERSION__)
void main(void)
#else
int main(int argc, char *argv[])
#endif
{
#if NUM_PHASES == 1
#define ph 0
#else
    int ph;
#endif
#if defined(TRNG_SUPPORT)
    uint16_t randy;
#endif
#if defined(TRNG_PURITY_TESTS)
    int fips_result;
#endif
    static int32_t x;
    int phase_state;
#if (defined(PHASE_REVERSED_DETECTION_SUPPORT)  &&  defined(ON_REVERSED_SELECT_POSITIVE_READING))  ||  defined(PHASE_UNBALANCED_DETECTION_SUPPORT)
    int metrology_state;
#endif

#if !defined(__MSP430__)
    if (start_host_environment(argc, argv) < 0)
        exit(2);
#endif
     
    //clear_calibration_data();
    system_setup(); 
  
#if defined(TRNG_PURITY_TESTS)
    fips_init();
    fips_result = -1;
#endif

#if defined(IEC62056_SUPPORT)
    serial_timer_init();
    iec62056_init();
#endif
    
#if defined(ESP_SUPPORT)
    esp_start_measurement();
#endif

#if defined(MULTI_RATE_SUPPORT)
    tariff_initialise();
#endif

    for (;;)
    {
        kick_watchdog();
        
        if(Contador4096 - TempoMapaMemoria > 4096)// atualiza 1s
        {
          update_input_registers(&input_registers);
          TempoMapaMemoria = Contador4096;
        }

#if defined(IEC62056_SUPPORT)
        iec62056_service();
#endif
#if defined(DLT645_SUPPORT)
        dlt645_service();
#endif

#if defined(TRNG_SUPPORT)
    #if defined(TRNG_PURITY_TESTS)
        /* Perform FIPS testing of the true random number generator */
        if (trng(&randy) == 0)
        {
            if ((fips_result = fips_test(randy)) != -1)
            {
                for (;;)
                  /* Stick here */;
            }
        }
    #elif defined(IEC62056_SUPPORT)
        if (trng(&randy) == 0)
        {
            /* Roll around the 16 byte salt buffer, updating it little by little */
            salt[salt_counter++] = randy >> 8;
            salt[salt_counter++] = randy;
            if (salt_counter >= 16)
                salt_counter = 0;
        }
    #endif
#endif
#if !defined(__MSP430__)
        /* In the host environment we need to simulate interrupts here */
        adc_interrupt();
#endif
#if NUM_PHASES > 1
        for (ph = 0;  ph < NUM_PHASES;  ph++)
#endif
        {
            /* Unless we are in normal operating mode, we should wait to be
               woken by a significant event from the interrupt routines. */
#if defined(__MSP430__)
            if (operating_mode != OPERATING_MODE_NORMAL)
                _BIS_SR(LPM0_bits);
#endif
#if defined(POWER_DOWN_SUPPORT)
            if (operating_mode == OPERATING_MODE_POWERFAIL)
                switch_to_powerfail_mode();
#endif
#if defined(LIMP_MODE_SUPPORT)  &&  defined(IEC1107_SUPPORT)
            if (!meter_calibrated())
                enable_ir_receiver();
#endif
            phase_state = phase_status(ph);
#if (defined(PHASE_REVERSED_DETECTION_SUPPORT)  &&  defined(ON_REVERSED_SELECT_POSITIVE_READING))  ||  defined(PHASE_UNBALANCED_DETECTION_SUPPORT)
            metrology_state = metrology_status();
#endif
            if ((phase_state & PHASE_STATUS_NEW_LOG))
            {
                /* The background activity has informed us that it is time to
                   perform a block processing operation. */
                /* We can only do real power assessment in full operating mode */
                calculate_phase_readings(ph);
                x = active_power(ph);
                
                // Send to IHD430 when 0th phase element is updated.
                #if defined(IHD430_SUPPORT)
                if(!ph) send_reading_to_CC2530_for_IHD430 ( active_power(FAKE_PHASE_TOTAL)/10);
                #endif
                
                if (abs32(x) < RESIDUAL_POWER_CUTOFF  ||  (phase_state & PHASE_STATUS_V_OVERRANGE))
                {
                    x = 0;
                    /* Turn off the LEDs, regardless of the internal state of the
                       reverse and imbalance assessments. */
#if defined(PHASE_REVERSED_DETECTION_SUPPORT)
                    metrology_state &= ~METROLOGY_STATUS_REVERSED;
                    clr_reverse_current_indicator();
#endif
#if defined(PHASE_UNBALANCED_DETECTION_SUPPORT)
                    metrology_state &= ~METROLOGY_STATUS_EARTHED;
                    clr_earthed_indicator();
#endif
                }
                else
                {
                    if (operating_mode == OPERATING_MODE_NORMAL)
                    {
#if defined(PHASE_REVERSED_DETECTION_SUPPORT)  &&  defined(ON_REVERSED_SELECT_POSITIVE_READING)
                        if ((phase_state & PHASE_STATUS_REVERSED))
                        {
                            metrology_state |= METROLOGY_STATUS_REVERSED;
                            set_reverse_current_indicator();
                        }
                        else
                        {
                            metrology_state &= ~METROLOGY_STATUS_REVERSED;
                            clr_reverse_current_indicator();
                        }
#endif
#if defined(PHASE_UNBALANCED_DETECTION_SUPPORT)
                        if ((phase_state & PHASE_STATUS_UNBALANCED))
                        {
                            metrology_state |= METROLOGY_STATUS_EARTHED;
                            set_earthed_indicator();
                        }
                        else
                        {
                            metrology_state &= ~METROLOGY_STATUS_EARTHED;
                            clr_earthed_indicator();
                        }
#endif
                    }
#if defined(LIMP_MODE_SUPPORT)
                    else
                    {
    #if defined(PHASE_REVERSED_DETECTION_SUPPORT)
                        /* We cannot tell forward from reverse current in limp mode,
                           so just say it is not reversed. */
                        metrology_state &= ~METROLOGY_STATUS_REVERSED;
                        clr_reverse_current_indicator();
    #endif
    #if defined(PHASE_UNBALANCED_DETECTION_SUPPORT)
                        /* We are definitely in the unbalanced state, but only set
                           the indicator if we have persistence checked, and the current
                           is sufficient to sustain operation. */
                        if ((phase_state & PHASE_STATUS_UNBALANCED)  &&  rms_current(ph) >= LIMP_MODE_MINIMUM_CURRENT)
                        {
                            metrology_state |= METROLOGY_STATUS_EARTHED;
                            set_earthed_indicator();
                        }
                        else
                        {
                            metrology_state &= ~METROLOGY_STATUS_EARTHED;
                            clr_earthed_indicator();
                        }
    #endif
    #if defined(LIMP_MODE_SUPPORT)  &&  defined(IEC1107_SUPPORT)
                        /* Only run the IR interface if we are sure there is enough power from the
                           supply to support the additional current drain. If we have not yet been
                           calibrated we had better keep the IR port running so we can complete the
                           calibration. */
                        if (rms_current(ch) >= LIMP_MODE_MINIMUM_CURRENT_FOR_IR
                            ||
                            !meter_calibrated())
                        {
                            enable_ir_receiver();
                        }
                        else
                        {
                            disable_ir_receiver();
                        }
    #endif
                    }
#endif
                }
#if defined(SUPPORT_ONE_POINT_AUTOCALIBRATION)
                if (ph == 0)
                    autocalibrate();
#endif
            }
#if defined(LIMP_MODE_SUPPORT)
            metrology_limp_normal_detection();
#endif
        }
#if NUM_PHASES > 1  &&  defined(NEUTRAL_MONITOR_SUPPORT)  &&  defined(IRMS_SUPPORT)
        if ((phase_status(FAKE_PHASE_NEUTRAL) & PHASE_STATUS_NEW_LOG))
        {
            /* The background activity has informed us that it is time to
               perform a block processing operation. */
            calculate_neutral_readings();
        }
#endif

#if defined(MULTI_RATE_SUPPORT)
        tariff_management();
#endif

#if defined(RTC_SUPPORT)

#if 1
        
#define TEMPO_OLED_NORMAL  2000L*4096/1000 // 2000 ms
#define TEMPO_BOTAO_RAPIDO  1000L*4096/1000 //  1000 ms

  enum EstadosTeclado {
  INICIA_ESPERA_PRIMEIRO_TECLA,
  ESPERA_PRIMEIRO_APERTO,
  ESPERA_PRIMEIRA_SOLTURA,  
  ESPERA_TECLA_APERTAR, 
  ESPERA_TECLA_SOLTAR,   
  ESPERA_TECLA_FINAL };
  
  
  
    switch (ModoDisplay)
    {
    case INICIA_ESPERA_PRIMEIRO_TECLA: // Inicio
            ModoDisplay=ESPERA_PRIMEIRO_APERTO;
            update_oled();
            TempoDisplayOled=Contador4096; // Reseta timer
        break;

    case ESPERA_PRIMEIRO_APERTO: // Botao solto, esperando botao ser pressionado
        if ((P1IN&BIT7)==0) {
            // Botao Pressionado
            ModoDisplay=ESPERA_PRIMEIRA_SOLTURA;
        }    
        else if ((Contador4096-TempoDisplayOled) > TEMPO_OLED_NORMAL) {
            update_oled();
            TempoDisplayOled=Contador4096; // Reseta timer
       }
        break;
    case ESPERA_PRIMEIRA_SOLTURA: // Botao pressionado => Pare
        if ((P1IN&BIT7)!=0) {
            // Botao Solto
               TempoDisplayOled=Contador4096; // Reseta timer
               ModoDisplay=ESPERA_TECLA_APERTAR;
            }   
        break;
    case ESPERA_TECLA_APERTAR:
        if ((P1IN&BIT7)==0) {
            // Botao pressionado
            update_oled();
            TempoDisplayOled=Contador4096; // Reseta timer
            TempoBotao=Contador4096; // Reseta timer 
            ModoDisplay=ESPERA_TECLA_SOLTAR;
        }
        break;
    case ESPERA_TECLA_SOLTAR:
        if ((Contador4096-TempoBotao) > TEMPO_BOTAO_RAPIDO) {
            TempoDisplayOled=Contador4096; // Reseta timer

            oled_step = 0;      // Volta para tela inicial
            update_oled();
            ModoDisplay=ESPERA_TECLA_FINAL;   

        }
        else {    
            if ((P1IN&BIT7)!=0) {
                // Botao Solto
                ModoDisplay=3; 
            }     
        }  
        break;

    case ESPERA_TECLA_FINAL:
        if ((P1IN&BIT7)!=0) {
            // Botao solto
            ModoDisplay=ESPERA_PRIMEIRO_APERTO;
        }
        break;        
    default:
        break;
    }
#endif
        /* Do display and other housekeeping here */
        if ((rtc_status & RTC_STATUS_TICKER))
        {
            /* Two seconds have passed */
            /* We have a 2 second tick */
            rtc_status &= ~RTC_STATUS_TICKER;

#if 0
        #if defined(OLED_DISPLAY_SUPPORT) 
            update_oled();
        #endif

        #if defined(LCD_DISPLAY_SUPPORT)
            /* Update the display, cycling through the phases */
            update_display();
        #endif
#endif

    #if !defined(__MSP430_HAS_RTC_C__)  &&  defined(RTC_SUPPORT)  &&  defined(CORRECTED_RTC_SUPPORT)
         correct_rtc();
    #endif
        }
#endif
    }
#if !defined(__IAR_SYSTEMS_ICC__)  &&  !defined(__TI_COMPILER_VERSION__)
    return  0;
#endif
}

#if defined(LIMP_MODE_SUPPORT)
void switch_to_limp_mode(void)
{
    clr_normal_indicator();
    #if defined(PHASE_REVERSED_DETECTION_SUPPORT)
    clr_reverse_current_indicator();
    #endif
    #if defined(IEC62056_21_SUPPORT)
    disable_ir_receiver();
    #endif
}
#endif

void switch_to_normal_mode(void)
{
#if defined(LIMP_MODE_SUPPORT)
    set_normal_indicator();
#endif
    /* The LCD might need to be revived */
#if defined(LCD_DISPLAY_SUPPORT)
    lcd_awaken();
#else
    /* Tell the world we are ready to start */
#endif
#if defined(IEC62056_21_SUPPORT)
    enable_ir_receiver();
#endif
}

void switch_to_powerfail_mode(void)
{
#if defined(LIMP_MODE_SUPPORT)
    clr_normal_indicator();
#endif
#if defined(PHASE_REVERSED_DETECTION_SUPPORT)
    clr_reverse_current_indicator();
#endif
#if defined(PHASE_UNBALANCED_DETECTION_SUPPORT)
    clr_earthed_indicator();
#endif
#if defined(IEC62056_21_SUPPORT)
    disable_ir_receiver();
#endif
}

#if defined(TOTAL_ACTIVE_ENERGY_PULSES_PER_KW_HOUR)  ||  defined(ACTIVE_ENERGY_PULSES_PER_KW_HOUR)
    #if NUM_PHASES == 1
void active_energy_pulse_start(void)
{
    custom_active_energy_pulse_start();
}
    #else
void active_energy_pulse_start(int ph)
{
    custom_active_energy_pulse_start(ph);
}
    #endif

    #if NUM_PHASES == 1
void active_energy_pulse_end(void)
{
    //custom_active_energy_pulse_end();
}
    #else
void active_energy_pulse_end(int ph)
{
    // custom_active_energy_pulse_end(ph);
}
    #endif
#endif

#if defined(TOTAL_REACTIVE_ENERGY_PULSES_PER_KVAR_HOUR)  ||  defined(REACTIVE_ENERGY_PULSES_PER_KVAR_HOUR)
    #if NUM_PHASES == 1
void reactive_energy_pulse_start(void)
{
    custom_reactive_energy_pulse_start();
}
    #else
void reactive_energy_pulse_start(int ph)
{
    custom_reactive_energy_pulse_start(ph);
}
    #endif

    #if NUM_PHASES == 1
void reactive_energy_pulse_end(void)
{
    custom_reactive_energy_pulse_end();
}
    #else
void reactive_energy_pulse_end(int ph)
{
    custom_reactive_energy_pulse_end(ph);
}
    #endif
#endif

void energy_update(int phx, int type, energy_t amount)
{
#if !defined(NO_ENERGY_ACCUMULATION)
    static const int map[16] =
    {
#if defined(ACTIVE_ENERGY_SUPPORT)  ||  defined(TOTAL_ACTIVE_ENERGY_SUPPORT)
        APP_ACTIVE_ENERGY_IMPORTED,
        APP_ACTIVE_ENERGY_EXPORTED,
#else
        -1,
        -1,
#endif
#if defined(FUNDAMENTAL_ACTIVE_ENERGY_SUPPORT)  ||  defined(TOTAL_FUNDAMENTAL_ACTIVE_ENERGY_SUPPORT)
        APP_FUNDAMENTAL_ACTIVE_ENERGY_IMPORTED,
        APP_FUNDAMENTAL_ACTIVE_ENERGY_EXPORTED,
#else
        -1,
        -1,
#endif
#if defined(REACTIVE_ENERGY_SUPPORT)  ||  defined(TOTAL_REACTIVE_ENERGY_SUPPORT)
        APP_REACTIVE_ENERGY_QUADRANT_I,
        APP_REACTIVE_ENERGY_QUADRANT_II,
        APP_REACTIVE_ENERGY_QUADRANT_III,
        APP_REACTIVE_ENERGY_QUADRANT_IV,
#else
        -1,
        -1,
        -1,
        -1,
#endif
#if defined(FUNDAMENTAL_REACTIVE_ENERGY_SUPPORT)  ||  defined(TOTAL_FUNDAMENTAL_REACTIVE_ENERGY_SUPPORT)
        APP_FUNDAMENTAL_REACTIVE_ENERGY_QUADRANT_I,
        APP_FUNDAMENTAL_REACTIVE_ENERGY_QUADRANT_II,
        APP_FUNDAMENTAL_REACTIVE_ENERGY_QUADRANT_III,
        APP_FUNDAMENTAL_REACTIVE_ENERGY_QUADRANT_IV,
#else
        -1,
        -1,
        -1,
        -1,
#endif
#if defined(APPARENT_ENERGY_SUPPORT)  ||  defined(TOTAL_APPARENT_ENERGY_SUPPORT)
        APP_APPARENT_ENERGY_IMPORTED,
        APP_APPARENT_ENERGY_EXPORTED,
#else
        -1,
        -1,
#endif
#if defined(INTEGRATED_V2_SUPPORT)  ||  defined(TOTAL_INTEGRATED_V2_SUPPORT)
        APP_INTEGRATED_V2,
#else
        -1,
#endif
#if defined(INTEGRATED_I2_SUPPORT)  ||  defined(TOTAL_INTEGRATED_I2_SUPPORT)
        APP_INTEGRATED_I2,
#else
        -1,
#endif
    };

    /* Adjust a stored energy level */
    if (type < 16  &&  map[type] >= 0)
        energy_consumed[phx][map[type]] += amount;
#endif
}

#if defined(SAG_SWELL_SUPPORT)
void sag_swell_event(int phx, int event)
{
    /* Processing in this routine MUST be very quick, as it is called from the
       metrology interrupt. It is OK to set flags, or perform other functions taking
       just a few instructions. Do not do anything slow. */
    switch (event)
    {
    //case SAG_SWELL_VOLTAGE_POWER_DOWN_OK:
        /* The power is OK */
        //break;
    case SAG_SWELL_VOLTAGE_POWER_DOWN_SAG:
        /* The power is failing, and important information should be saved before the power
           supply starts to drop. */
        break;
    case SAG_SWELL_VOLTAGE_SAG_ONSET:
        sag_events[phx]++;
        sag_duration[phx]++;
        break;
    case SAG_SWELL_VOLTAGE_SAG_CONTINUING:
        sag_duration[phx]++;
        break;
    case SAG_SWELL_VOLTAGE_NORMAL:
        /* The voltage has returned to its normal range */
        break;
    case SAG_SWELL_VOLTAGE_SWELL_ONSET:
        swell_events[phx]++;
        swell_duration[phx]++;
        break;
    case SAG_SWELL_VOLTAGE_SWELL_CONTINUING:
        swell_duration[phx]++;
        break;
    }
}
#endif

#if defined(x__MSP430__)
void power_sense(void)
{
    #if defined(POWER_DOWN_SUPPORT)  &&  defined(POWER_UP_BY_SUPPLY_SENSING)
    /* Select the lower threshold to watch for the power supply dying. */
    CACTL1 = CAREF_1 | CAON;
    #endif
    #if defined(BASIC_KEYPAD_SUPPORT)  ||  defined(CUSTOM_KEYPAD_SUPPORT)
    if (keypad_debounce())
        _BIC_SR_IRQ(LPM0_bits);
    #endif

    #if defined(POWER_DOWN_SUPPORT)  &&  defined(POWER_UP_BY_SUPPLY_SENSING)
        #if defined(__MSP430_HAS_COMPA__)
    if ((CACTL2 & CAOUT))
        #else
    /* Use an I/O pin to sense the power falling */
    POWER_GOOD_THRESHOLD_LOW;
    if (!POWER_GOOD_SENSE)
        #endif
    {
        /* The comparator output can oscillate a little around the
           switching point, so we need to do some debouncing. */
        if (power_down_debounce < POWER_FAIL_DEBOUNCE + 1)
        {
            if (++power_down_debounce == POWER_FAIL_DEBOUNCE)
            {
                power_down_debounce = 0;
                /* The power is falling. We need to get to a low power
                   consumption state now! The battery will be supplying the
                   meter soon. */
                operating_mode = OPERATING_MODE_POWERFAIL;
        #if defined(__MSP430__)
                /* Get the foreground to respond quickly. It might be conserving
                   power (e.g. in limp mode). */
                _BIC_SR_IRQ(LPM0_bits);
        #endif
            }
        }
    }
    else
    {
        power_down_debounce = 0;
    }
        #if defined(__MSP430_HAS_COMPA__)
    CACTL1 &= ~(CAON);
        #endif
    #endif
}
#endif

int is_calibration_enabled(void)
{
    /* This custom function must check the conditions, such as passswords and shorting link tests,
       appropriate to the meter's "enable calibration" condition. */
    return custom_is_calibration_enabled();
}

#if defined(__IAR_SYSTEMS_ICC__)  ||  defined(__TI_COMPILER_VERSION__)

#if defined(__MSP430AFE253__)
#pragma vector = PORT2_VECTOR, \
		 	 	 PORT1_VECTOR, \
		 	 	 TIMERA1_VECTOR, \
		 	 	 TIMERA0_VECTOR, \
	 	 	     /* USART0TX_VECTOR, */ \
 	 	 	 	 /* USART0RX_VECTOR, */ \
 	 	 	 	 /* WDT_VECTOR, */ \
 	 	 	 	 /* SD24_VECTOR, */ \
 	 	         NMI_VECTOR
#elif defined(__MSP430F427__)  ||  defined(__MSP430F427A__)
#pragma vector = /* BASICTIMER_VECTOR, */ \
		 	 	 PORT2_VECTOR, \
		 	 	 PORT1_VECTOR, \
		 	 	 TIMERA1_VECTOR, \
		 	 	 /* TIMERA0_VECTOR, */ \
	 	 	     /* USART0TX_VECTOR, */ \
 	 	 	 	 /* USART0RX_VECTOR, */ \
 	 	 	 	 WDT_VECTOR, \
 	 	 	 	 /* SD16_VECTOR, */ \
 	 	         NMI_VECTOR
#elif defined(__MSP430FE427__)  ||  defined(__MSP430FE427A__)  ||  defined(__MSP430FE4272__)
#pragma vector = /* BASICTIMER_VECTOR, */ \
		 	 	 PORT2_VECTOR, \
		 	 	 PORT1_VECTOR, \
		 	 	 TIMERA1_VECTOR, \
		 	 	 /* TIMERA0_VECTOR, */ \
	 	 	     /* USART0TX_VECTOR, */ \
 	 	 	 	 /* USART0RX_VECTOR, */ \
 	 	 	 	 WDT_VECTOR, \
 	 	 	 	 /* SD16_VECTOR, */ \
                 /* ESP430_VECTOR, */ \
 	 	         NMI_VECTOR
#elif defined(__MSP430F4794__)
#pragma vector = /* BASICTIMER_VECTOR, */ \
				 PORT2_VECTOR, \
				 USCIAB1TX_VECTOR, \
				 USCIAB1RX_VECTOR, \
				 PORT1_VECTOR, \
				 TIMERA1_VECTOR, \
				 TIMERA0_VECTOR, \
				 /* SD16A_VECTOR, */ \
				 /* USCIAB0TX_VECTOR, */ \
				 /* USCIAB0RX_VECTOR, */ \
				 WDT_VECTOR, \
				 COMPARATORA_VECTOR, \
				 TIMERB1_VECTOR, \
				 TIMERB0_VECTOR, \
				 NMI_VECTOR
#elif defined(__MSP430F47197__)  ||  defined(__MSP430F47187__)
#pragma vector = /* DMA_VECTOR, */ \
                 /* BASICTIMER_VECTOR, */ \
				 PORT2_VECTOR, \
				 /* USCIAB1TX_VECTOR, */ \
				 /* USCIAB1RX_VECTOR, */ \
				 PORT1_VECTOR, \
				 TIMERA1_VECTOR, \
				 TIMERA0_VECTOR, \
				 /* SD16A_VECTOR, */ \
				 /* USCIAB0TX_VECTOR, */ \
				 /* USCIAB0RX_VECTOR, */ \
				 WDT_VECTOR, \
				 COMPARATORA_VECTOR, \
				 TIMERB1_VECTOR, \
				 TIMERB0_VECTOR, \
				 NMI_VECTOR
#elif defined(__MSP430F6736__)
#pragma vector = /* RTC_VECTOR, */ \
	             LCD_C_VECTOR, \
                 TIMER3_A1_VECTOR, \
                 TIMER3_A0_VECTOR, \
                 PORT2_VECTOR, \
                 TIMER2_A1_VECTOR, \
                 TIMER2_A0_VECTOR, \
                 PORT1_VECTOR, \
                 TIMER1_A1_VECTOR, \
                 TIMER1_A0_VECTOR, \
                 /*DMA_VECTOR, */ \
                 USCI_A2_VECTOR, \
                 /* USCI_A1_VECTOR, */ \
                 TIMER0_A1_VECTOR, \
                 /* TIMER0_A0_VECTOR, */ \
                 /* SD24B_VECTOR, */ \
                 /* ADC10_VECTOR, */ \
                 USCI_B0_VECTOR, \
                 /* USCI_A0_VECTOR, */ \
                 /* WDT_VECTOR, */ \
                 UNMI_VECTOR, \
                 SYSNMI_VECTOR
#elif defined(__MSP430F67641__) || defined(__MSP430F67641A__)
#pragma vector = /* RTC_VECTOR, */ \
	             LCD_C_VECTOR, \
                 TIMER3_A1_VECTOR, \
                 TIMER3_A0_VECTOR, \
                 PORT2_VECTOR, \
                 TIMER2_A1_VECTOR, \
                 TIMER2_A0_VECTOR, \
                 PORT1_VECTOR, \
                 TIMER1_A1_VECTOR, \
                 TIMER1_A0_VECTOR, \
                 /*DMA_VECTOR, */ \
                 USCI_A2_VECTOR, \
                 /* USCI_A1_VECTOR, */ \
                 TIMER0_A1_VECTOR, \
                 /* TIMER0_A0_VECTOR, */ \
                 /* SD24B_VECTOR, */ \
                 /* ADC10_VECTOR, */ \
                 USCI_B0_VECTOR, \
                 /* USCI_A0_VECTOR, */ \
                 /* WDT_VECTOR, */ \
                 UNMI_VECTOR, \
                 SYSNMI_VECTOR                   
#elif defined(__MSP430F6749__)  ||  defined(__MSP430F6779__)
#pragma vector = AES_VECTOR, \
                 COMP_B_VECTOR, \
                 /* RTC_VECTOR, */ \
	             LCD_C_VECTOR, \
                 TIMER3_A1_VECTOR, \
                 TIMER3_A0_VECTOR, \
                 PORT2_VECTOR, \
                 TIMER2_A1_VECTOR, \
                 TIMER2_A0_VECTOR, \
                 PORT1_VECTOR, \
                 USCI_B1_VECTOR, \
                 USCI_A3_VECTOR, \
                 TIMER1_A1_VECTOR, \
                 TIMER1_A0_VECTOR, \
                 DMA_VECTOR, \
                 /* USCI_A2_VECTOR, */ \
                 /* USCI_A1_VECTOR, */ \
                 TIMER0_A1_VECTOR, \
                 TIMER0_A0_VECTOR, \
                 /* SD24B_VECTOR, */ \
                 /* ADC10_VECTOR, */ \
                 USCI_B0_VECTOR, \
                 /* USCI_A0_VECTOR, */ \
                 /* WDT_VECTOR, */ \
                 UNMI_VECTOR, \
                 SYSNMI_VECTOR
#elif defined(__MSP430F67491__)  ||  defined(__MSP430F67791__)
#pragma vector = COMP_B_VECTOR, \
                 /* RTC_VECTOR, */ \
	             LCD_C_VECTOR, \
                 TIMER3_A1_VECTOR, \
                 TIMER3_A0_VECTOR, \
                 PORT2_VECTOR, \
                 TIMER2_A1_VECTOR, \
                 TIMER2_A0_VECTOR, \
                 PORT1_VECTOR, \
                 USCI_B1_VECTOR, \
                 /* USCI_A3_VECTOR, */ \
                 TIMER1_A1_VECTOR, \
                 TIMER1_A0_VECTOR, \
                 DMA_VECTOR, \
                 /* USCI_A2_VECTOR, */ \
                 /* USCI_A1_VECTOR, */ \
                 TIMER0_A1_VECTOR, \
                 TIMER0_A0_VECTOR, \
                 /* SD24B_VECTOR, */ \
                 /* ADC10_VECTOR, */ \
                 USCI_B0_VECTOR, \
                 /* USCI_A0_VECTOR, */ \
                 /* WDT_VECTOR, */ \
                 UNMI_VECTOR, \
                 SYSNMI_VECTOR
#elif defined(__MSP430I4020__)
#pragma vector = PORT1_VECTOR, \
		         TIMER0_A1_VECTOR, \
		         TIMER0_A0_VECTOR, \
                 USCI_B0_VECTOR, \
                 /* USCI_A0_VECTOR, */ \
                 /* WDT_VECTOR, */ \
                 VMON_VECTOR, \
                 /* SD24_VECTOR, */ \
                 NMI_VECTOR
#elif defined(__MSP430i2040__)  ||  defined(__MSP430i2041__)  ||  defined(__MSP430I2040__)  ||  defined(__MSP430I2041__)
#pragma vector = PORT1_VECTOR, \
		         PORT2_VECTOR, \
                 TIMER0_A1_VECTOR, \
                 TIMER0_A0_VECTOR, \
                 TIMER1_A1_VECTOR, \
                 TIMER1_A0_VECTOR, \
                 USCI_B0_VECTOR, \
                 /* USCI_A0_VECTOR, */ \
                 WDT_VECTOR, \
                 VMON_VECTOR, \
                 /* SD24_VECTOR, */ \
                 NMI_VECTOR
#else
#error Device not recognised
#endif
__interrupt void ISR_trap(void)
{
    /* The following will cause an access violation which results in a PUC reset */
    WDTCTL = 0;
}
#endif
