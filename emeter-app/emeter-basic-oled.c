/*******************************************************************************
 *  emeter-basic-display.c -
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
#include <stdio.h>
#include <stdbool.h>
#if defined(__MSP430__)
#include <msp430.h>
#endif

#include "stdarg.h"

#include "emeter-template.h"

#include <emeter-toolkit.h>
#include <emeter-metrology.h>

#include "emeter-main.h"
#include "emeter-app.h"
#include "emeter-rtc.h"
#include "emeter-basic-display.h"
#if defined(MULTI_RATE_SUPPORT)
#include "emeter-multirate.h"
#endif
#include "emeter-oled.h"
#include "emeter-communication.h"

#if defined(MULTI_RATE_SUPPORT)
uint8_t info_step;
uint8_t info_substep;
#endif

volatile int oled_step=-10;

#if defined(OLED_DISPLAY_SUPPORT)  

void OLED_display_startup_message(void)
{
 
    ssd1306_clearDisplay();
    
    //ssd1306_printText_peq(0,1,"ABCDEFGHIJKLMNOPQRTSU");
    //ssd1306_printText_peq(0,2,"012345678901234567890"); 

    ssd1306_printText(3,0,"START");

    //ssd1306_printText_peq(0,2,"012345678901234567890");    
    ssd1306_printText_peq(0,3,"   MEDIDOR MECALOR");
    ssd1306_printText_peq(0,5,"    Z Tecnologia");
    ssd1306_printText_peq(0,7,"     Versao 1.0");   
}

static __inline__ void oled_date_temp()
{
    char oled[17];   
    uint8_t date[6];

    Oled_titulo("MECALOR");  

    #if defined(RTC_SUPPORT)
        get_rtc(date);

        sprintf(oled, "%02d/%02d/20%02d",date[2],date[1],date[0]);
        ssd1306_printText_peq(5, 3,oled); 

        sprintf(oled, "%02d:%02d:%02d",date[3],date[4],date[5]);
        ssd1306_printText_peq(6, 5,oled);
    #endif

    #if defined(TEMPERATURE_SUPPORT)
        Oled_display_valor(0,7,temperature(),1,""," Graus Celcius");
    #endif

}

static __inline__ void oled_active_power()
{
    power_t x;
    int f;
    char* final;

    Oled_titulo("Active Power"); 
    
   
    #if defined(ACTIVE_POWER_DISPLAY_IN_KW)
        f=1000;
        final= " KW"; 
    #else
        f=1;
        final = " W";
    #endif    


    x=active_power(FAKE_PHASE_TOTAL)/f;        
    
    Oled_display_valor(1,3,x,3,"Total: ",final);    
    
    x=active_power(0)/f;
    if (x==POWER_OVERRANGE)
        ssd1306_printText(3, 5,"A:  == POWER_OVERRANGE =="); 
    else    
        Oled_display_valor(3,5,x,3,"A:   ",final);

    x=active_power(1)/f;
    if (x==POWER_OVERRANGE)
        ssd1306_printText(3, 6,"B:  == POWER_OVERRANGE =="); 
    else    
        Oled_display_valor(3,6,x,3,"B:   ",final);

    x=active_power(2)/f;
    if (x==POWER_OVERRANGE)
        ssd1306_printText(3, 7,"C:  == POWER_OVERRANGE =="); 
    else    
        Oled_display_valor(3,7,x,3,"C:   ",final);    
    

    
}

#if defined(REACTIVE_POWER_SUPPORT)  ||  defined(TOTAL_REACTIVE_POWER_SUPPORT)

    static __inline__ void oled_reactive_power()
    {

        /* Display reactive power in 0.01W increments */

        power_t x;
        Oled_titulo(" Reactive Power");  

        Oled_display_valor(1,3,reactive_power(FAKE_PHASE_TOTAL),3,"Total: "," W");    

        x=reactive_power(0);
        if (x==POWER_OVERRANGE)
            ssd1306_printText(3, 5,"A:  == POWER_OVERRANGE =="); 
        else    
            Oled_display_valor(3,5,x,3,"A:   "," W");

        x=reactive_power(1);
        if (x==POWER_OVERRANGE)
            ssd1306_printText(3, 6,"B:  == POWER_OVERRANGE =="); 
        else    
            Oled_display_valor(3,6,x,3,"B:   "," W");

        x=reactive_power(2);
        if (x==POWER_OVERRANGE)
            ssd1306_printText(3, 7,"C:  == POWER_OVERRANGE =="); 
        else    
            Oled_display_valor(3,7,x,3,"C:   "," W");    
    }
#endif

#if defined(APPARENT_POWER_SUPPORT)  ||  defined(TOTAL_APPARENT_POWER_SUPPORT)

    static __inline__ void oled_apparent_power()
    {

        power_t x;
        int f;
        char* final;

        #if defined(ACTIVE_POWER_DISPLAY_IN_KW) 
            f=1000;
            final= " KW"; 
        #else
            f=1;
            final = " W";
        #endif    

        Oled_titulo("Apparent Power");  

        Oled_display_valor(1,3,apparent_power(FAKE_PHASE_TOTAL),3,"Total: ",final);    

        x=apparent_power(0)/f;
        if (x==POWER_OVERRANGE)
            ssd1306_printText(3, 5,"A:  == POWER_OVERRANGE =="); 
        else    
            Oled_display_valor(3,5,x,3,"A:   ",final);

        x=apparent_power(1)/f;
        if (x==POWER_OVERRANGE)
            ssd1306_printText(3, 6,"B:  == POWER_OVERRANGE =="); 
        else    
            Oled_display_valor(3,6,x,3,"B:   ",final);

        x=apparent_power(2)/f;
        if (x==POWER_OVERRANGE)
            ssd1306_printText(3, 7,"C:  == POWER_OVERRANGE =="); 
        else    
            Oled_display_valor(3,7,x,3,"C:   ",final); 

    }
#endif

#if defined(IRMS_SUPPORT)  &&  defined(VRMS_SUPPORT)  &&  defined(POWER_FACTOR_SUPPORT)

    static __inline__ void oled_power_factor()
    {
        power_factor_t x;
        char* final;

        // TODO Verificar sinal

        Oled_titulo("Power Factor");

        x=power_factor(0);
        if (x<0) {
        final=" L";
        x=-x;
        }  
        else
        final=" C";  
        Oled_display_valor(4,3,x,4,"A: ",final);

        x=power_factor(1);
        if (x<0) {
        final=" L";
        x=-x;
        }  
        else
        final=" C";  
        Oled_display_valor(4,5,x,4,"B: ",final);

        x=power_factor(2);
        if (x<0) {
        final=" L";
        x=-x;
        }  
        else
        final=" C";
        Oled_display_valor(4,7,x,4,"C: ",final);    

    }
#endif



#if defined(VRMS_SUPPORT)

    static __inline__ void oled_rms_voltage()
    {
        rms_voltage_t x;

        /* Display RMS voltage in 0.1V or 0.01V increments */

        Oled_titulo("Volts RMS");    
        
        x=rms_voltage(0);
        if (x==RMS_VOLTAGE_OVERRANGE)
            ssd1306_printText(4, 3,"A:  == OVERVOLTAGE =="); 
        else    
            Oled_display_valor(4,3,x,3,"A: "," V");

        x=rms_voltage(1);
        if (x==RMS_VOLTAGE_OVERRANGE)
            ssd1306_printText(4, 5,"B:  == OVERVOLTAGE =="); 
        else    
            Oled_display_valor(4,5,x,3,"B: "," V");

        x=rms_voltage(2);
        if (x==RMS_VOLTAGE_OVERRANGE)
            ssd1306_printText(4, 7,"C:  == OVERVOLTAGE =="); 
        else    
            Oled_display_valor(4,7,x,3,"C: "," V");
    }
#endif    
    
#if defined(IRMS_SUPPORT)

    static __inline__ void oled_rms_current()
    {
        rms_current_t x;

        /* Display RMS current in 1mA increments */

        Oled_titulo("I RMS");    
        
        x=rms_current(0);
        if (x==RMS_CURRENT_OVERRANGE)
            ssd1306_printText(4, 3,"A:  == OVERCURRENT =="); 
        else    
            Oled_display_valor(4,3,x,3,"A: "," A");

        x=rms_current(1);
        if (x==RMS_CURRENT_OVERRANGE)
            ssd1306_printText(4, 5,"B:  == OVERCURRENT =="); 
        else    
            Oled_display_valor(4,5,x,3,"B: "," A");

        x=rms_current(2);
        if (x==RMS_CURRENT_OVERRANGE)
            ssd1306_printText(4, 7,"C:  == OVERCURRENT =="); 
        else    
            Oled_display_valor(4,7,x,3,"C: "," A");
    }
#endif    
 

#if defined(MAINS_FREQUENCY_SUPPORT)

static __inline__ void oled_mains_frequency()
    {

        /* Display mains frequency in 0.1Hz or 0.01Hz increments */

        Oled_titulo("FREQUENCIA"); 
            
        Oled_display_valor(4,3,mains_frequency(0),2,"A:  "," Hz");
        Oled_display_valor(4,5,mains_frequency(1),2,"B:  "," Hz");
        Oled_display_valor(4,7,mains_frequency(2),2,"C:  "," Hz");      

    }
#endif

#if defined(ACTIVE_ENERGY_SUPPORT)  ||  defined(TOTAL_ACTIVE_ENERGY_SUPPORT)

static __inline__ void oled_imported_active_energy()
    {
        int f;
        char* final;

        #if defined(ACTIVE_POWER_DISPLAY_IN_KW)
            f=1000;
            final= " KWh"; 
        #else
            f=1;
            final = " Wh";
        #endif    

        Oled_titulo("Active Energy"); 
       
        long x=(long) energy_consumed[FAKE_PHASE_TOTAL][APP_ACTIVE_ENERGY_IMPORTED];
        Oled_display_valor(0,3,x/f,3,"Total: ",final); 
       
        
        x=(long) energy_consumed[0][APP_ACTIVE_ENERGY_IMPORTED];
        Oled_display_valor(3,5,x/f,3,"A:  ",final);
       
        x=(long) energy_consumed[1][APP_ACTIVE_ENERGY_IMPORTED];
        Oled_display_valor(3,6,x/f,3,"B:  ",final);
   

        x=(long) energy_consumed[2][APP_ACTIVE_ENERGY_IMPORTED];
        Oled_display_valor(3,7,x/f,3,"C:  ",final); 

        
        
        //Oled_display_valor(3,5,(long)energy_consumed[0][APP_ACTIVE_ENERGY_IMPORTED]/f,3,"A:  ",final);
        //Oled_display_valor(3,6,(long)energy_consumed[1][APP_ACTIVE_ENERGY_IMPORTED]/f,3,"B:  ",final);
        //Oled_display_valor(3,7,(long)energy_consumed[2][APP_ACTIVE_ENERGY_IMPORTED]/f,3,"C:  ",final); 
        
    }
#endif


#if defined(REACTIVE_ENERGY_SUPPORT)  ||  defined(TOTAL_REACTIVE_ENERGY_SUPPORT)

static __inline__ void oled_imported_reactive_energy()
{
    long x;

    /* Display per phase or total imported reactive energy */

    int f;
    char* final;

    #if defined(ACTIVE_POWER_DISPLAY_IN_KW)
        f=1000;
        final= " KW"; 
    #else
        f=1;
        final = " W";
    #endif    

    Oled_titulo("Reactive Energy");  

    x = (long) energy_consumed[FAKE_PHASE_TOTAL][APP_REACTIVE_ENERGY_QUADRANT_I] + (long) energy_consumed[FAKE_PHASE_TOTAL][APP_REACTIVE_ENERGY_QUADRANT_IV];
    x/=f;
    Oled_display_valor(0,3,x,3,"Total:  ",final);    

    x = (long) energy_consumed[0][APP_REACTIVE_ENERGY_QUADRANT_I] + (long) energy_consumed[0][APP_REACTIVE_ENERGY_QUADRANT_IV];
    x/=f;
    Oled_display_valor(3,5,x,3,"A:   ",final);

    x = (long) energy_consumed[1][APP_REACTIVE_ENERGY_QUADRANT_I] +(long)  energy_consumed[1][APP_REACTIVE_ENERGY_QUADRANT_IV];
    x/=f;
    Oled_display_valor(3,6,x,3,"B:   ",final);

    x = (long) energy_consumed[2][APP_REACTIVE_ENERGY_QUADRANT_I] + (long) energy_consumed[2][APP_REACTIVE_ENERGY_QUADRANT_IV];
    x/=f;
    Oled_display_valor(3,7,x,3,"C:   ",final); 
  
}
#endif




enum
{
    OLED_ITEM_DATE_TIM_TEMPERATURE,  
    
#if defined(VRMS_SUPPORT)    
    OLED_ITEM_VOLTAGE,
#endif    

#if defined(IRMS_SUPPORT)    
    OLED_ITEM_CURRENT,
#endif      

    OLED_ITEM_ACTIVE_POWER, // e total

#if defined(REACTIVE_POWER_SUPPORT)
    OLED_ITEM_REACTIVE_POWER,  // e total
#endif

#if defined(APPARENT_POWER_SUPPORT)
    OLED_ITEM_APPARENT_POWER, 
#endif

#if defined(POWER_FACTOR_SUPPORT)    
    OLED_ITEM_POWER_FACTOR,
#endif

#if defined(MAINS_FREQUENCY_SUPPORT)
    OLED_ITEM_MAINS_FREQUENCY,
#endif

#if defined(ACTIVE_ENERGY_SUPPORT)  ||  defined(TOTAL_ACTIVE_ENERGY_SUPPORT)
    OLED_ITEM_ACTIVE_ENERGY,
#endif    

#if defined(REACTIVE_ENERGY_SUPPORT)  ||  defined(TOTAL_REACTIVE_ENERGY_SUPPORT)    
    OLED_ITEM_REACTIVE_ENERGY,
#endif    

    OLED_ITEM_SELECT_RESTART
};

void display_oled_item(int item)
{
 
    switch (item)
    {
        case OLED_ITEM_DATE_TIM_TEMPERATURE:
            oled_date_temp();
            break;

        case OLED_ITEM_ACTIVE_POWER: // e total
            oled_active_power();
            break;
        #if defined(REACTIVE_POWER_SUPPORT)
            case OLED_ITEM_REACTIVE_POWER:  // e total
                oled_reactive_power();
                break;
        #endif    
        #if defined(APPARENT_POWER_SUPPORT)
            case OLED_ITEM_APPARENT_POWER:
                oled_apparent_power();
                break;
        #endif

        #if defined(POWER_FACTOR_SUPPORT)   
            case OLED_ITEM_POWER_FACTOR:
                oled_power_factor();
                break;
        #endif    

        #if defined(VRMS_SUPPORT)
            case OLED_ITEM_VOLTAGE:
                oled_rms_voltage();
                break;
        #endif    

        #if defined(IRMS_SUPPORT) 
            case OLED_ITEM_CURRENT:
                oled_rms_current();
                break;
        #endif

        #if defined(MAINS_FREQUENCY_SUPPORT)
            case OLED_ITEM_MAINS_FREQUENCY:
                oled_mains_frequency();
                break;
        #endif    

        #if defined(ACTIVE_ENERGY_SUPPORT)  ||  defined(TOTAL_ACTIVE_ENERGY_SUPPORT)
            case OLED_ITEM_ACTIVE_ENERGY:
                oled_imported_active_energy();
                break;
        #endif    

        #if defined(REACTIVE_ENERGY_SUPPORT)  ||  defined(TOTAL_REACTIVE_ENERGY_SUPPORT)    
    case OLED_ITEM_REACTIVE_ENERGY:
                oled_imported_reactive_energy();
                break;
        #endif    
    }        
}

void update_oled(void)
{
    ssd1306_clearDisplay();
    if (oled_step<0) {
        Oled_titulo("Aguarde"); 
        char oled[17];
        sprintf(oled,"%2d",-oled_step);
        ssd1306_printText(4,2,oled );
        oled_step++;
    }
    else
    {

      if (oled_step == OLED_ITEM_SELECT_RESTART)
              oled_step = 0;
      display_oled_item(oled_step++); 
     
/*
      // Teste de TX modbus

      uint8_t *buffer_tx_modbus;
      buffer_tx_modbus = ports[1].tx_msg.buf.uint8;
      
      buffer_tx_modbus[0] = 1; // Endere?o do escravo
      buffer_tx_modbus[1] = 0x03; // N?mero do comando
      buffer_tx_modbus[2] = 10; // N?mero de bytes a seguir

      // Simula 10 valores de dados
      for (int i = 0; i < 10; i++) {
          buffer_tx_modbus[3 + i] = i; // Dados simulados
      }
      uint16_t crc = CalculateCRC(buffer_tx_modbus, 13);
      buffer_tx_modbus[13] = crc & 0xFF; // LSB
      buffer_tx_modbus[14] = (crc >> 8) & 0xFF; // MSB  
*/
    }
   
    
    /* Now deal with things which are constantly displayed */
    #if defined(BATTERY_MONITOR_SUPPORT)  &&  defined(ICON_BATTERY)
        lcd_icon(ICON_BATTERY, (meter_status & STATUS_BATTERY_OK));
    #endif
}



#endif

