/*******************************************************************************
 *  emeter-setup.c -
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
#if !defined(__MSP430__)
#include <stdio.h>
#include <stdlib.h>
#endif
#if defined(__GNUC__)
#include <signal.h>
#endif
#if defined(__MSP430__)
#include <msp430.h>
#endif

#include "emeter-template.h"

#include <emeter-toolkit.h>
#include <hal_pmm.h>
#include <emeter-metrology.h>
#include <emeter-metrology-internal.h>

#include "emeter-app.h"
#include "emeter-rtc.h"
#include "emeter-lcd.h"
#include "emeter-communication.h"
#include "emeter-oled.h" 

static __inline__ void set_clock_slow(void)
{
#if defined(__MSP430_HAS_BC2__)
    BCSCTL1 &= ~XT2OFF;                 /* Activate XT2 high freq xtal */
    BCSCTL3 |= (XT2S_2 | LFXT1S_2);     /* 3-16MHz crystal or resonator */
    do
    {
        volatile int i;

        IFG1 &= ~OFIFG;                 /* Clear OSCFault flag */
        for (i = 0xFFF;  i > 0;  i--)   /* Time for flag to set */
            /*delay*/;
    }
    while (IFG1 & OFIFG);               /* OSCFault flag still set? */
    BCSCTL2 |= (SELS | SELM_2);         /* MCLK = XT2 HF XTAL (safe) */
#endif

#if defined(__MSP430_HAS_DCO__)
    /* Set up the DCO clock */
    BCSCTL1 |= (RSEL0 | RSEL1 | RSEL2); /* Select the highest nominal freq */
    BCSCTL2 |= DCOR;                    /* Select the external clock control resistor pin */
    DCOCTL = 0xFF;                      /* Select the highest speed. */
#endif

#if defined(__MSP430_HAS_FLLPLUS__)  ||  defined(__MSP430_HAS_FLLPLUS_SMALL__)
    FLL_CTL0 |= OSCCAP_3;               /* Configure load caps */
    #if defined(XT2OFF)
    FLL_CTL1 |= XT2OFF;
    #endif
    SCFI0 = SCFI0_LOW;
    SCFQCTL = SCFQCTL_LOW;
    FLL_CTL0 |= DCOPLUS;
    /* There seems no benefit in waiting for the FLL to settle at this point. */
#endif

#if defined(__MSP430_HAS_UCS__)
    SetVCore(3);
    __bis_SR_register(SCG0);                        /* Disable the FLL control loop */
    UCSCTL6 = (UCSCTL6 | XT1DRIVE_3);               /* Highest drive setting for XT1 startup */
    while (SFRIFG1 & OFIFG)
    {
        /* Check OFIFG fault flag */
        UCSCTL7 &= ~(DCOFFG | XT1LFOFFG | XT2OFFG); /* Clear OSC fault flags */
        SFRIFG1 &= ~OFIFG;                          /* Clear OFIFG fault flag */
    }
    UCSCTL6 = (UCSCTL6 & ~(XT1DRIVE_3));            /* Reduce drive to something weaker */

    UCSCTL0 = 0;
    UCSCTL1 = DCORSEL_6;                            /* Set RSELx for DCO = 25MHz */
    UCSCTL2 = FLLD_2 | (192 - 1);                   /* Set DCO Multiplier for 25MHz */
                                                    /* Set FLL to 32768*4*192 => 25165824Hz */
    __bic_SR_register(SCG0);                        /* Enable the FLL control loop */

    UCSCTL5 |= DIVS__1;
    UCSCTL4 = SELM__DCOCLK | SELS__DCOCLK | SELA__XT1CLK;     /* 24MHz MCLK, 24MHz SMCLK, 32KHz ACLK */
#endif

#if defined(__MSP430_HAS_CS__)
    /* DCOR select use of an external timing resistor */
    CSCTL0 = DCOR;
    //CSCTL0 = DCOBYP;
    //CSCTL0 = 0;
    /* SMCLK and MCLK divide by 1 */
    CSCTL1 = 0;

    CSIRFCAL = 0x99;
    CSIRTCAL = 0xC0;
    CSERFCAL = 0x8C;
    CSERTCAL = 0x00;
    
    REFCAL0 = 0x3B;
    REFCAL1 = 0x00;
    
    SD24TRIM = SD24REFDLYTRIM0 | 0x04;
#if 0
#if 0
    /* Clock system internal resistor frequency calibration */
    if ((ptr = tlv_find(TLV_CAL_CSIRFCAL)))
        CSIRFCAL = *ptr;
    /* Clock system internal resistor temperature calibration */
    if ((ptr = tlv_find(TLV_CAL_CSIRTCAL)))
        CSIRTCAL = *ptr;
    /* Clock system external resistor frequency calibration */
    if ((ptr = tlv_find(TLV_CAL_CSERFCAL)))
        CSERFCAL = *ptr;
    /* Clock system external resistor temperature calibration */
    if ((ptr = tlv_find(TLV_CAL_CSERTCAL)))
        CSERTCAL = *ptr;
#else
    /* Temporary approx value, in the absence of a real TLV table */
    CSIRFCAL = 0x99;
    CSIRTCAL = 0x00;
    CSERFCAL = 0x00;
    CSERTCAL = 0x00;
#endif
#endif
#endif
}

static __inline__ void set_clock_fast(void)
{
#if defined(__MSP430_HAS_FLLPLUS__)  ||  defined(__MSP430_HAS_FLLPLUS_SMALL__)
    /* Speed up the clock to 8.388608MHz */
    SCFI0 = SCFI0_HIGH;
    SCFQCTL = SCFQCTL_HIGH;
    #if 0
    {
        int i;
        for (i = 0xFFFF;  i;  i--);
            __no_operation();
        __bis_SR_register(SCG0);
    } 
    SCFQCTL |= SCFQ_M;
    SCFI0 &= ~0x03;
    SCFI1 &= ~0x07;
    SCFI1 += 8;
    #endif
    /* There seems no benefit in waiting for the FLL to settle at this point. */
#endif
}

#if defined(__MSP430__)
void system_setup(void)
{
    #if 0 //defined(__MSP430_HAS_TLV__)
    uint8_t *ptr;
    #endif

    #if defined(__MSP430_HAS_AUX_SUPPLY__)
    /* Enable the charger for AUXVCC3, to ensure the 32kHz oscillator is powered */
    PMMCTL0_H = PMMPW_H;  
    SVSMHCTL&=~SVSMHRRL_7;
    SVSMHCTL|=SVSMHRRL_4;
    AUX3CHCTL = AUXCHKEY | AUXCHEN | AUXCHC_1 | AUXCHV_1;
    AUXCTL0 = AUXKEY;
    AUXADCCTL = AUXADC | AUXADCSEL_0 | AUXADCR_0;
    AUXCTL2 |=AUX0LVL_6 +AUX1LVL_5 + AUX2LVL_5;
    AUXIE |= (AUX0SWIE | AUX1SWIE | AUX2SWIE | AUXSWGIE | AUX0DRPIE | AUX1DRPIE | AUX2DRPIE | AUXMONIE );
    PMMCTL0_H = 0;  
    #endif

    #if !defined(__MSP430_HAS_BT__)  &&  !defined(__MSP430_HAS_BT_RTC__)  &&  !defined(__MSP430_HAS_RTC_C__)  &&  !defined(__MSP430_HAS_RTC_CE__)
    WDTCTL = WDT_ADLY_1000;
        #if defined(__MSP430_HAS_SFR__)
    SFRIE1 |= WDTIE;                    /* Enable the WDT interrupt */
        #else
    IE1 |= WDTIE;                       /* Enable the WDT interrupt */
        #endif
    #endif
    
    set_clock_slow();

    #if defined(__MSP430_HAS_BT__)  ||  defined(__MSP430_HAS_BT_RTC__)
    /* Basic timer setup */
    /* Set ticker to 32768/(256*256) */
        #if defined(__MSP430_HAS_BT__)
    BTCTL = BT_fLCD_DIV64 | BT_fCLK2_DIV128 | BT_fCLK2_ACLK_DIV256;
        #else
    BTCTL = BT_fCLK2_DIV128 | BT_fCLK2_ACLK_DIV256;
        #endif
    /* Enable the 1 second counter interrupt */
    IE2 |= BTIE;
    #endif

    #if defined(__MSP430_HAS_BT__)  ||  defined(__MSP430_HAS_BT_RTC__)  ||  defined(__MSP430_HAS_RTC_C__)  ||  defined(__MSP430_HAS_RTC_CE__)
    /* We want a real watchdog function, but it doesn't have to be fast. */
    /* Use the longest timer - 1s */
        #if defined(USE_WATCHDOG)
    kick_watchdog();    /* Set the watchdog timer to exactly 1s */
        #else
    WDTCTL = (WDTCTL & 0xFF) | WDTPW | WDTHOLD;
        #endif
    #endif

    #if defined(P1OUT_INIT)
    P1OUT = P1OUT_INIT;
    #endif
    #if defined(P1DIR_INIT)
    //P1DIR = P1DIR_INIT; LI:
    P1DIR = 0x28;
    #endif
    #if defined(P1SEL_INIT)
    //P1SEL = P1SEL_INIT; LI:
    P1SEL = 0x3C;
    #endif
    #if defined(P1SEL0_INIT)
    P1SEL0 = P1SEL0_INIT;
    #endif
    #if defined(P1SEL1_INIT)
    P1SEL1 = P1SEL1_INIT;
    #endif
    #if defined(P1SEL2_INIT)
    P1SEL2 = P1SEL2_INIT;
    #endif
    #if defined(P1REN_INIT)
    P1REN = P1REN_INIT;
    #endif

    #if defined(P2OUT_INIT)
    P2OUT = P2OUT_INIT;
    #endif
    #if defined(P2DIR_INIT)
    P2DIR = P2DIR_INIT;
    P2DIR |= BIT3;
    #endif
    #if defined(P2SEL_INIT)
    P2SEL = P2SEL_INIT;
    P2SEL |= BIT2;
    #endif
    #if defined(P2SEL0_INIT)
    P2SEL0 = P2SEL0_INIT;
    #endif
    #if defined(P2SEL1_INIT)
    P2SEL1 = P2SEL1_INIT;
    #endif
    #if defined(P2SEL2_INIT)
    P2SEL2 = P2SEL2_INIT;
    #endif
    #if defined(P2REN_INIT)
    P2REN = P2REN_INIT;
    #endif

    #if defined(P3OUT_INIT)
    P3OUT = P3OUT_INIT;
    #endif
    #if defined(P3DIR_INIT)
    P3DIR = P3DIR_INIT;
    #endif
    #if defined(P3SEL_INIT)
    P3SEL = P3SEL_INIT;
    #endif
    #if defined(P3SEL0_INIT)
    P3SEL0 = P3SEL0_INIT;
    #endif
    #if defined(P3SEL1_INIT)
    P3SEL1 = P3SEL1_INIT;
    #endif
    #if defined(P3REN_INIT)
    P3REN = P3REN_INIT;
    #endif

    #if defined(P4OUT_INIT)
    P4OUT = P4OUT_INIT;
    #endif
    #if defined(P4DIR_INIT)
    P4DIR = P4DIR_INIT;
    #endif
    #if defined(P4SEL_INIT)
    P4SEL = P4SEL_INIT;
    #endif
    #if defined(P4SEL0_INIT)
    P4SEL0 = P4SEL0_INIT;
    #endif
    #if defined(P4SEL1_INIT)
    P4SEL1 = P4SEL1_INIT;
    #endif
    #if defined(P4REN_INIT)
    P4REN = P4REN_INIT;
    #endif

    #if defined(P5OUT_INIT)
    P5OUT = P5OUT_INIT;
    #endif
    #if defined(P5DIR_INIT)
    P5DIR = P5DIR_INIT;
    #endif
    #if defined(P5SEL_INIT)
    P5SEL = P5SEL_INIT;
    #endif
    #if defined(P5SEL0_INIT)
    P5SEL0 = P5SEL0_INIT;
    #endif
    #if defined(P5SEL1_INIT)
    P5SEL1 = P5SEL1_INIT;
    #endif
    #if defined(P5REN_INIT)
    P5REN = P5REN_INIT;
    #endif

    #if defined(P6OUT_INIT)
    P6OUT = P6OUT_INIT;
    #endif
    #if defined(P6DIR_INIT)
    P6DIR = P6DIR_INIT;
    #endif
    #if defined(P6SEL_INIT)
    P6SEL = P6SEL_INIT;
    #endif
    #if defined(P6REN_INIT)
    P6REN = P6REN_INIT;
    #endif

    #if defined(P7OUT_INIT)
    P7OUT = P7OUT_INIT;
    #endif
    #if defined(P7DIR_INIT)
    P7DIR = P7DIR_INIT;
    #endif
    #if defined(P7SEL_INIT)
    P7SEL = P7SEL_INIT;
    #endif
    #if defined(P7REN_INIT)
    P7REN = P7REN_INIT;
    #endif

    #if defined(P8OUT_INIT)
    P8OUT = P8OUT_INIT;
    #endif
    #if defined(P8DIR_INIT)
    P8DIR = P8DIR_INIT;
    #endif
    #if defined(P8SEL_INIT)
    P8SEL = P8SEL_INIT;
    #endif
    #if defined(P8REN_INIT)
    P8REN = P8REN_INIT;
    #endif

    #if defined(P9OUT_INIT)
    P9OUT = P9OUT_INIT;
    #endif
    #if defined(P9DIR_INIT)
    P9DIR = P9DIR_INIT;
    #endif
    #if defined(P9SEL_INIT)
    P9SEL = P9SEL_INIT;
    #endif
    #if defined(P9REN_INIT)
    P9REN = P9REN_INIT;
    #endif

    #if defined(P10OUT_INIT)
    P10OUT = P10OUT_INIT;
    #endif
    #if defined(P10DIR_INIT)
    P10DIR = P10DIR_INIT;
    #endif
    #if defined(P10SEL_INIT)
    P10SEL = P10SEL_INIT;
    #endif
    #if defined(P10SEL0_INIT)
    P10SEL0 = P10SEL0_INIT;
    #endif
    #if defined(P10SEL1_INIT)
    P10SEL1 = P10SEL1_INIT;
    #endif
    #if defined(P10REN_INIT)
    P10REN = P10REN_INIT;
    #endif

	#if defined(P11OUT_INIT)
	P11OUT = P11OUT_INIT;
	#endif
	#if defined(P11DIR_INIT)
	P11DIR = P11DIR_INIT;
	#endif
	#if defined(P11SEL_INIT)
	P11SEL = P11SEL_INIT;
	#endif
	#if defined(P11SEL0_INIT)
	P11SEL0 = P11SEL0_INIT;
	#endif
	#if defined(P11SEL1_INIT)
	P11SEL1 = P11SEL1_INIT;
	#endif
    #if defined(P11REN_INIT)
    P11REN = P11REN_INIT;
    #endif

	#if defined(P12OUT_INIT)
	P12OUT = P12OUT_INIT;
	#endif
	#if defined(P12DIR_INIT)
	P12DIR = P12DIR_INIT;
	#endif
	#if defined(P12SEL_INIT)
	P12SEL = P12SEL_INIT;
	#endif
	#if defined(P12SEL0_INIT)
	P12SEL0 = P12SEL0_INIT;
	#endif
	#if defined(P12SEL1_INIT)
	P12SEL1 = P12SEL1_INIT;
	#endif
    #if defined(P12REN_INIT)
    P12REN = P12REN_INIT;
    #endif

    #if defined(PJOUT_INIT)
    PJOUT = PJOUT_INIT;
    #endif
    #if defined(PJDIR_INIT)
    PJDIR = PJDIR_INIT;
    #endif
    #if defined(PJSEL_INIT)
    PJSEL = PJSEL_INIT;
    #endif
    #if defined(PJREN_INIT)
    PJREN = PJREN_INIT;
    #endif

    #if defined(CUSTOM_PORT_MAP)
    CUSTOM_PORT_MAP;
    #endif

    #if defined(LCD_DISPLAY_SUPPORT)
    lcd_init();
    display_startup_message();
    #endif
     
    #if defined(OLED_DISPLAY_SUPPORT)
    i2c_init();
    SSD1306_Init();
    OLED_display_startup_message();
    //Oled_teste();
    //__delay_cycles(100000000);
    #endif
    


    #if defined(IO_EXPANDER_SUPPORT)
    set_io_expander(0, 0);
    #endif

    #if defined(__MSP430_HAS_TA3__)  &&  defined(__MSP430_HAS_SD16_3__)
    /* Use timer A to control the ADC sampling interval in limp mode. */
    /* CCR0 determines the sample period - 1024Hz */
    TAR = 0;
    TACCR0 = 32 - 1;
    TACCTL0 = 0;
    TACCTL1 = OUTMOD_3;
    TACTL = TACLR | MC_1 | TASSEL_1;
    #endif


    
    metrology_init();
    metrology_disable_analog_front_end();
    

    #if defined(POWER_UP_BY_SUPPLY_SENSING)
    /* Set up comparator A to monitor a drooping voltage within the
       e-meter's main power supply. This is an early warning of power
       fail, so we can get to low power mode before we start burning the
       battery. */
    CACTL1 = CAREF_1;
    CACTL2 = P2CA1 | CAF;
    P1SEL |= BIT7;
    CAPD |= BIT7;
    #endif

    #if defined(UART_0_SUPPORT)
    serial_configure(0, 1, cfg_info->baud_rate);
    #endif
    #if defined(UART_1_SUPPORT)
    serial_configure(1, 1, UART_1_BAUD_RATE);
    #endif
    #if defined(UART_2_SUPPORT)
    serial_configure(2, 1, UART_2_BAUD_RATE);
    #endif
    #if defined(UART_3_SUPPORT)
    serial_configure(3, 1, UART_3_BAUD_RATE);
    #endif
    
    #if defined(IHD430_SUPPORT)
    IHD430_UART_configure();
    #endif

    #if defined(PHASE_REVERSED_DETECTION_SUPPORT)  ||  defined(PHASE_UNBALANCED_DETECTION_SUPPORT)
    clr_normal_indicator();
    #endif
    #if defined(PHASE_UNBALANCED_DETECTION_SUPPORT)
    clr_earthed_indicator();
    #endif
    #if defined(PHASE_REVERSED_DETECTION_SUPPORT)
    clr_reverse_current_indicator();
    #endif
    #if defined(TOTAL_ACTIVE_ENERGY_PULSES_PER_KW_HOUR)
    active_energy_pulse_end(FAKE_PHASE_TOTAL);
    #endif
    #if defined(TOTAL_REACTIVE_ENERGY_PULSES_PER_KVAR_HOUR)
    reactive_energy_pulse_end(FAKE_PHASE_TOTAL);
    #endif

    metrology_init_from_nv_data();

    #if defined(RTC_SUPPORT)
    rtc_init();
    #endif
    #if defined(EXTERNAL_EEPROM_SUPPORT)
    external_eeprom_init();
    #endif

    #if defined(IEC62056_21_SUPPORT)
    /* Configure the USART and 38kHz output bits */
    P2SEL |= (BIT5 | BIT4 | BIT3);
    P2DIR |= (BIT4 | BIT3);

    /* Configure the bit that powers the 38kHz receiver */
    P1DIR |= (BIT5);
    P1OUT |= (BIT5);

        #if defined(INFRA_RED_38K_FROM_TIMER_A)
    /* Program TA0 to output a 38kHz clock, based on the core frequency */
    TAR = 0;
    TACCR0 = IR_38K_DIVISOR - 1;        /* Load period register */
    TACCTL0 = OUTMOD_4;                 /* Set outmode 4 for toggle */
    TACTL = TACLR | MC_1 | TASSEL_2;    /* Start TIMER_B up mode, SMCLK as input clock */
        #endif
        #if defined(INFRA_RED_38K_FROM_TIMER_B)
    /* Program TB2 to output a 38kHz clock, based on the core frequency */
    TBR = 0;
    TBCCR0 = IR_38K_DIVISOR - 1;        /* Load period register */
    TBCCR2 = IR_38K_DIVISOR - 10;
    TBCCTL2 = OUTMOD_4;                 /* Set outmode 4 for toggle */
    TBCTL = TBCLR | MC_1 | TBSSEL_2;    /* Start TIMER_B up mode, SMCLK as input clock */
        #endif
    #endif

    __enable_interrupt();

    #if defined(POWER_DOWN_SUPPORT)
    /* Now go to lower power mode, until we know we should do otherwise */
    switch_to_powerfail_mode();
    #else
        #if defined(__MSP430_HAS_SVS__)  &&  !defined(__MSP430I4020__)
    /* Before we go to high speed we need to make sure the supply voltage is 
       adequate. If there is an SVS we can use that. There should be no wait
       at this point, since we should only have been woken up if the supply
       is healthy. However, it seems better to be cautious. */
    SVSCTL |= 0x60;
    /* Wait for adequate voltage to run at full speed */
    while (!(SVSCTL & SVSON))
        /* dummy loop */;
    while ((SVSCTL & SVSOP))
        /* dummy loop */;
    /* The voltage should now be OK to run the CPU at full speed. Now it should
       be OK to use the SVS as a reset source. */
    SVSCTL |= PORON;
        #endif

    /* Take control of the EEPROM signals. */
        #if defined(EXTERNAL_EEPROM_SUPPORT)
    enable_eeprom_port();
        #endif

    set_clock_fast();

    kick_watchdog();
    #endif
    metrology_switch_to_normal_mode();
}
#else
void system_setup(void)
{
    #if defined(TEMPERATURE_SUPPORT)
    temperature = 0;
    #endif
}
#endif

#if defined(__MSP430_HAS_AUX_SUPPLY__)
uint8_t present_power_source = 0;
uint8_t power_sources_status = 0;
uint16_t aux_ints = 0;

/* Auxiliary power module interrupt */
ISR(AUX, aux_interrupt)
{
    aux_ints++;
    switch (__even_in_range(AUXIV, 16))
    {
    case AUXIV_NONE:
        break;
    case AUXIV_AUXSWNMIFG:
        /* Global (non-)maskable supply switched interrupt flag */
        break;
    case AUXIV_AUX0SWIFG:
        /* Switched to DVCC interrupt flag */
        present_power_source = 0;
        break;
    case AUXIV_AUX1SWIFG:
        /* Switched to AUXVCC1 interrupt flag */
        present_power_source = 1;
        break;
    case AUXIV_AUX2SWIFG:
        /* Switched to AUXVCC2 interrupt flag */
        present_power_source = 2;
        break;
    case AUXIV_AUX0DRPIFG:
        power_sources_status |= 0x01;
        break;
    case AUXIV_AUX1DRPIFG:
        /* AUXVCC1 below threshold interrupt flag */
        power_sources_status |= 0x02;
        break;
    case AUXIV_AUX2DRPIFG:
        /* AUXVCC2 below threshold interrupt flag */
        power_sources_status |= 0x04;
        break;
    case AUXIV_AUXMONIFG:
        /* Supply monitor interrupt flag */
        break;
    }
}
#endif
