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

#if !defined(NULL)
#define NULL    (void *) 0
#endif

static uint8_t comms_status;

enum
{
    DLT645_STATUS_PASSWORD_OK = 0x01
};

static const uint8_t address[6] =
{
    0x99, 0x99, 0x99, 0x99, 0x99, 0x99
};

uint16_t *next_flash_loc;

int prepare_tx_message(int port, int len)
{
    int i;
    uint8_t *s;

    s = ports[port].tx_msg.buf.uint8;
    s[0] = 0xFE;
    s[1] = 0xFE;
    s[2] = 0xFE;
    s[3] = 0xFE;
    s[4] = 0x68;
    s[5] = address[0];
    s[6] = address[1];
    s[7] = address[2];
    s[8] = address[3];
    s[9] = address[4];
    s[10] = address[5];
    s[11] = 0x68;
    s[12] = 0x23;
    s[13] = len;
    s[DLT645_PREAMBLE_BYTES + DLT645_MESSAGE_HEADER_BYTES + len] = 0;
    s[DLT645_PREAMBLE_BYTES + DLT645_MESSAGE_HEADER_BYTES + len + 1] = 0x16;
    for (i = DLT645_PREAMBLE_BYTES;  i < DLT645_PREAMBLE_BYTES + DLT645_MESSAGE_HEADER_BYTES + len;  i++)
        s[DLT645_PREAMBLE_BYTES + DLT645_MESSAGE_HEADER_BYTES + len] += s[i];
    len = DLT645_PREAMBLE_BYTES + DLT645_MESSAGE_HEADER_BYTES + len + DLT645_MESSAGE_TRAILER_BYTES;
    if (len > DLT645_PREAMBLE_BYTES + DLT645_MESSAGE_HEADER_BYTES + MAX_DLT645_MSG_BODY + DLT645_MESSAGE_TRAILER_BYTES)
        return false;
    serial_write(port, s, len);
    return  true;
}

static void send_consumption_report(int port, serial_msg_t *rx_msg, int rx_len, int phx)
{
    serial_msg_buf_t *tx;
    uint8_t *tx8;
#if !defined(NO_ENERGY_ACCUMULATION)
    uint16_t *tx16;
#endif

    tx = &ports[port].tx_msg;
    tx8 = &tx->buf.uint8[DLT645_PREAMBLE_BYTES + DLT645_MESSAGE_HEADER_BYTES];
#if !defined(NO_ENERGY_ACCUMULATION)
    tx16 = (uint16_t *) &tx->buf.uint8[DLT645_PREAMBLE_BYTES + DLT645_MESSAGE_HEADER_BYTES];
#endif

    tx8[0] = rx_msg->uint8[DLT645_MESSAGE_HEADER_BYTES];
    tx8[1] = rx_msg->uint8[DLT645_MESSAGE_HEADER_BYTES + 1] | 0x80;
#if !defined(NO_ENERGY_ACCUMULATION)
#if defined(ACTIVE_ENERGY_SUPPORT)
    *((uint64_t *) &tx16[1]) = energy_consumed[phx][APP_ACTIVE_ENERGY_IMPORTED];
    *((uint64_t *) &tx16[5]) = energy_consumed[phx][APP_ACTIVE_ENERGY_EXPORTED];
#else
    *((uint64_t *) &tx16[1]) = 0;
    *((uint64_t *) &tx16[5]) = 0;
#endif
#if defined(REACTIVE_ENERGY_SUPPORT)
    *((uint64_t *) &tx16[9]) = energy_consumed[phx][APP_REACTIVE_ENERGY_QUADRANT_I];
    *((uint64_t *) &tx16[13]) = energy_consumed[phx][APP_REACTIVE_ENERGY_QUADRANT_II];
    *((uint64_t *) &tx16[17]) = energy_consumed[phx][APP_REACTIVE_ENERGY_QUADRANT_III];
    *((uint64_t *) &tx16[21]) = energy_consumed[phx][APP_REACTIVE_ENERGY_QUADRANT_IV];
#else
    *((uint64_t *) &tx16[9]) = 0;
    *((uint64_t *) &tx16[13]) = 0;
    *((uint64_t *) &tx16[17]) = 0;
    *((uint64_t *) &tx16[21]) = 0;
#endif
#if defined(APPARENT_ENERGY_SUPPORT)
    *((uint64_t *) &tx16[25]) = energy_consumed[phx][APP_APPARENT_ENERGY_IMPORTED];
    *((uint64_t *) &tx16[29]) = energy_consumed[phx][APP_APPARENT_ENERGY_EXPORTED];
#else
    *((uint64_t *) &tx16[25]) = 0;
    *((uint64_t *) &tx16[29]) = 0;
#endif
#endif
    prepare_tx_message(port, 66);
}

static void send_consumption_report_aggregate(int port, serial_msg_t *rx_msg, int rx_len, int phx)
{
    serial_msg_buf_t *tx;
    uint8_t *tx8;
#if !defined(NO_ENERGY_ACCUMULATION)
    uint16_t *tx16;
#endif

    tx = &ports[port].tx_msg;
    tx8 = &tx->buf.uint8[DLT645_PREAMBLE_BYTES + DLT645_MESSAGE_HEADER_BYTES];
#if !defined(NO_ENERGY_ACCUMULATION)
    tx16 = (uint16_t *) &tx->buf.uint8[DLT645_PREAMBLE_BYTES + DLT645_MESSAGE_HEADER_BYTES];
#endif

    tx8[0] = rx_msg->uint8[DLT645_MESSAGE_HEADER_BYTES];
    tx8[1] = rx_msg->uint8[DLT645_MESSAGE_HEADER_BYTES + 1] | 0x80;
#if !defined(NO_ENERGY_ACCUMULATION)
#if defined(TOTAL_ACTIVE_ENERGY_SUPPORT)
    *((uint64_t *) &tx16[1]) = energy_consumed[phx][APP_ACTIVE_ENERGY_IMPORTED];
    *((uint64_t *) &tx16[5]) = energy_consumed[phx][APP_ACTIVE_ENERGY_EXPORTED];
#else
    *((uint64_t *) &tx16[1]) = 0;
    *((uint64_t *) &tx16[5]) = 0;
#endif
#if defined(TOTAL_REACTIVE_ENERGY_SUPPORT)
    *((uint64_t *) &tx16[9]) = energy_consumed[phx][APP_REACTIVE_ENERGY_QUADRANT_I];
    *((uint64_t *) &tx16[13]) = energy_consumed[phx][APP_REACTIVE_ENERGY_QUADRANT_II];
    *((uint64_t *) &tx16[17]) = energy_consumed[phx][APP_REACTIVE_ENERGY_QUADRANT_III];
    *((uint64_t *) &tx16[21]) = energy_consumed[phx][APP_REACTIVE_ENERGY_QUADRANT_IV];
#else
    *((uint64_t *) &tx16[9]) = 0;
    *((uint64_t *) &tx16[13]) = 0;
    *((uint64_t *) &tx16[17]) = 0;
    *((uint64_t *) &tx16[21]) = 0;
#endif
#if defined(TOTAL_APPARENT_ENERGY_SUPPORT)
    *((uint64_t *) &tx16[25]) = energy_consumed[phx][APP_APPARENT_ENERGY_IMPORTED];
    *((uint64_t *) &tx16[29]) = energy_consumed[phx][APP_APPARENT_ENERGY_EXPORTED];
#else
    *((uint64_t *) &tx16[25]) = 0;
    *((uint64_t *) &tx16[29]) = 0;
#endif
#endif
    prepare_tx_message(port, 66);
}


static void send_readings_report(int port, serial_msg_t *rx_msg, int rx_len, int phx)
{
    serial_msg_buf_t *tx;
    uint8_t *tx8;
    uint16_t *tx16;

    tx = &ports[port].tx_msg;
    tx8 = &tx->buf.uint8[DLT645_PREAMBLE_BYTES + DLT645_MESSAGE_HEADER_BYTES];
    tx16 = (uint16_t *) &tx->buf.uint8[DLT645_PREAMBLE_BYTES + DLT645_MESSAGE_HEADER_BYTES];

    tx8[0] = rx_msg->uint8[DLT645_MESSAGE_HEADER_BYTES];
    tx8[1] = rx_msg->uint8[DLT645_MESSAGE_HEADER_BYTES + 1] | 0x80;
    *((int32_t *) &tx16[1]) = rms_voltage(phx);
#if defined(IRMS_SUPPORT)
    #if NUM_PHASES == 1  &&  defined(NEUTRAL_MONITOR_SUPPORT)
    *((int32_t *) &tx16[3]) = rms_current(FAKE_PHASE_LIVE);
    #else
    *((int32_t *) &tx16[3]) = rms_current(phx);
    #endif
#else
    *((int32_t *) &tx16[3]) = 0;
#endif
#if NUM_PHASES == 1  &&  defined(NEUTRAL_MONITOR_SUPPORT)
    *((int32_t *) &tx16[5]) = active_power(FAKE_PHASE_LIVE);
#else
    *((int32_t *) &tx16[5]) = active_power(phx);
#endif
#if defined(REACTIVE_POWER_SUPPORT)
    *((int32_t *) &tx16[7]) = reactive_power(phx);
#else
    *((int32_t *) &tx16[7]) = 0;
#endif
#if defined(APPARENT_POWER_SUPPORT)
    *((int32_t *) &tx16[9]) = apparent_power(phx);
#else
    *((int32_t *) &tx16[9]) = 0;
#endif

#if defined(POWER_FACTOR_SUPPORT)
    tx16[11] = power_factor(phx);
#else
    tx16[11] = 0;
#endif
    tx16[12] = mains_frequency(phx);
#if !defined(ESP_SUPPORT)
    *((int32_t *) &tx16[13]) = voltage_dc_estimate(phx);
    *((int32_t *) &tx16[15]) = current_dc_estimate(phx);
#else
    *((int32_t *) &tx16[13]) = 0;
    *((int32_t *) &tx16[15]) = 0;
#endif
    prepare_tx_message(port, 34);
}

static void send_extra_readings_report(int port, serial_msg_t *rx_msg, int rx_len, int phx)
{
    serial_msg_buf_t *tx;
    uint8_t *tx8;
    uint16_t *tx16;

    tx = &ports[port].tx_msg;
    tx8 = &tx->buf.uint8[DLT645_PREAMBLE_BYTES + DLT645_MESSAGE_HEADER_BYTES];
    tx16 = (uint16_t *) &tx->buf.uint8[DLT645_PREAMBLE_BYTES + DLT645_MESSAGE_HEADER_BYTES];

    tx8[0] = rx_msg->uint8[DLT645_MESSAGE_HEADER_BYTES];
    tx8[1] = rx_msg->uint8[DLT645_MESSAGE_HEADER_BYTES + 1] | 0x80;
#if defined(FUNDAMENTAL_ACTIVE_POWER_SUPPORT)
    *((uint32_t *) &tx16[1]) = fundamental_active_power(phx);
#else
    *((uint32_t *) &tx16[1]) = 0;
#endif
#if defined(FUNDAMENTAL_REACTIVE_POWER_SUPPORT)
    *((uint32_t *) &tx16[3]) = fundamental_reactive_power(phx);
#else
    *((uint32_t *) &tx16[3]) = 0;
#endif
#if defined(FUNDAMENTAL_VRMS_SUPPORT)
    *((uint32_t *) &tx16[5]) = fundamental_rms_voltage(phx);
#else
    *((uint32_t *) &tx16[5]) = 0;
#endif
#if defined(FUNDAMENTAL_IRMS_SUPPORT)
    *((uint32_t *) &tx16[7]) = fundamental_rms_current(phx);
#else
    *((uint32_t *) &tx16[7]) = 0;
#endif
#if defined(VOLTAGE_THD_SUPPORT)
    tx16[9] = voltage_thd(phx);
#else
    tx16[9] = 0;
#endif
#if defined(CURRENT_THD_SUPPORT)
    tx16[10] = current_thd(phx);
#else
    tx16[10] = 0;
#endif
#if NUM_PHASES > 1  &&  defined(FUNDAMENTAL_VRMS_SUPPORT)
    tx16[11] = phase_to_phase_angle(phx);
#else
    tx16[11] = 0;
#endif
#if defined(SAG_SWELL_SUPPORT)
    tx16[12] = sag_events[phx];
    *((uint32_t *) &tx16[13]) = sag_duration[phx];
    tx16[15] = swell_events[phx];
    *((uint32_t *) &tx16[16]) = swell_duration[phx];
#else
    tx16[12] = 0;
    *((uint32_t *) &tx16[13]) = 0;
    tx16[15] = 0;
    *((uint32_t *) &tx16[16]) = 0;
#endif
    prepare_tx_message(port, 36);
}

static void dlt645_process_rx_message(int port, serial_msg_t *rx_msg, int rx_len)
{
    int32_t z;
    int32_t z1;
#if NUM_PHASES == 1
#define phase_no 0
#else
    int phase_no;
#endif
    uint16_t *last_flash_loc;
    serial_msg_buf_t *tx;
    uint8_t *tx8;
    uint16_t *tx16;

    tx = &ports[port].tx_msg;
    tx8 = &tx->buf.uint8[DLT645_PREAMBLE_BYTES + DLT645_MESSAGE_HEADER_BYTES];
    tx16 = (uint16_t *) &tx->buf.uint8[DLT645_PREAMBLE_BYTES + DLT645_MESSAGE_HEADER_BYTES];

    /* Messages with type 0x23 are custom messages we
      use for calibration, password protection, etc.
      All other message types go to a custom message
      handler (if available). */
    if (rx_msg->uint8[8] != 0x23)
    {
#if defined(CUSTOM_SERIAL_MESSAGE_SUPPORT)
        custom_serial_message_handler(&rx_msg, rx_msg_len);
#endif
        return;
    }
    if ((rx_msg->uint8[DLT645_MSG_RX_START_BODY + 1] & 0x80))
    {
        /* This looks like one of our own messages, which has echoed back
           to us */
        return;
    }

    /* Only process messages if the password has been given correctly
       (except for the password test message, of course). */
    if (!(comms_status & DLT645_STATUS_PASSWORD_OK)  &&  rx_msg->uint8[DLT645_MESSAGE_HEADER_BYTES] != HOST_CMD_SET_PASSWORD)
        return;

    switch (rx_msg->uint8[DLT645_MSG_RX_START_BODY])
    {
    case HOST_CMD_GET_METER_CONFIGURATION:
        tx8[0] = rx_msg->uint8[DLT645_MSG_RX_START_BODY];
        tx8[1] = rx_msg->uint8[DLT645_MSG_RX_START_BODY + 1] | 0x80;
        tx8[2] = NUM_PHASES;
#if defined(NEUTRAL_MONITOR_SUPPORT)
        tx8[3] = 1;
#else
        tx8[3] = 0;
#endif
        tx8[4] = 0x00
#if defined(LIMP_MODE_SUPPORT)
               | 0x01
#endif
//#if defined(PHASE_CORRECTION_SUPPORT)
               | 0x02
//#endif
#if defined(DYNAMIC_PHASE_CORRECTION_SUPPORT)
               | 0x04
#endif
#if defined(RTC_SUPPORT)
               | 0x08
#endif
#if defined(CORRECTED_RTC_SUPPORT)
               | 0x10
#endif
#if defined(TEMPERATURE_SUPPORT)
               | 0x20
#endif
#if defined(MULTI_RATE_SUPPORT)
               | 0x80
#endif
               | 0x00;

        tx8[5] = MEASURES_ACTIVE_POWER
#if defined(REACTIVE_POWER_SUPPORT)  &&  !defined(REACTIVE_POWER_BY_QUADRATURE_SUPPORT)
               | MEASURES_TRIGONOMETRIC_REACTIVE_POWER
#endif
#if defined(APPARENT_POWER_SUPPORT)
               | MEASURES_APPARENT_POWER
#endif
#if defined(VRMS_SUPPORT)
               | MEASURES_VRMS
#endif
#if defined(IRMS_SUPPORT)
               | MEASURES_IRMS
#endif
#if defined(POWER_FACTOR_SUPPORT)
               | MEASURES_POWER_FACTOR
#endif
#if defined(MAINS_FREQUENCY_SUPPORT)
               | MEASURES_MAINS_FREQUENCY
#endif
#if defined(REACTIVE_POWER_SUPPORT)  &&  defined(REACTIVE_POWER_BY_QUADRATURE_SUPPORT)
               | MEASURES_QUADRATURE_REACTIVE_POWER
#endif
               | 0x00;
        tx8[6] = 0x00
#if defined(FUNDAMENTAL_ACTIVE_POWER_SUPPORT)
               | MEASURES_FUNDAMENTAL_ACTIVE_POWER
#endif
#if defined(FUNDAMENTAL_REACTIVE_POWER_SUPPORT)
               | MEASURES_FUNDAMENTAL_REACTIVE_POWER
#endif
#if defined(FUNDAMENTAL_VRMS_SUPPORT)
               | MEASURES_FUNDAMENTAL_VRMS
#endif
#if defined(FUNDAMENTAL_IRMS_SUPPORT)
               | MEASURES_FUNDAMENTAL_IRMS
#endif
#if defined(VOLTAGE_THD_SUPPORT)
               | MEASURES_VOLTAGE_THD
#endif
#if defined(CURRENT_THD_SUPPORT)
               | MEASURES_CURRENT_THD
#endif
#if defined(SAG_SWELL_SUPPORT)
               | MEASURES_SAG_SWELL
#endif
               | 0x00;
        tx8[7] = 0;
        tx16[4] = MAINS_NOMINAL_FREQUENCY;
        tx16[5] = MAINS_NOMINAL_VOLTAGE;
        tx16[6] = MAINS_BASIS_CURRENT;
        tx16[7] = MAINS_MAXIMUM_CURRENT;
        *((uint32_t *) &tx16[8]) = SAMPLES_PER_10_SECONDS*10;
        prepare_tx_message(port, 20);
        break;
    case HOST_CMD_SET_METER_CONSUMPTION:
        z = rx_msg->uint16[DLT645_MSG_RX_START_BODY_W + 1];
        z |= (int32_t) rx_msg->uint16[DLT645_MSG_RX_START_BODY_W + 2] << 16;
        z1 = rx_msg->uint16[DLT645_MSG_RX_START_BODY_W + 3];
        z1 |= (int32_t) rx_msg->uint16[DLT645_MSG_RX_START_BODY_W + 4] << 16;
        custom_set_consumption(z, z1);
        tx8[0] = rx_msg->uint8[DLT645_MSG_RX_START_BODY];
        tx8[1] = rx_msg->uint8[DLT645_MSG_RX_START_BODY + 1] | 0x80;
        prepare_tx_message(port, 2);
    case HOST_CMD_SET_RTC:
#if defined(RTC_SUPPORT)
        set_rtc(&rx_msg->uint8[DLT645_MSG_RX_START_BODY + 2]);
#endif
#if defined(MULTI_RATE_SUPPORT)
        multirate_align_with_rtc();
#endif
        tx8[0] = rx_msg->uint8[DLT645_MSG_RX_START_BODY];
        tx8[1] = rx_msg->uint8[DLT645_MSG_RX_START_BODY + 1] | 0x80;
        prepare_tx_message(port, 2);
        break;
    case HOST_CMD_GET_RTC:
        tx8[0] = rx_msg->uint8[DLT645_MSG_RX_START_BODY];
        tx8[1] = rx_msg->uint8[DLT645_MSG_RX_START_BODY + 1] | 0x80;
#if defined(RTC_SUPPORT)
        get_rtc(&tx8[2]);
#endif
#if defined(TEMPERATURE_SUPPORT)
        tx16[4] = temperature()*10;
#else
        tx16[4] = 0;
#endif
        prepare_tx_message(port, 10);
        break;
    case HOST_CMD_ALIGN_WITH_CALIBRATION_FACTORS:
        align_metrology_with_calibration_data();
        tx8[0] = rx_msg->uint8[DLT645_MSG_RX_START_BODY];
        tx8[1] = rx_msg->uint8[DLT645_MSG_RX_START_BODY + 1] | 0x80;
        prepare_tx_message(port, 2);
        break;
    case HOST_CMD_SET_PASSWORD:
        /* Check the calibration password */
        if (rx_msg->uint16[DLT645_MSG_RX_START_BODY_W + 1] == SERIAL_CALIBRATION_PASSWORD_1
            &&
            rx_msg->uint16[DLT645_MSG_RX_START_BODY_W + 2] == SERIAL_CALIBRATION_PASSWORD_2
            &&
            rx_msg->uint16[DLT645_MSG_RX_START_BODY_W + 3] == SERIAL_CALIBRATION_PASSWORD_3
            &&
            rx_msg->uint16[DLT645_MSG_RX_START_BODY_W + 4] == SERIAL_CALIBRATION_PASSWORD_4)
        {
            comms_status |= DLT645_STATUS_PASSWORD_OK;
            tx8[0] = rx_msg->uint8[DLT645_MSG_RX_START_BODY];
            tx8[1] = rx_msg->uint8[DLT645_MSG_RX_START_BODY + 1] | 0x80;
            prepare_tx_message(port, 2);
        }
        else
        {
            /* Only respond to a bad password, if the password was good before. That lets
               us know we have unset the password OK, but doesn't give any information to
               people trying to attack the meter. */
            if ((comms_status & DLT645_STATUS_PASSWORD_OK))
            {
                tx8[0] = rx_msg->uint8[DLT645_MSG_RX_START_BODY];
                tx8[1] = rx_msg->uint8[DLT645_MSG_RX_START_BODY + 1] | 0x80;
                prepare_tx_message(port, 2);
            }
            comms_status &= ~DLT645_STATUS_PASSWORD_OK;
        }
        break;
    case HOST_CMD_GET_READINGS_PHASE_1:
#if NUM_PHASES > 1
    case HOST_CMD_GET_READINGS_PHASE_2:
    case HOST_CMD_GET_READINGS_PHASE_3:
        /* Exchange voltage, current and power readings (neutral),
           frequency, power factor and reactive power readings. */
        phase_no = rx_msg->uint8[DLT645_MSG_RX_START_BODY] - HOST_CMD_GET_READINGS_PHASE_1;
#endif
        send_readings_report(port, rx_msg, rx_len, phase_no);
        break;
#if defined(NEUTRAL_MONITOR_SUPPORT)
    case HOST_CMD_GET_READINGS_NEUTRAL:
        /* Exchange voltage, current and power readings (neutral),
           frequency, power factor and reactive power readings. */
        //send_readings_report(port, rx_msg, rx_len, FAKE_PHASE_NEUTRAL);
        tx8[0] = rx_msg->uint8[DLT645_MSG_RX_START_BODY];
        tx8[1] = rx_msg->uint8[DLT645_MSG_RX_START_BODY + 1] | 0x80;
        *((uint32_t *) &tx16[1]) = 0;
        *((uint32_t *) &tx16[3]) = rms_current(FAKE_PHASE_NEUTRAL);
    #if NUM_PHASES == 1
        *((uint32_t *) &tx16[5]) = active_power(FAKE_PHASE_NEUTRAL);
    #else
        *((uint32_t *) &tx16[5]) = 0;
    #endif
        *((uint32_t *) &tx16[7]) = 0;
    #if NUM_PHASES == 1
        *((uint32_t *) &tx16[9]) = 0;
        tx16[11] = 0;
        tx16[12] = mains_frequency(FAKE_PHASE_NEUTRAL);
    #else
        #if defined(RESIDUAL_IRMS_SUPPORT)
        *((uint32_t *) &tx16[9]) = residual_3phase_rms_current();
        #else
        *((uint32_t *) &tx16[9]) = 0;
        #endif
        tx16[11] = 0;
        tx16[12] = 0;
    #endif
        tx16[13] = 0;
        tx16[14] = 0;
    #if defined(ESP_SUPPORT)
        *((int32_t *) &tx16[15]) = 0;
    #else
        *((int32_t *) &tx16[15]) = current_dc_estimate(FAKE_PHASE_NEUTRAL);
    #endif
        prepare_tx_message(port, 34);
        break;
#endif
    case HOST_CMD_GET_CONSUMPTION_PHASE_1:
#if NUM_PHASES > 1
    case HOST_CMD_GET_CONSUMPTION_PHASE_2:
    case HOST_CMD_GET_CONSUMPTION_PHASE_3:
        phase_no = rx_msg->uint8[DLT645_MSG_RX_START_BODY] - HOST_CMD_GET_CONSUMPTION_PHASE_1;
#endif
        send_consumption_report(port, rx_msg, rx_len, phase_no);
        break;
    case HOST_CMD_GET_CONSUMPTION_AGGREGATE:
        send_consumption_report_aggregate(port, rx_msg, rx_len, FAKE_PHASE_TOTAL);
        break;
    case HOST_CMD_GET_EXTRA_READINGS_PHASE_1:
#if NUM_PHASES > 1
    case HOST_CMD_GET_EXTRA_READINGS_PHASE_2:
    case HOST_CMD_GET_EXTRA_READINGS_PHASE_3:
        phase_no = rx_msg->uint8[DLT645_MSG_RX_START_BODY] - HOST_CMD_GET_EXTRA_READINGS_PHASE_1;
#endif
        send_extra_readings_report(port, rx_msg, rx_len, phase_no);
        break;
#if defined(NEUTRAL_MONITOR_SUPPORT)
    case HOST_CMD_GET_EXTRA_READINGS_NEUTRAL:
        /* Exchange voltage, current and power readings (neutral),
           frequency, power factor and reactive power readings. */
        tx8[0] = rx_msg->uint8[DLT645_MSG_RX_START_BODY];
        tx8[1] = rx_msg->uint8[DLT645_MSG_RX_START_BODY + 1] | 0x80;
        tx16[1] = 0;
        tx16[2] = 0;
        tx16[3] = 0;
        tx16[4] = 0;
        tx16[5] = 0;
        tx16[6] = 0;
        tx16[7] = 0;
        tx16[8] = 0;
        tx16[9] = 0;
        tx16[10] = 0;
        tx16[11] = 0;
        tx16[12] = 0;
        tx16[13] = 0;
        tx16[14] = 0;
        tx16[15] = 0;
        tx16[16] = 0;
        prepare_tx_message(port, 34);
        break;
#endif
    case HOST_CMD_SUMCHECK_MEMORY:
        /* Sumcheck a specified memory area, and return the resulting sumcheck value. */
        if (!is_calibration_enabled())
            break;
        next_flash_loc = (uint16_t *) (intptr_t) *((uint32_t *) &rx_msg->uint16[DLT645_MSG_RX_START_BODY_W + 1]);
        last_flash_loc = (uint16_t *) (intptr_t) *((uint32_t *) &rx_msg->uint16[DLT645_MSG_RX_START_BODY_W + 3]);
        tx8[0] = rx_msg->uint8[DLT645_MSG_RX_START_BODY];
        tx8[1] = rx_msg->uint8[DLT645_MSG_RX_START_BODY + 1] | 0x80;
        tx16[1] = 0;
        while (next_flash_loc < last_flash_loc)
            tx16[1] += *next_flash_loc++;
        tx16[1] += *next_flash_loc;
        prepare_tx_message(port, 4);
        break;
    case HOST_CMD_GET_RAW_ACTIVE_POWER_PHASE_1:
#if NUM_PHASES > 1
    case HOST_CMD_GET_RAW_ACTIVE_POWER_PHASE_2:
    case HOST_CMD_GET_RAW_ACTIVE_POWER_PHASE_3:
        phase_no = rx_msg->uint8[DLT645_MSG_RX_START_BODY] - HOST_CMD_GET_RAW_ACTIVE_POWER_PHASE_1;
#endif
        tx8[0] = rx_msg->uint8[DLT645_MSG_RX_START_BODY];
        tx8[1] = rx_msg->uint8[DLT645_MSG_RX_START_BODY + 1] | 0x80;
#if !defined(ESP_SUPPORT)
        *((uint64_t *) &tx16[2]) = dot_product(phase_no, DOT_PRODUCT_TYPE_P_ACTIVE, &tx16[1]);
        prepare_tx_message(port, 12);
#endif
        break;
#if defined(REACTIVE_POWER_BY_QUADRATURE_SUPPORT)
    case HOST_CMD_GET_RAW_REACTIVE_POWER_PHASE_1:
    #if NUM_PHASES > 1
    case HOST_CMD_GET_RAW_REACTIVE_POWER_PHASE_2:
    case HOST_CMD_GET_RAW_REACTIVE_POWER_PHASE_3:
        phase_no = rx_msg->uint8[DLT645_MSG_RX_START_BODY] - HOST_CMD_GET_RAW_REACTIVE_POWER_PHASE_1;
    #endif
        tx8[0] = rx_msg->uint8[DLT645_MSG_RX_START_BODY];
        tx8[1] = rx_msg->uint8[DLT645_MSG_RX_START_BODY + 1] | 0x80;
    #if !defined(ESP_SUPPORT)
        *((uint64_t *) &tx16[2]) = dot_product(phase_no, DOT_PRODUCT_TYPE_P_REACTIVE, &tx16[1]);
        prepare_tx_message(port, 12);
    #endif
        break;
#endif
#if NUM_PHASES == 1  &&  defined(NEUTRAL_MONITOR_SUPPORT)
    case HOST_CMD_GET_RAW_ACTIVE_POWER_NEUTRAL:
        tx8[0] = rx_msg->uint8[DLT645_MSG_RX_START_BODY];
        tx8[1] = rx_msg->uint8[DLT645_MSG_RX_START_BODY + 1] | 0x80;
    #if !defined(ESP_SUPPORT)
        *((uint64_t *) &tx16[2]) = dot_product(FAKE_PHASE_NEUTRAL, DOT_PRODUCT_TYPE_P_ACTIVE, &tx16[1]);
        prepare_tx_message(port, 12);
    #endif
        break;
    #if defined(REACTIVE_POWER_BY_QUADRATURE_SUPPORT)
    case HOST_CMD_GET_RAW_REACTIVE_POWER_NEUTRAL:
        tx8[0] = rx_msg->uint8[DLT645_MSG_RX_START_BODY];
        tx8[1] = rx_msg->uint8[DLT645_MSG_RX_START_BODY + 1] | 0x80;
        #if !defined(ESP_SUPPORT)
        *((uint64_t *) &tx16[2]) = dot_product(FAKE_PHASE_NEUTRAL, DOT_PRODUCT_TYPE_P_REACTIVE, &tx16[1]);
        prepare_tx_message(port, 12);
        #endif
        break;
    #endif
#endif
#if defined(CORRECTED_RTC_SUPPORT)
    case HOST_CMD_CHECK_RTC_ERROR:
        tx8[0] = rx_msg->uint8[DLT645_MSG_RX_START_BODY];
        tx8[1] = rx_msg->uint8[DLT645_MSG_RX_START_BODY + 1] | 0x80;
    #if !defined(__MSP430_HAS_RTC_C__)  &&  defined(RTC_SUPPORT)  &&  defined(CORRECTED_RTC_SUPPORT)
        #if defined(__MSP430_HAS_TA3__)
        z = assess_rtc_speed();
        #endif
    #endif
        tx16[1] = z;
        tx16[2] = z >> 16;
        prepare_tx_message(port, 6);
        break;
    case HOST_CMD_RTC_CORRECTION:
        tx8[0] = rx_msg->uint8[DLT645_MSG_RX_START_BODY];
        tx8[1] = rx_msg->uint8[DLT645_MSG_RX_START_BODY + 1] | 0x80;
    #if defined(RTC_SUPPORT)  &&  defined(CORRECTED_RTC_SUPPORT)
        tx16[1] = rtc_correction;
    #endif
        prepare_tx_message(port, 6);
        break;
#endif
#if defined(MULTI_RATE_SUPPORT)
    case HOST_CMD_MULTIRATE_SET_PARAMETERS:
        i = multirate_put(&rx_msg->uint8[DLT645_MSG_RX_START_BODY]);
        tx8[0] = rx_msg->uint8[DLT645_MSG_RX_START_BODY];
        tx8[1] = rx_msg->uint8[DLT645_MSG_RX_START_BODY + 1] | 0x80;
        tx16[1] = i;
        prepare_tx_message(port, 4);
        break;
    case HOST_CMD_MULTIRATE_GET_PARAMETERS:
        i = multirate_get(&rx_msg->uint8[DLT645_MSG_RX_START_BODY], tx8);
        prepare_tx_message(port, i);
        break;
    case HOST_CMD_MULTIRATE_CLEAR_USAGE:
        i = multirate_clear_usage(&rx_msg->uint8[DLT645_MSG_RX_START_BODY]);
        tx8[0] = rx_msg->uint8[DLT645_MSG_RX_START_BODY];
        tx8[1] = rx_msg->uint8[DLT645_MSG_RX_START_BODY + 1] | 0x80;
        tx16[1] = i;
        prepare_tx_message(port, 4);
        break;
    case HOST_CMD_MULTIRATE_GET_USAGE:
        i = multirate_get_usage(&rx_msg->uint8[DLT645_MSG_RX_START_BODY], tx8);
        prepare_tx_message(port, i);
        break;
#endif
    case HOST_CMD_CLEAR_CALIBRATION_DATA:
        if (!is_calibration_enabled())
            break;
        clear_calibration_data();
        tx8[0] = rx_msg->uint8[DLT645_MSG_RX_START_BODY];
        tx8[1] = rx_msg->uint8[DLT645_MSG_RX_START_BODY + 1] | 0x80;
        prepare_tx_message(port, 2);
        break;
#if defined(NEUTRAL_MONITOR_SUPPORT)
    case HOST_CMD_SET_CALIBRATION_NEUTRAL:
        rx_msg->uint8[DLT645_MSG_RX_START_BODY] = (uint8_t) HOST_CMD_SET_CALIBRATION_PHASE_1 + FAKE_PHASE_NEUTRAL;
#endif
    case HOST_CMD_SET_CALIBRATION_PHASE_1:
#if NUM_PHASES >= 2
    case HOST_CMD_SET_CALIBRATION_PHASE_2:
#endif
#if NUM_PHASES >= 3
    case HOST_CMD_SET_CALIBRATION_PHASE_3:
#endif
#if NUM_PHASES > 1
        phase_no = rx_msg->uint8[DLT645_MSG_RX_START_BODY] - HOST_CMD_SET_CALIBRATION_PHASE_1;
#endif
        if (!is_calibration_enabled())
            break;
        set_v_dc_estimate(phase_no, 0, rx_msg->uint16[DLT645_MSG_RX_START_BODY_W + 1]);
#if defined(LIMP_MODE_SUPPORT)
        set_v_dc_estimate(phase_no, 1, rx_msg->uint16[DLT645_MSG_RX_START_BODY_W + 2]);
#endif
        set_i_dc_estimate(phase_no, 0, rx_msg->uint16[DLT645_MSG_RX_START_BODY_W + 3]);
#if defined(LIMP_MODE_SUPPORT)
        set_i_dc_estimate(phase_no, 1, rx_msg->uint16[DLT645_MSG_RX_START_BODY_W + 4]);
#endif
        set_v_ac_offset(phase_no, ((uint32_t *) &rx_msg->uint16[DLT645_MSG_RX_START_BODY_W + 5])[0]);
        set_i_ac_offset(phase_no, ((uint32_t *) &rx_msg->uint16[DLT645_MSG_RX_START_BODY_W + 7])[0]);
        set_phase_corr(phase_no, rx_msg->uint16[DLT645_MSG_RX_START_BODY_W + 9]);
        set_V_rms_scaling(phase_no, 0, rx_msg->uint16[DLT645_MSG_RX_START_BODY_W + 10]);
#if defined(LIMP_MODE_SUPPORT)
        set_V_rms_scaling(phase_no, 1, rx_msg->uint16[DLT645_MSG_RX_START_BODY_W + 11]);
#endif
        set_I_rms_scaling(phase_no, 0, rx_msg->uint16[DLT645_MSG_RX_START_BODY_W + 12]);
#if defined(LIMP_MODE_SUPPORT)
        set_I_rms_scaling(phase_no, 1, rx_msg->uint16[DLT645_MSG_RX_START_BODY_W + 13]);
#endif
        set_P_scaling(phase_no, rx_msg->uint16[DLT645_MSG_RX_START_BODY_W + 14]);
        tx8[0] = rx_msg->uint8[DLT645_MSG_RX_START_BODY];
        tx8[1] = rx_msg->uint8[DLT645_MSG_RX_START_BODY + 1] | 0x80;
        prepare_tx_message(port, 2);
        break;
#if defined(NEUTRAL_MONITOR_SUPPORT)
    case HOST_CMD_GET_CALIBRATION_NEUTRAL:
        rx_msg->uint8[DLT645_MSG_RX_START_BODY] = (uint8_t) HOST_CMD_GET_CALIBRATION_PHASE_1 + FAKE_PHASE_NEUTRAL;
#endif
    case HOST_CMD_GET_CALIBRATION_PHASE_1:
#if NUM_PHASES >= 2
    case HOST_CMD_GET_CALIBRATION_PHASE_2:
#endif
#if NUM_PHASES >= 3
    case HOST_CMD_GET_CALIBRATION_PHASE_3:
#endif
#if NUM_PHASES > 1
        phase_no = rx_msg->uint8[DLT645_MSG_RX_START_BODY] - HOST_CMD_GET_CALIBRATION_PHASE_1;
#endif
        if (!is_calibration_enabled())
            break;
        tx8[0] = rx_msg->uint8[DLT645_MSG_RX_START_BODY];
        tx8[1] = rx_msg->uint8[DLT645_MSG_RX_START_BODY + 1] | 0x80;
        tx16[1] = get_v_dc_estimate(phase_no, 0);
#if defined(LIMP_MODE_SUPPORT)
        tx16[2] = get_v_dc_estimate(phase_no, 1);
#else
        tx16[2] = 0;
#endif
        tx16[3] = get_i_dc_estimate(phase_no, 0);
#if defined(LIMP_MODE_SUPPORT)
        tx16[4] = get_i_dc_estimate(phase_no, 1);
#else
        tx16[4] = 0;
#endif
        *((uint32_t *) &tx16[5]) = get_v_ac_offset(phase_no);
        *((uint32_t *) &tx16[7]) = get_i_ac_offset(phase_no);
        tx16[9] = get_phase_corr(phase_no);
        tx16[10] = get_V_rms_scaling(phase_no, 0);
#if defined(LIMP_MODE_SUPPORT)
        tx16[11] = get_V_rms_scaling(phase_no, 1);
#else
        tx16[11] = 0;
#endif
        tx16[12] = get_I_rms_scaling(phase_no, 0);
#if defined(LIMP_MODE_SUPPORT)
        tx16[13] = get_I_rms_scaling(phase_no, 1);
#else
        tx16[13] = 0;
#endif
        tx16[14] = get_P_scaling(phase_no);
        prepare_tx_message(port, 30);
        break;
    case HOST_CMD_SET_CALIBRATION_EXTRAS:
        set_calibration_status(rx_msg->uint16[1]);
#if defined(TEMPERATURE_SUPPORT)
        set_temperature_parameters(rx_msg->uint16[2], rx_msg->uint16[3], rx_msg->uint16[4]);
#endif
        tx8[0] = rx_msg->uint8[DLT645_MSG_RX_START_BODY];
        tx8[1] = rx_msg->uint8[DLT645_MSG_RX_START_BODY + 1] | 0x80;
        prepare_tx_message(port, 2);
        break;
    case HOST_CMD_GET_CALIBRATION_EXTRAS:
        tx8[0] = rx_msg->uint8[DLT645_MSG_RX_START_BODY];
        tx8[1] = rx_msg->uint8[DLT645_MSG_RX_START_BODY + 1] | 0x80;
        tx16[1] = get_calibration_status();
        tx16[2] = DEFAULT_ROOM_TEMPERATURE;
#if defined(TEMPERATURE_SUPPORT)
        tx16[3] = get_temperature_intercept();
        tx16[4] = get_temperature_slope();
#else
        tx16[3] = 0;
        tx16[4] = 0;
#endif
        prepare_tx_message(port, 10);
        break;
    default:
        /* For all other message types reply with type 0xFF - bad message type */
        tx8[0] = rx_msg->uint8[DLT645_MSG_RX_START_BODY];
        tx8[1] = 0xFF;
        prepare_tx_message(port, 2);
        break;
    }
}

/* This routine is called regularly from the main polling loop, to check for completed incoming
   DLT-645 messages, and to service them. */
void dlt645_service(void)
{
    int port;

    for (port = 0;  port < MAX_UART_PORT;  port++)
    {
        if (ports[port].rx_frame_pending)
        {
            dlt645_process_rx_message(port, &ports[port].rx_msg.buf, ports[port].rx_msg.len);
            ports[port].rx_frame_pending = false;
        }
    }
}

/* This routine is called from within UART port interrupts, so it must be kept lean and mean. */
void dlt645_rx_byte(int port, uint8_t c)
{
    int i;
    int sum;

    if (ports[port].rx_frame_pending)
        return;
    //if (ports[port].rx_msg.inter_char_timeout == 0)
    //    ports[port].rx_msg.ptr = 0;
    ports[port].rx_msg.inter_char_timeout = SAMPLES_PER_10_SECONDS/200;
    if (ports[port].rx_msg.ptr == 0)
    {
      
        if (c == 0x68)
        {
            /* We have found a start of packet marker */
            ports[port].rx_msg.buf.uint8[ports[port].rx_msg.ptr++] = c;
            ports[port].rx_msg.len = 12 + MAX_DLT645_MSG_BODY;
        }
    }
    else
    {
        if (ports[port].rx_msg.ptr == 9)
        {
            /* We are at the length byte. If the length is in range, accept it.
               Otherwise we need to abandon this packet. */
            if (c <= MAX_DLT645_MSG_BODY)
                ports[port].rx_msg.len = 12 + c;
            else
                ports[port].rx_msg.ptr = 0;
        }
        ports[port].rx_msg.buf.uint8[ports[port].rx_msg.ptr++] = c;
        if (ports[port].rx_msg.ptr == ports[port].rx_msg.len)
        {
            /* This should be the end of the packet */
            if (ports[port].rx_msg.buf.uint8[ports[port].rx_msg.len - 1] == 0x16)
            {
                /* Check the sumcheck byte */
                sum = ports[port].rx_msg.buf.uint8[0];
                for (i = 1;  i < ports[port].rx_msg.len - 2;  i++)
                    sum += ports[port].rx_msg.buf.uint8[i];
                if (ports[port].rx_msg.buf.uint8[ports[port].rx_msg.len - 2] == (sum & 0xFF))
                {
                    /* Good packet received */
                    ports[port].rx_frame_pending = true;
                }
            }
            ports[port].rx_msg.ptr = 0;
        }
    }
      
}
