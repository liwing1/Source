/*******************************************************************************
 *  dlt645-decs.h -
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

#define DLT645_PREAMBLE_BYTES                   4

#define DLT645_MESSAGE_HEADER_BYTES             10

#define DLT645_MESSAGE_TRAILER_BYTES            2

/* The starting byte of the message body in a received DLT645 message. */
#define DLT645_MSG_RX_START_BODY                DLT645_MESSAGE_HEADER_BYTES
/* The starting 16 bit word of the message body in a received DLT645 message. */
#define DLT645_MSG_RX_START_BODY_W              (DLT645_MESSAGE_HEADER_BYTES >> 1)
/* The maximum DLT645 message body length to allow for when sizing buffers. */
#define MAX_DLT645_MSG_BODY                     66

enum
{
    MEASURES_ACTIVE_POWER                       = 0x0001,
    MEASURES_TRIGONOMETRIC_REACTIVE_POWER       = 0x0002,
    MEASURES_APPARENT_POWER                     = 0x0004,
    MEASURES_VRMS                               = 0x0008,
    MEASURES_IRMS                               = 0x0010,
    MEASURES_POWER_FACTOR                       = 0x0020,
    MEASURES_MAINS_FREQUENCY                    = 0x0040,
    MEASURES_QUADRATURE_REACTIVE_POWER          = 0x0080,
#if defined(MEASURES_BITS_16WIDE)
    MEASURES_FUNDAMENTAL_ACTIVE_POWER           = 0x0200,
    MEASURES_FUNDAMENTAL_REACTIVE_POWER         = 0x0400,
    MEASURES_FUNDAMENTAL_VRMS                   = 0x0800,
    MEASURES_FUNDAMENTAL_IRMS                   = 0x1000,
    MEASURES_VOLTAGE_THD                        = 0x2000,
    MEASURES_CURRENT_THD                        = 0x4000,
    MEASURES_SAG_SWELL                          = 0x8000
#endif
};

#if !defined(MEASURES_BITS_16WIDE)
enum
{
    MEASURES_WITH_MULTIPLE_GAIN_STAGES          = 0x01,
    MEASURES_FUNDAMENTAL_ACTIVE_POWER           = 0x02,
    MEASURES_FUNDAMENTAL_REACTIVE_POWER         = 0x04,
    MEASURES_FUNDAMENTAL_VRMS                   = 0x08,
    MEASURES_FUNDAMENTAL_IRMS                   = 0x10,
    MEASURES_VOLTAGE_THD                        = 0x20,
    MEASURES_CURRENT_THD                        = 0x40,
    MEASURES_SAG_SWELL                          = 0x80
};
#endif

enum host_commands_e
{
    HOST_CMD_GET_METER_CONFIGURATION            = 0x56,
    HOST_CMD_SET_METER_CONSUMPTION              = 0x57,
    HOST_CMD_SET_RTC                            = 0x58,
    HOST_CMD_GET_RTC                            = 0x59,
    HOST_CMD_ALIGN_WITH_CALIBRATION_FACTORS     = 0x5A,
    HOST_CMD_SET_PASSWORD                       = 0x60,
    HOST_CMD_GET_READINGS_PHASE_1               = 0x61,
    HOST_CMD_GET_READINGS_PHASE_2               = 0x62,
    HOST_CMD_GET_READINGS_PHASE_3               = 0x63,
    HOST_CMD_GET_READINGS_NEUTRAL               = 0x64,
    HOST_CMD_GET_CONSUMPTION_PHASE_1            = 0x65,
    HOST_CMD_GET_CONSUMPTION_PHASE_2            = 0x66,
    HOST_CMD_GET_CONSUMPTION_PHASE_3            = 0x67,
    HOST_CMD_GET_CONSUMPTION_AGGREGATE          = 0x68,
    HOST_CMD_GET_EXTRA_READINGS_PHASE_1         = 0x69,
    HOST_CMD_GET_EXTRA_READINGS_PHASE_2         = 0x6A,
    HOST_CMD_GET_EXTRA_READINGS_PHASE_3         = 0x6B,
    HOST_CMD_GET_EXTRA_READINGS_NEUTRAL         = 0x6C,
    HOST_CMD_GET_READINGS_AGGREGATE             = 0x6D,
    HOST_CMD_GET_EXTRA_READINGS_AGGREGATE       = 0x6E,
    HOST_CMD_SUMCHECK_MEMORY                    = 0x75,
    HOST_CMD_GET_RAW_ACTIVE_POWER_PHASE_1       = 0x91,
    HOST_CMD_GET_RAW_ACTIVE_POWER_PHASE_2       = 0x92,
    HOST_CMD_GET_RAW_ACTIVE_POWER_PHASE_3       = 0x93,
    HOST_CMD_GET_RAW_REACTIVE_POWER_PHASE_1     = 0x95,
    HOST_CMD_GET_RAW_REACTIVE_POWER_PHASE_2     = 0x96,
    HOST_CMD_GET_RAW_REACTIVE_POWER_PHASE_3     = 0x97,
    HOST_CMD_GET_RAW_ACTIVE_POWER_NEUTRAL       = 0x99,
    HOST_CMD_GET_RAW_REACTIVE_POWER_NEUTRAL     = 0x9D,
    HOST_CMD_CHECK_RTC_ERROR                    = 0xA0,
    HOST_CMD_RTC_CORRECTION                     = 0xA1,
    HOST_CMD_MULTIRATE_SET_PARAMETERS           = 0xC0,
    HOST_CMD_MULTIRATE_GET_PARAMETERS           = 0xC1,
    HOST_CMD_MULTIRATE_CLEAR_USAGE              = 0xC2,
    HOST_CMD_MULTIRATE_GET_USAGE                = 0xC3,
    HOST_CMD_CLEAR_CALIBRATION_DATA             = 0xD0,
    HOST_CMD_SET_CALIBRATION_PHASE_1            = 0xD1,
    HOST_CMD_SET_CALIBRATION_PHASE_2            = 0xD2,
    HOST_CMD_SET_CALIBRATION_PHASE_3            = 0xD3,
    HOST_CMD_SET_CALIBRATION_NEUTRAL            = 0xD4,
    HOST_CMD_SET_CALIBRATION_EXTRAS             = 0xD5,
    HOST_CMD_GET_CALIBRATION_PHASE_1            = 0xD6,
    HOST_CMD_GET_CALIBRATION_PHASE_2            = 0xD7,
    HOST_CMD_GET_CALIBRATION_PHASE_3            = 0xD8,
    HOST_CMD_GET_CALIBRATION_NEUTRAL            = 0xD9,
    HOST_CMD_GET_CALIBRATION_EXTRAS             = 0xDA
};
