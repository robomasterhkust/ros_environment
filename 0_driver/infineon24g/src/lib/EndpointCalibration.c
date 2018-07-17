/**
 * \file EndpointCalibration.c
 *
 * No documentation is available for this file, as it is legacy code about to
 * be removed and just exists to keep the GUI compiling.
 */

/* ===========================================================================
** Copyright (C) 2018 Infineon Technologies AG
** All rights reserved.
** ===========================================================================
**
** ===========================================================================
** This document contains proprietary information of Infineon Technologies AG.
** Passing on and copying of this document, and communication of its contents
** is not permitted without Infineon's prior written authorisation.
** ===========================================================================
*/

/*
==============================================================================
   1. INCLUDE FILES
==============================================================================
*/
#include "EndpointCalibration.h"
#define __PROTOCOL_INCLUDE_ENDPOINT_ONLY_API__
#include "Protocol_internal.h"
#undef __PROTOCOL_INCLUDE_ENDPOINT_ONLY_API__
#include <stdlib.h>
#include <string.h>

/*
==============================================================================
   2. LOCAL DEFINITIONS
==============================================================================
*/

/**
 * A radar endpoint defines the following command codes.
 */
 
#define  MSG_SAVE_FLASH_CALIBRATION     0x01  /**< A message to save calibration data from EEPROM */

#define  MSG_SEND_FLASH_CALIBRATION     0x02  /**< A message to read calibration data from EEPROM */

#define  MSG_CLEAR_FLASH_CALIBRATION    0x03  /**< A message to delete calibration data from EEPROM */

#define  MSG_SAVE_SRAM_CALIBRATION      0x04  /**< A message to save calibration data in SRAM */

#define  MSG_SEND_SRAM_CALIBRATION      0x05  /**< A message to read calibration data in SRAM */

#define  MSG_CLEAR_SRAM_CALIBRATION     0x06  /**< A message to delete calibration data from SRAM */

/*
==============================================================================
   5. LOCAL FUNCTION PROTOTYPES
==============================================================================
*/
extern const char* ep_radar_get_error_code_description(uint16_t error_code);

static void parse_payload(int32_t protocol_handle, uint8_t endpoint,
                          const uint8_t* payload, uint16_t payload_size);

static int32_t parse_calibration_data(int32_t protocol_handle,
                                      uint8_t endpoint,
                                      const uint8_t* payload,
                                      uint16_t payload_size);

/*
==============================================================================
   4. DATA
==============================================================================
*/

const Endpoint_Definition_t ep_calibration_definition =
{
    /*.type             =*/ 0x5243414c, /* ASCII code 'RCAL'*/
    /*.min_version      =*/ 1,
    /*.max_version      =*/ 1,
    /*.description      =*/ "ifxRadar Calibration",
    /*.parse_payload    =*/ parse_payload,
    /*.get_status_descr =*/ ep_radar_get_error_code_description
};

static Callback_Calibration_Data_t callback_calibration_data = NULL;
static void* context_calibration_data = NULL;

/*
==============================================================================
  6. LOCAL FUNCTIONS
==============================================================================
*/

static void parse_payload(int32_t protocol_handle, uint8_t endpoint,
                          const uint8_t* payload, uint16_t payload_size)
{
    /* try to parse payload for all supported message types, stop when parsing
     * was successful
     */
    if (parse_calibration_data(protocol_handle, endpoint,
                               payload, payload_size))
        return;
}

static int32_t parse_calibration_data(int32_t protocol_handle,
                                      uint8_t endpoint,
                                      const uint8_t* payload,
                                      uint16_t payload_size)
{
    if (((protocol_read_payload_uint8(payload, 0) == MSG_SEND_FLASH_CALIBRATION) || 
		(protocol_read_payload_uint8( payload, 0) == MSG_SEND_SRAM_CALIBRATION)) && (payload_size >= 2))
    {
        if (callback_calibration_data)
        {
            uint16_t num_of_bytes;
            uint16_t total_samples;
            uint16_t current_sample;
            uint16_t read_idx = 3;
            uint16_t sample_bit_mask;
            float norm_factor;
            float* sample_data;
            float* write_ptr;

            num_of_bytes = protocol_read_payload_uint16(payload,  1);
            total_samples = (num_of_bytes >> 1);

            sample_bit_mask = (1 << 12) - 1;
            norm_factor = 1.0f / (float)(sample_bit_mask);

            if (payload_size  == read_idx + total_samples * 2)
            {
                sample_data = (float*)malloc(total_samples * sizeof(float));
                write_ptr = sample_data;

                while (total_samples)
                {
                    current_sample = protocol_read_payload_uint16(payload, read_idx);
                    *write_ptr++ = (current_sample & sample_bit_mask) * norm_factor;
                    read_idx += 2;
                    --total_samples;
                }

                /* send Calibration data array to callback */
                total_samples = num_of_bytes >> 1;
                callback_calibration_data(context_calibration_data,
                                          protocol_handle, endpoint,
                                          sample_data, total_samples);
                free(sample_data);
            }
        }
        return 1;
    }
    return 0;
}

/*
==============================================================================
   7. EXPORTED FUNCTIONS
==============================================================================
*/

int32_t ep_calibration_is_compatible_endpoint(int32_t protocol_handle,
											  uint8_t endpoint)
{
	return protocol_is_endpoint_compatible(protocol_handle, endpoint,
		&ep_calibration_definition);
}

void ep_calibration_set_callback_calibration_data(Callback_Calibration_Data_t
                                                       callback,
                                                   void* context)
{
    callback_calibration_data = callback;
    context_calibration_data = context;
}

int32_t ep_calibration_set_calibration_data(int32_t protocol_handle,
                                         uint8_t endpoint)
{
    /* build message to send */
    uint8_t cmd_message[1];

    protocol_write_payload_uint8(cmd_message, 0, MSG_SAVE_FLASH_CALIBRATION);

    /* send message and process received response */
    return protocol_send_and_receive(protocol_handle, endpoint,
                                     &ep_calibration_definition,
                                     cmd_message, sizeof(cmd_message));
}


int32_t ep_calibration_set_sram_calibration_data(int32_t protocol_handle,
                                         uint8_t endpoint)
{
    /* build message to send */
    uint8_t cmd_message[1];

    protocol_write_payload_uint8(cmd_message, 0, MSG_SAVE_SRAM_CALIBRATION);

    /* send message and process received response */
    return protocol_send_and_receive(protocol_handle, endpoint,
                                     &ep_calibration_definition,
                                     cmd_message, sizeof(cmd_message));
}

int32_t ep_calibration_clear_calibration_data(int32_t protocol_handle,
                                               uint8_t endpoint )
{
    /* build message to send */
    uint8_t cmd_message[1];

    protocol_write_payload_uint8(cmd_message, 0, MSG_CLEAR_FLASH_CALIBRATION);

    /* send message and process received response */
    return protocol_send_and_receive(protocol_handle, endpoint,
                                     &ep_calibration_definition,
                                     cmd_message, sizeof(cmd_message));
}

int32_t ep_calibration_clear_sram_calibration_data(int32_t protocol_handle,
                                               uint8_t endpoint )
{
    /* build message to send */
    uint8_t cmd_message[1];

    protocol_write_payload_uint8(cmd_message, 0, MSG_CLEAR_SRAM_CALIBRATION);

    /* send message and process received response */
    return protocol_send_and_receive(protocol_handle, endpoint,
                                     &ep_calibration_definition,
                                     cmd_message, sizeof(cmd_message));
}

int32_t ep_calibration_get_calibration_data(int32_t protocol_handle,
                                             uint8_t endpoint)
{
    /* build message to send */
    uint8_t cmd_message[1];

    protocol_write_payload_uint8 (cmd_message, 0, MSG_SEND_FLASH_CALIBRATION);

    /* send message and process received response */
    return protocol_send_and_receive(protocol_handle, endpoint,
                                     &ep_calibration_definition,
                                     cmd_message, sizeof(cmd_message));
}

int32_t ep_calibration_get_sram_calibration_data(int32_t protocol_handle,
                                             uint8_t endpoint)
{
    /* build message to send */
    uint8_t cmd_message[1];

    protocol_write_payload_uint8 (cmd_message, 0, MSG_SEND_SRAM_CALIBRATION);

    /* send message and process received response */
    return protocol_send_and_receive(protocol_handle, endpoint,
                                     &ep_calibration_definition,
                                     cmd_message, sizeof(cmd_message));
}
/* --- End of File -------------------------------------------------------- */
