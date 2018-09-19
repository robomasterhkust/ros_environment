/**
 * \file EndpointTargetDetection.c
 *
 * No documentation is available for this file, as it is legacy code about to
 * be removed and just exists to keep the GUI compiling.
 */

/* ===========================================================================
** Copyright (C) 2016-2017 Infineon Technologies AG
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
#include "EndpointTargetDetection.h"
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
 
#define  MSG_GET_DSP_SETTINGS           0x00  /**< A message to retrieve dsp settings */

#define  MSG_SET_DSP_SETTINGS           0x01  /**< A message to set dsp settings */

#define  MSG_GET_TARGETS                0x02  /**< A message to get targets */

/*
==============================================================================
   5. LOCAL FUNCTION PROTOTYPES
==============================================================================
*/
extern const char* ep_radar_get_error_code_description(uint16_t error_code);

static void parse_payload(int32_t protocol_handle, uint8_t endpoint,
                          const uint8_t* payload, uint16_t payload_size);

static int32_t parse_dsp_settings(int32_t protocol_handle, uint8_t endpoint,
                                  const uint8_t* payload,
                                  uint16_t payload_size);

static int32_t parse_target_info(int32_t protocol_handle, uint8_t endpoint,
                                 const uint8_t* payload,
                                 uint16_t payload_size);

/*
==============================================================================
   4. DATA
==============================================================================
*/

const Endpoint_Definition_t ep_targetdetect_definition =
{
    /*.type             =*/ 0x52544443, /* ASCII code 'RTDC'*/
    /*.min_version      =*/ 1,
    /*.max_version      =*/ 1,
    /*.description      =*/ "ifxRadar Target Detection",
    /*.parse_payload    =*/ parse_payload,
    /*.get_status_descr =*/ ep_radar_get_error_code_description
};

static Callback_Dsp_Settings_t callback_dsp_settings = NULL;
static void* context_dsp_settings = NULL;

static Callback_Target_Processing_t callback_target_processing = NULL;
static void* context_target_processing = NULL;

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
    if (parse_dsp_settings(protocol_handle, endpoint, payload, payload_size))
        return;

    if (parse_target_info(protocol_handle, endpoint, payload, payload_size))
        return;
}

static int32_t parse_target_info(int32_t protocol_handle, uint8_t endpoint,
                                 const uint8_t* payload,
                                 uint16_t payload_size)
{
    if ((protocol_read_payload_uint8(payload, 0) == MSG_GET_TARGETS) &&
        (payload_size >= 2))
    {
        // callbacks for target info are not defined yet
        if (callback_target_processing)
        {
            Target_Info_t* targets;
            uint8_t i;

            uint8_t num_targets = protocol_read_payload_uint8(payload, 1);

            const uint16_t target_data_size = sizeof(float) * 7 +
                                              sizeof(uint32_t);

            if (payload_size  == num_targets * target_data_size + 2)
            {
                targets = (Target_Info_t*)malloc(num_targets *
                                                 sizeof(Target_Info_t));

                for (i = 0; i < num_targets; ++i)
                {
                    targets[i] = *(Target_Info_t*)(payload + 2 +
                                                   i * target_data_size);
                }

                /* send target info to callback */
                callback_target_processing(context_target_processing,
                                           protocol_handle, endpoint,
                                           targets, num_targets);

                free (targets);
            }
        }
        return 1;
    }
    return 0;
}

static int32_t parse_dsp_settings(int32_t protocol_handle, uint8_t endpoint,
                                  const uint8_t* payload,
                                  uint16_t payload_size)
{
    if ((protocol_read_payload_uint8(payload, 0) == MSG_GET_DSP_SETTINGS) &&
        (payload_size == 18))
    {
        if (callback_dsp_settings)
        {
            DSP_Settings_t dsp_settings;

            dsp_settings.arithmetic_mean_count = *(uint8_t*) (payload +  1);
            dsp_settings.min_distance_cm =       *(uint16_t*)(payload +  2);
            dsp_settings.max_distance_cm =       *(uint16_t*)(payload +  4);
            dsp_settings.min_speed_kmh =         *(uint16_t*)(payload +  6);
            dsp_settings.max_speed_kmh =         *(uint16_t*)(payload +  8);
            dsp_settings.min_angle_degree =      *(uint16_t*)(payload + 10);
            dsp_settings.max_angle_degree =      *(uint16_t*)(payload + 12);
            dsp_settings.range_threshold =       *(uint16_t*)(payload + 14);
            dsp_settings.speed_threshold =       *(uint16_t*)(payload + 16);

            /* send DSP settings to callback */
            callback_dsp_settings(context_dsp_settings,
                                  protocol_handle, endpoint,
                                  &dsp_settings);
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

int32_t ep_targetdetect_is_compatible_endpoint(int32_t protocol_handle,
                                               uint8_t endpoint)
{
    return protocol_is_endpoint_compatible(protocol_handle, endpoint,
                                           &ep_targetdetect_definition);
}

void ep_targetdetect_set_callback_dsp_settings(Callback_Dsp_Settings_t
                                                 callback,
                                               void* context)
{
    callback_dsp_settings = callback;
    context_dsp_settings = context;
}

void ep_targetdetect_set_callback_target_processing(Callback_Target_Processing_t
                                                        callback,
                                                    void* context)
{
    callback_target_processing = callback;
    context_target_processing = context;
}

int32_t ep_targetdetect_set_dsp_settings(int32_t protocol_handle,
                                         uint8_t endpoint,
                                         const DSP_Settings_t* dsp_settings)
{
    /* build message to send */
    uint8_t cmd_message[18];

    protocol_write_payload_uint8 (cmd_message,  0, MSG_SET_DSP_SETTINGS);
    protocol_write_payload_uint16(cmd_message,  1, dsp_settings->arithmetic_mean_count);
    protocol_write_payload_uint16(cmd_message,  2, dsp_settings->min_distance_cm);
    protocol_write_payload_uint16(cmd_message,  4, dsp_settings->max_distance_cm);
    protocol_write_payload_uint16(cmd_message,  6, dsp_settings->min_speed_kmh);
    protocol_write_payload_uint16(cmd_message,  8, dsp_settings->max_speed_kmh);
    protocol_write_payload_uint16(cmd_message, 10, dsp_settings->min_angle_degree);
    protocol_write_payload_uint16(cmd_message, 12, dsp_settings->max_angle_degree);
    protocol_write_payload_uint16(cmd_message, 14, dsp_settings->range_threshold);
    protocol_write_payload_uint16(cmd_message, 16, dsp_settings->speed_threshold);

    /* send message and process received response */
    return protocol_send_and_receive(protocol_handle, endpoint,
                                     &ep_targetdetect_definition,
                                     cmd_message, sizeof(cmd_message));
}

int32_t ep_targetdetect_get_dsp_settings(int32_t protocol_handle,
                                          uint8_t endpoint)
{
    /* build message to send */
    uint8_t cmd_message[1];

    protocol_write_payload_uint8 (cmd_message, 0, MSG_GET_DSP_SETTINGS);

    /* send message and process received response */
    return protocol_send_and_receive(protocol_handle, endpoint,
                                     &ep_targetdetect_definition,
                                     cmd_message, sizeof(cmd_message));
}

int32_t ep_targetdetect_get_targets(int32_t protocol_handle, uint8_t endpoint)
{
    /* build message to send */
    uint8_t cmd_message[1];

    protocol_write_payload_uint8 (cmd_message, 0, MSG_GET_TARGETS);

    /* send message and process received response */
    return protocol_send_and_receive(protocol_handle, endpoint,
                                     &ep_targetdetect_definition,
                                     cmd_message, sizeof(cmd_message));
}

/* --- End of File -------------------------------------------------------- */
