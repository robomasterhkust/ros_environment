/*
  ==============================================================================

    EndPointTargetDetection.h
    Created: 17 Oct 2016 11:07:11am
    Author:  Assad Ali

  ==============================================================================
*/

#ifndef ENDPOINTTARGETDETECT_H_INCLUDED
#define ENDPOINTTARGETDETECT_H_INCLUDED

/*
==============================================================================
   1. INCLUDE FILES
==============================================================================
*/
#include <stdint.h>
#include "EndpointRadarErrorCodes.h"

/* Enable C linkage if header is included in C++ files */
#ifdef __cplusplus
extern "C"
{
#endif /* __cplusplus */

/*
==============================================================================
   2. DEFINITIONS
==============================================================================
*/

/*
==============================================================================
   3. TYPES
==============================================================================
*/

/**
 *
 * \brief Data structure for DSP settings for the target detection processing.
 *
 * @{
 */
typedef struct
{
    uint8_t  arithmetic_mean_count;
    uint16_t min_distance_cm;
    uint16_t max_distance_cm;
    uint16_t min_speed_kmh;
    uint16_t max_speed_kmh;
    uint16_t min_angle_degree;
    uint16_t max_angle_degree;
    uint16_t range_threshold;
    uint16_t speed_threshold;
} DSP_Settings_t;

typedef struct
{
    uint32_t target_id;        /**< An unique ID of that target. */
    float    level;            /**< The Level at the peak in dB relative to
                                    threshold. */
    float    radius;           /**< The Distance of the target from the
                                    sensor. */
    float    azimuth;          /**< The azimuth angle of the target. Positive
                                    values in right direction from the sensing
                                    board perspective. */
    float    elevation;        /**< The elevation angle of the target.
                                    Positive values in up direction from the
                                    sensing board perspective. */
    float    radial_speed;     /**< The change of radius per second. */
    float    azimuth_speed;    /**< The change of azimuth angle per second. */
    float    elevation_speed;  /**< The change of elevation angle per second.
                                    */
} Target_Info_t;


// ---------------------------------------------------------------------------------

typedef void(*Callback_Dsp_Settings_t)(void* context,
                                       int32_t protocol_handle,
                                       uint8_t endpoint,
                                       const DSP_Settings_t*);

typedef void(*Callback_Target_Processing_t)(void* context,
                                            int32_t protocol_handle,
                                            uint8_t endpoint,
                                            const Target_Info_t* targets,
                                            uint8_t num_targets);
                                               

/*
==============================================================================
   5. FUNCTION PROTOTYPES AND INLINE FUNCTIONS
==============================================================================
*/

/**
 * \brief This function checks if an endpoint in a device is an TargetDetect
 *        endpoint.
 *
 * This function checks type and version of the specified endpoint in a
 * connected device and returns a code that indicates if that endpoint is
 * compatible to the radar endpoint implementation in this module.
 * 
 * \param[in] protocol_handle  A handle to the connection to the sensor
 *                             device.
 * \param[in] endpoint         The endpoint in the connected device to be
 *                             checked for compatibility.
 *
 * \return If the specified endpoint is compatible to this implementation the
 *         function returns 0. If the endpoint is not compatible, a negative
 *         error code is returned.
 */
int32_t ep_targetdetect_is_compatible_endpoint(int32_t protocol_handle,
                                               uint8_t endpoint);

/**
 * \brief This functions registers a callback for radar frame data messages.
 *
 * If a callback for this message type is registered, that callback is issued
 * every time, a connected board sends a message containing radar frame data.
 * If no callback is registered the message is ignored.
 *
 * Connection and handle and endpoint number of the sending endpoint are
 * passed to the callback along with the message data. Furthermore the pointer
 * context set by this function is forwarded to the callback.
 *
 * For more information about the callback function see
 * \ref Callback_Data_Frame_t.
 *
 * \param[in] callback  The function to be called when a radar frame data
 *                      message is received.
 * \param[in] context   A data pointer that is forwarded to the callback
 *                      function.
 */
void ep_targetdetect_set_callback_dsp_settings(Callback_Dsp_Settings_t
                                                   callback,
                                               void* context);

void ep_targetdetect_set_callback_target_processing(Callback_Target_Processing_t
                                                        callback,
                                                    void* context);

int32_t ep_targetdetect_set_dsp_settings(int32_t protocol_handle,
                                         uint8_t endpoint,
                                         const DSP_Settings_t* pDSPSettings);

int32_t ep_targetdetect_get_dsp_settings(int32_t protocol_handle,
                                         uint8_t endpoint);

int32_t ep_targetdetect_get_targets(int32_t protocol_handle, uint8_t endpoint);

/* --- Close open blocks -------------------------------------------------- */

/* Disable C linkage for C++ files */
#ifdef __cplusplus
}  /* extern "C" */
#endif /* __cplusplus */

/* End of include guard */
#endif /* ENDPOINTTARGETDETECT_H_INCLUDED */

/* --- End of File -------------------------------------------------------- */
