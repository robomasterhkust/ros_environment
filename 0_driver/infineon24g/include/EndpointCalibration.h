/*
  ==============================================================================

    EndPointCalibration.h
    Created: 27 Feb 2018 11:07:11am
    Author:  Assad Ali

  ==============================================================================
*/

#ifndef ENDPOINT_CALIBRATION_H_INCLUDED
#define ENDPOINT_CALIBRATION_H_INCLUDED

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

  typedef void (*Callback_Calibration_Data_t)(void *context,
                                              int32_t protocol_handle,
                                              uint8_t endpoint,
                                              const float *calibration_data_ptr,
                                              uint16_t num_of_samples);

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
  int32_t ep_calibration_is_compatible_endpoint(int32_t protocol_handle,
                                                uint8_t endpoint);

  /**
* \brief This functions registers a callback for radar calibration data messages.
*
* If a callback for this message type is registered, that callback is issued
* every time, a connected board sends a message containing radar calibration data.
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
  void ep_calibration_set_callback_calibration_data(Callback_Calibration_Data_t
                                                        callback,
                                                    void *context);

  int32_t ep_calibration_set_calibration_data(int32_t protocol_handle,
                                              uint8_t endpoint);

  int32_t ep_calibration_get_calibration_data(int32_t protocol_handle,
                                              uint8_t endpoint);

  int32_t ep_calibration_clear_calibration_data(int32_t protocol_handle,
                                                uint8_t endpoint);

  int32_t ep_calibration_set_sram_calibration_data(int32_t protocol_handle,
                                                   uint8_t endpoint);

  int32_t ep_calibration_get_sram_calibration_data(int32_t protocol_handle,
                                                   uint8_t endpoint);

  int32_t ep_calibration_clear_sram_calibration_data(int32_t protocol_handle,
                                                     uint8_t endpoint);

/* --- Close open blocks -------------------------------------------------- */

/* Disable C linkage for C++ files */
#ifdef __cplusplus
} /* extern "C" */
#endif /* __cplusplus */

/* End of include guard */
#endif /* ENDPOINT_CALIBRATION_H_INCLUDED */

/* --- End of File -------------------------------------------------------- */
