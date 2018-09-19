/*
 ===================================================================================
 Name        : extract_target_data.c
 Author      : Pooja Agrawal
 Version     :
 Copyright   : 2014-2017, Infineon Technologies AG
 Description : Example of how to extract target data using the C communication library
 ===================================================================================
 */

/*
 * Copyright (c) 2014-2017, Infineon Technologies AG
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,are permitted provided that the
 * following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list of conditions and the following
 * disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following
 * disclaimer in the documentation and/or other materials provided with the distribution.
 *
 * Neither the name of the copyright holders nor the names of its contributors may be used to endorse or promote
 * products derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE  FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY,OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <stdio.h>
#include <string.h>
#include "Protocol.h"
#include "COMPort.h"
#include "EndpointRadarBase.h"
#include "EndpointTargetDetection.h"
#include "EndpointCalibration.h"
#include "ros/ros.h"
#include "sensor_msgs/PointCloud.h"
#include <thread>
#include <chrono>

ros::Publisher pointCloudPub;

// called every time ep_targetdetect_get_targets method is called to return measured time domain signals
void received_target_info(void *context,
                          int32_t protocol_handle,
                          uint8_t endpoint,
                          const Target_Info_t *targets, uint8_t num_targets)
{
    sensor_msgs::PointCloud ptCloud;
    ptCloud.channels.push_back(sensor_msgs::ChannelFloat32());
    ptCloud.channels[0].name = "Radial speed";
    uint32_t j;
    for (j = 0; j < num_targets; j++, targets++)
    {
        geometry_msgs::Point32 point;
        point.y = point.z = 0;
        printf("*********************Received targets*********************\n");
        printf("Received target: target_id %d\n", targets->target_id);
        printf("Received target: Distance %f [cm]\n", point.x = targets->radius);
        printf("Received target: Radial speed %f\n", targets->radial_speed);
        printf("Received target: Azimuth %f\n", targets->azimuth);
        printf("Received target: Azimuth speed %f\n", targets->azimuth_speed);
        printf("Received target: Elevation %f \n", targets->elevation);
        printf("Received target: Elevation speed %f \n", targets->elevation_speed);
        printf("\n");
        ptCloud.points.push_back(point);
        ptCloud.channels[0].values.push_back(targets->radial_speed);
    }
    pointCloudPub.publish(ptCloud);
}

// called every time ep_radar_base_get_frame_data method is called to return measured time domain signals
void received_frame_data(void *context,
                         int32_t protocol_handle,
                         uint8_t endpoint,
                         const Frame_Info_t *frame_info)
{
    // Print the sampled data which can be found in frame_info->sample_data
    for (uint32_t i = 0; i + 7 < frame_info->num_samples_per_chirp; i += 8)
    {
        printf("%d : \t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\n", i, frame_info->sample_data[i],
               frame_info->sample_data[i + 1],
               frame_info->sample_data[i + 2],
               frame_info->sample_data[i + 3],
               frame_info->sample_data[i + 4],
               frame_info->sample_data[i + 5],
               frame_info->sample_data[i + 6],
               frame_info->sample_data[i + 7]);
    }
}

int radar_auto_connect(void)
{
    int radar_handle = 0;
    int num_of_ports = 0;
    char comp_port_list[256];
    char *comport;
    const char *delim = ";";

    //----------------------------------------------------------------------------

    num_of_ports = com_get_port_list(comp_port_list, (size_t)256);

    if (num_of_ports == 0)
    {
        return -1;
    }
    else
    {
        comport = strtok(comp_port_list, delim);

        while (num_of_ports > 0)
        {
            num_of_ports--;

            // open COM port
            radar_handle = protocol_connect(comport);

            printf("%s result in %d", comport, radar_handle);

            if (radar_handle >= 0)
            {
                break;
            }
            comport = strtok(NULL, delim);
        }

        return radar_handle;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "inf24g");
    ros::NodeHandle nh = ros::NodeHandle("~");
    pointCloudPub = nh.advertise<sensor_msgs::PointCloud>("inf24radar", 3);

    int res = -1;
    int protocolHandle = 0;
    int endpointRadarBase = 0;
    int endpointTargetDetection = 0;

    // open COM port
    protocolHandle = radar_auto_connect();

    // get endpoint ids
    if (protocolHandle >= 0)
    {
        uint32_t i;
        for (i = 1; i <= protocol_get_num_endpoints(protocolHandle); ++i)
        {
            // current endpoint is radar base endpoint
            if (ep_radar_base_is_compatible_endpoint(protocolHandle, i) == 0)
            {
                endpointRadarBase = i;
                continue;
            }
            if (ep_targetdetect_is_compatible_endpoint(protocolHandle, i) == 0)
            {
                endpointTargetDetection = i;
                continue;
            }
        }
    }
    printf("protocolHandle: %d\n", protocolHandle);
    printf("endpointRadarBase: %d, endpointTargetDetection: %d\n", endpointRadarBase, endpointTargetDetection);

    if (endpointRadarBase > 0 || endpointTargetDetection > 0)
    {
        // register call back functions for target data
        ep_targetdetect_set_callback_target_processing(received_target_info, NULL);
        // ep_radar_base_set_callback_data_frame(received_frame_data, NULL);
        ep_calibration_get_calibration_data(protocolHandle, endpointRadarBase);
        ep_calibration_set_calibration_data(protocolHandle, endpointRadarBase);

        res = ep_radar_base_set_automatic_frame_trigger(protocolHandle,
                                                        endpointRadarBase,
                                                        0);
        while (ros::ok())
        {
            // get target data
            ep_targetdetect_get_targets(protocolHandle, endpointTargetDetection);
            // get raw data
            // res = ep_radar_base_get_frame_data(protocolHandle, endpointRadarBase, 1);

            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }
    else
    {
        printf("no device found\n");
        return 0;
    }
    return res;
}
