#ifndef CAMERA_H
#define CAMERA_H

#include <algorithm>
#include <iostream>
#include <vector>

#include <cv_bridge/cv_bridge.h>
#include <dynamic_reconfigure/server.h>
#include <errno.h>
//#include <mvIMPACT_CPP/mvIMPACT_acquire.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Float64.h>
#include <unordered_map>

#include "multiCameraReader.h"

#include <djiros/HardwareSync.h>

namespace ptgrey
{

struct CameraSetting
{
    unsigned int serial; // special, diffrent with bluefox!!!
    std::string topic;

    bool is_print_info;
    bool is_sync;
    bool is_auto_shutter;

    double shutter_ms;
    double brightness;
    double exposure;
    double gain;
    double frameRate;
    double cam_cnt;
};

class Camera
{
    public:
    static constexpr int MAX_CAM_CNT = 10;

    Camera( ros::NodeHandle param_nh );

    ~Camera( ) { delete camReader; }

    void feedImages( );
    bool wait_for_imu_ack( SyncAckInfo& sync_ack, int& queue_size );

    void process_slow_sync( );

    bool isOK( );
    bool is_slave_mode( ) const;
    bool is_fast_mode( ) const;

    private:
    // Node handle
    ros::NodeHandle pnode;

    ptgrey_reader::multiCameraReader* camReader;

    std::vector< ros::Publisher > image_pubs;
    std::unordered_map< std::string, ros::Publisher > img_publisher;
    std::unordered_map< std::string, sensor_msgs::Image > img_buffer;
    std::vector< ptgrey_reader::cvImage > images_tmp;

    // Internal parameters that cannot be changed
    bool ok;
    ros::Time capture_time;
    bool m_is_color;
    bool m_is_slave_mode;
    bool m_verbose_output;
    bool m_fast_mode;

    // User specified parameters
    int cam_cnt;
    double m_fps;

    // Hardware sync related
    public:
    std::shared_ptr< HardwareSynchronizer > m_hwsync;
    int m_hwsync_grab_count;
    int m_max_req_number;
};
}
#endif // CAMERA_H
