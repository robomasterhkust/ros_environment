/**
 * @brief 
 * 
 * @file main.cpp
 * @author Alex Au
 * @date 2018-09-04
 */
 */
#include <chrono>
#include "ros/ros.h"

using namespace std;
using namespace cv;


/**
 * @brief 
 * TODO: automatically create entries in the xml according to the camera's capability according to the serial number specified
 */
int main(int argc, char **argv)
{
    ros::init(argc, argv, "flir_reader");
    ros::NodeHandle nh("~");

    bool enable_publish = true;
    bool enable_publish= false;
    bool is_grey = false;
    bool is_auto_shutter = false;
    bool is_sync = true;
    bool is_roi = false;
    double brightness = 0.1;
    double exposure = 0.1;
    double gain = 1.0;
    double frameRate = 20.0;
    double shutter = 5.0;
    int WB_red = 500;
    int WB_Blue = 800;
    double saturation = 100;
    double hue = 0;
    double sharpness = 0;
    double sw_down_sample_scale = 1;
    int size_x = 0, size_y = 0;
    int center_x = 0, center_y = 0;
    int cropper_x = 0, cropper_y = 0;
    int src_rows, src_cols;
    cv::Mat image_grey;

    nh.getParam("is_pub", is_pub);
    nh.getParam("enablepreview", enablepreview);
    nh.getParam("is_print", is_print);
    nh.getParam("serialNum", serialNum);
    nh.getParam("is_auto_shutter", is_auto_shutter);
    nh.getParam("is_sync", is_sync);
    nh.getParam("is_roi", is_roi);
    nh.getParam("brightness", brightness);
    nh.getParam("exposure", exposure);
    nh.getParam("gain", gain);
    nh.getParam("frameRate", frameRate);
    nh.getParam("shutter", shutter);
    nh.getParam("WB_red", WB_red);
    nh.getParam("WB_Blue", WB_Blue);
    nh.getParam("saturation", saturation);
    nh.getParam("hue", hue);
    nh.getParam("sharpness", sharpness);

    nh.getParam("sw_down_sample_scale", sw_down_sample_scale);
    nh.getParam("size_x", size_x);
    nh.getParam("size_y", size_y);
    nh.getParam("center_x", center_x);
    nh.getParam("center_y", center_y);
    nh.getParam("cropper_x", cropper_x);
    nh.getParam("cropper_y", cropper_y);

    return 0;
}
