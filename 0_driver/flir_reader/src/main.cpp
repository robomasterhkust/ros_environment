/**
 * @brief 
 * 
 * @file main.cpp
 * @author Alex Au
 * @date 2018-09-04
 */
#include <chrono>
#include "ros/ros.h"
#include "CamBase.hpp"
#include "FlirCam.hpp"

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
    PointGreyCamera cam("test.xml");
    cam.loadAllConfig();
    cam.initialize();
    cam.info();
    cam.applySetting();
    cam.startStream();
    FrameInfo *f = cam.getFrame();
    cv::imshow("img", f->img);
    cv::waitKey(0);
    return 0;
}
