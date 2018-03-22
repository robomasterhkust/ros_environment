//g++ --std=c++11 -pthread main.cpp Camera/Camera.cpp Camera/V4LCamDriver.cpp Camera/mvCamera.cpp Settings/Settings.cpp ArmorDetection/ArmorDetection.cpp Utilities/StopWatch.cpp multiThread/ThreadPool.cpp `pkg-config --libs --cflags opencv` -lMVSDK
// use ./a.out [number of worker thread to run]
#define DEBUG
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include "ConcurrentQueue.hpp"
#include "Settings.hpp"
#include "Camera.hpp"
#include "V4LCamDriver.hpp"
#include "ArmorDetection.hpp"
#include "StopWatch.hpp"
#include <iostream>
#include <time.h>
#include <opencv2/opencv.hpp>
#include <chrono>
#include "ThreadPool.hpp"
#include "geometry_msgs/Twist.h"

using namespace std;
using namespace cv;

//this is global and used by many other files
Settings settings("settings.xml");
static geometry_msgs::Twist msg_Twist_Gimbal;

//Handle keyboard control override
void key_Callback(const geometry_msgs::Twist &key_msg)
{
    if (key_msg.linear.x > 0.0)
        msg_Twist_Gimbal.angular.y = 0.5;
    else if (key_msg.linear.x < 0.0)
        msg_Twist_Gimbal.angular.y = -0.5;
    else
        msg_Twist_Gimbal.angular.y = 0.0;

    msg_Twist_Gimbal.angular.z = key_msg.angular.z / 2.0;
};

int main(int argc, char **argv)
{
    //ROS Init
    ros::init(argc, argv, "CV_node");
    ros::NodeHandle rosNodeHandle;
    ros::Publisher chatter_pub = rosNodeHandle.advertise<std_msgs::String>("cv_result", 1000);
    ros::Subscriber key_sub = rosNodeHandle.subscribe("/key_vel", 100, key_Callback);
    memset(&msg_Twist_Gimbal, 0, sizeof(geometry_msgs::Twist));
    ros::Rate loop_rate(50);
    //ROS Init End

    settings.load();
    const int no_of_threads = (argc > 1) ? (argv[1][0] - '0') : (1);
    ThreadPool mainThreadPool;
    mainThreadPool.initialize();
    mainThreadPool.startThreads(no_of_threads);

    while (1)
    {
        if (mainThreadPool.run)
        {
            mainThreadPool.doBossWork();
        }

        int keyin = waitKey(1);
        switch (keyin)
        {
        case 32:
            if (mainThreadPool.run)
                mainThreadPool.stopThreads();
            else
                mainThreadPool.startThreads(no_of_threads);
            break;
        case 27:
            mainThreadPool.stopThreads();
            return 0;
        }
    }
}