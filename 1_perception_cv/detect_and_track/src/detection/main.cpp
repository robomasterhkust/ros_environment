/**
  * @brief 
  * it is quite tricky to run this with th mv camera
  * the command i used is rosrun --prefix "sudo -E bash -c" rm_cv rm_cv_node
  * @file main.cpp
  * @author Alex Au
  * @date 2018-04-02
 */
#include <sstream>
#include <iostream>
#include <time.h>
#include <opencv2/opencv.hpp>
#include <chrono>
#include <cstring>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "defines.hpp"
#include "ConcurrentQueue.hpp"
#include "Settings.hpp"
#include "Camera.hpp"
#include "ArmorDetection.hpp"
#include "StopWatch.hpp"
#include "cvThreadPool.hpp"
#include "ROSInterface.hpp"

using namespace std;
using namespace cv;

namespace detectionNodeShared
{
ROSInterface *rosIntertface;
Settings settings("settings.xml");

} // namespace detectionNodeShared

int main(int argc, char **argv)
{
    detectionNodeShared::rosIntertface = new ROSInterface(argc, argv);
    detectionNodeShared::settings.load();
    ThreadPool mainThreadPool;
    if (!mainThreadPool.initialize())
    {
        return 1;
    }

    mainThreadPool.startThreads(detectionNodeShared::settings.threadCount);

    ros::Rate loop_rate(200);
    
    while (ros::ok())
    {
        mainThreadPool.doBossWork();
        loop_rate.sleep();
    }

    mainThreadPool.stopThreads();
    return 0;
}
