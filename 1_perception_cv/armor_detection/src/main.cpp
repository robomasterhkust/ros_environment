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
#include "V4LCamDriver.hpp"
#include "ArmorDetection.hpp"
#include "StopWatch.hpp"
#include "cvThreadPool.hpp"
#include "geometry_msgs/Twist.h"

using namespace std;
using namespace cv;

//this is global and used by many other files
Settings settings("settings.xml");

int main(int argc, char **argv)
{
    settings.load();
    ThreadPool mainThreadPool(argc, argv);
    if (!mainThreadPool.initialize())
    {
        return 1;
    }

    mainThreadPool.startThreads(THREAD_COUNT);

    ros::Rate loop_rate(60);

    while (1)
    {
        if (!ros::ok())
            break;

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
                mainThreadPool.startThreads(THREAD_COUNT);
            break;
        case 27:
            mainThreadPool.stopThreads();
            return 0;
        }

        loop_rate.sleep();
    }
}
