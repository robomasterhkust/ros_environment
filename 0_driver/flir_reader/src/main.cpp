/**
 * @brief 
 * 
 * @file main.cpp
 * @author Alex Au
 * @date 2018-09-04
 */
#include <chrono>
#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include "CamBase.hpp"
#include "FlirCam.hpp"

using namespace std;
using namespace cv;

vector<pair<ros::Publisher *, PointGreyCamera *>> image_publishers;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "flir_reader");
    ros::NodeHandle nh("~");

    int num_cams;
    nh.getParam("num_cams", num_cams);
    printf("num_cams: %d", num_cams);

    for (int i = 0; i < num_cams; i++)
    {
        string fn = nh.param<string>("cam" + to_string(i), "");
        if (fn != "")
        {
            image_publishers.push_back(pair<ros::Publisher *, PointGreyCamera *>());
            image_publishers.back().first = new ros::Publisher(nh.advertise<sensor_msgs::Image>("/cam" + to_string(i), 3));
            image_publishers.back().second = new PointGreyCamera(fn);
            if (image_publishers.back().second->loadAllConfig())
            {
                image_publishers.back().second->initialize();
                image_publishers.back().second->info();
                image_publishers.back().second->applySetting();
                printf("apply setting\n");
                image_publishers.back().second->startStream();
                printf("startStream\n");
            }
            else
            {
                delete image_publishers.back().first;
                delete image_publishers.back().second;
                image_publishers.pop_back();
            }
        }
    };
    if (image_publishers.size() == 0)
    {
        return 0;
    }

    //TODO: multithread
    while (1)
    {
        for (auto p : image_publishers)
        {
            cv_bridge::CvImage *f = p.second->getFrameROS();
            cv::imshow("img", f->image);
            sensor_msgs::Image imgMsg;
            f->toImageMsg(imgMsg);
            p.first->publish(imgMsg);
            delete f;
        };

        if (cv::waitKey(1) == 27)
            break;
    }

    for (auto p : image_publishers)
    {
        delete p.first;
        delete p.second;
    };

    return 0;
}
