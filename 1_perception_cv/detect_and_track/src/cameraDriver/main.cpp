/**
 * @brief 
 * 
 * @file main.cpp
 * @author Alex Au
 * @date 2018-09-04
 */
#include <chrono>
#include <ros/ros.h>
#include <thread>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include "CamBase.hpp"
#include "defines.hpp"
#include <signal.h>

using namespace std;
using namespace cv;

vector<pair<ros::Publisher *, CamBase *>> image_publishers;
vector<thread *> readers;

volatile bool thdShouldTerminate = false;

/**
 * @brief a thread function to keep reading from a camera object and publish result to ROS
 */
void reader(ros::Publisher *pub, CamBase *cam)
{
    if (pub && cam)
        while (!thdShouldTerminate)
        {
            //TODO: software limit of framerate
            FrameInfo *f = cam->getFrame();
            if (f)
            {
                cv_bridge::CvImage rosimg;
                std_msgs::Header h;
                h.stamp = f->rosheader.stamp;
                rosimg.image = f->img;
                rosimg.encoding = sensor_msgs::image_encodings::BGR8;
                rosimg.header = f->rosheader;
                sensor_msgs::Image imgMsg;
                rosimg.toImageMsg(imgMsg);
                pub->publish(imgMsg);
                printf("captured from %s, latency: %f\n", cam->getName().c_str(), (ros::Time::now() - imgMsg.header.stamp).toSec());
                delete f;
            }
        }
};

int main(int argc, char **argv)
{
    ROS_INFO("Usage: In the launch file specify the amount of cameras in num_cams\n"
             "Then in the ROS_HOME directory (the working directory of this execuble), under the folder " CAM_CONFIG_FOLDER_STR
             "place cam0.xml cam1.xml ... as the configuration for the cameras");

    ros::init(argc, argv, "cam_reader");
    ros::NodeHandle nh("~");

    int num_cams;
    nh.getParam("num_cams", num_cams);
    printf("num_cams: %d\n", num_cams);

    for (int i = 0; i < num_cams; i++)
    {
        printf("reading cam %d\n", i);
        string fn = nh.param<string>("cam" + to_string(i), "");
        if (fn != "")
        {
            CamBase *tempCam = startCamFromFile(fn);
            if (tempCam)
            {
                image_publishers.push_back(pair<ros::Publisher *, CamBase *>());
                image_publishers.back().first = new ros::Publisher(nh.advertise<sensor_msgs::Image>("/cam" + to_string(i), 3));
                image_publishers.back().second = tempCam;
                readers.push_back(new thread(&reader,
                                             image_publishers.back().first,
                                             image_publishers.back().second));

                ros::param::set("/cam" + to_string(i) + "/cameraMatrix/fx", tempCam->getCameraMatrix().at<float>(1, 1));
                ros::param::set("/cam" + to_string(i) + "/cameraMatrix/cx", tempCam->getCameraMatrix().at<float>(1, 3));
                ros::param::set("/cam" + to_string(i) + "/cameraMatrix/fy", tempCam->getCameraMatrix().at<float>(2, 2));
                ros::param::set("/cam" + to_string(i) + "/cameraMatrix/cy", tempCam->getCameraMatrix().at<float>(2, 3));
                for (int i = 0; i < tempCam->getDistCoeffs().rows; i++)
                    ros::param::set("/cam" + to_string(i) + "/cameraMatrix/" + to_string(i), tempCam->getDistCoeffs().at<float>(i, 1));
            }
        }
    };
    if (image_publishers.size() == 0)
    {
        printf("no device available to publish image from\n");
        return 0;
    }

    namedWindow("cam_reader");

    while (cv::getWindowProperty("cam_reader", 1) != -1)
    {
        if (cv::waitKey(100) == 27)
        {
            break;
        }
    }
    thdShouldTerminate = true;

    for (int i = 0; i < image_publishers.size(); i++)
    {
        readers[i]->join(); //join before delete!
        delete image_publishers[i].first;
        delete image_publishers[i].second;
    };

    return 0;
}
