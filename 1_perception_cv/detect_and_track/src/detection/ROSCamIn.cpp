#include "ROSCamIn.hpp"
#include <cv_bridge/cv_bridge.h>
#include "Settings.hpp"
#include "Camera.hpp"
#include "main.hpp"
#include <mutex>
#include "ros/ros.h"

ROSCamIn::ROSCamIn(const string &config_filename)
    : Camera(config_filename),
      imgSub_Ptr(NULL),
      inputq(5){};

ROSCamIn::~ROSCamIn()
{
    delete imgSub_Ptr;
}

void ROSCamIn::pushImg(const boost::shared_ptr<sensor_msgs::Image> msg)
{
    // static std::mutex lock;
    // std::lock_guard<std::mutex> gl(lock);
    boost::shared_ptr<sensor_msgs::Image> *temp = new boost::shared_ptr<sensor_msgs::Image>(msg);
    inputq.enqueue(temp);
    ROS_INFO("%s incoming image with latency: %f ms", this->config_filename.c_str(), (ros::Time::now() - msg->header.stamp).toSec() * 1000.0);
}

FrameInfo *ROSCamIn::getFrame()
{
    //const sensor_msgs::Image *tempin;
    const boost::shared_ptr<sensor_msgs::Image> *tempin;
    if (inputq.dequeue(tempin))
    {
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(**tempin, (*tempin)->encoding);
        }
        catch (cv_bridge::Exception &e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return NULL;
        }

        FrameInfo *tempout = new FrameInfo(this);
        tempout->rosheader = (*tempin)->header;
        tempout->img = cv_ptr->image;

        delete tempin;
        return tempout;
    }
    return NULL;
};

bool ROSCamIn::initialize() { return detectionNodeShared::rosIntertface; };
void ROSCamIn::discardFrame(){};
bool ROSCamIn::startStream()
{
    if (!imgSub_Ptr)
    {
        imgSub_Ptr = new ros::Subscriber(detectionNodeShared::rosIntertface->rosNodeHandle->subscribe(topicName, 1, &ROSCamIn::pushImg, this));
        ROS_INFO("Subscribed to %s", topicName.c_str());
    }
    return true;
};
bool ROSCamIn::closeStream()
{
    imgSub_Ptr->shutdown();
    delete imgSub_Ptr;
    imgSub_Ptr = NULL;
    return true;
};
void ROSCamIn::info(){};

bool ROSCamIn::loadDriverParameters(const FileStorage &fs)
{
    const FileNode &node = fs["ROSCamIn"];
    fsHelper::readOrDefault(node["topicName"], topicName);
    ;
};

bool ROSCamIn::storeDriverParameters(FileStorage &fs)
{
    fs << "ROSCamIn"
       << "{"
       << "topicName" << topicName
       << "}";
}

bool ROSCamIn::setCamConfig()
{
    return true;
};

bool ROSCamIn::getCamConfig()
{
    return true;
};