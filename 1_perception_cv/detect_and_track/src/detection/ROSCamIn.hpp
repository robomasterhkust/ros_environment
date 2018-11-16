#pragma once
#include "Camera.hpp"
#include "opencv2/core/core.hpp"
#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "ConcurrentQueue.hpp"
#include <string>

class ROSCamIn : public Camera
{
public:
  ROSCamIn(const string &config_filename);
  ~ROSCamIn();

  bool loadDriverParameters(const FileStorage &fs);
  bool storeDriverParameters(FileStorage &fs);

  bool setCamConfig();
  bool getCamConfig();

  bool initialize();
  void discardFrame();

  FrameInfo *getFrame();

  void info();

  bool startStream();
  bool closeStream();

private:
  void pushImg(const boost::shared_ptr<sensor_msgs::Image> msg);
  //void pushImg(const sensor_msgs::Image &newimg);

  ConcurrentQueue<const boost::shared_ptr<sensor_msgs::Image>> inputq;
  //ros::Time lastReadTime;
  ros::Subscriber *imgSub_Ptr;
  std::string topicName;
};