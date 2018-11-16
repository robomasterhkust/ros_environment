/**
 * @brief 
 * 
 * @file ROSInterface.hpp
 * @author Alex Au
 * @date 2018-06-14
 */

#pragma once
#include <cstring>
#include <string>
#include <thread>
#include <mutex>
#include <opencv2/opencv.hpp>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Point.h"
#include "std_msgs/Empty.h"
#include "ArmorDetection.hpp"
#include "ConcurrentQueue.hpp"

class ROSInterface
{
public:
  ROSInterface(int argc, char **argv);
  ~ROSInterface();

  bool tryProcess(ConcurrentQueue<ArmorStorage> &inputQ, ConcurrentQueue<ArmorStorage> &outputQueue);
  void publishCoors(ArmorStorage &input);

  ros::NodeHandle *rosNodeHandle;

private:
  ros::AsyncSpinner *spinner;
  ros::Publisher armor_publisher;
  ros::Publisher twistStampedPub;
  ros::Publisher vertice_publisher;
};
