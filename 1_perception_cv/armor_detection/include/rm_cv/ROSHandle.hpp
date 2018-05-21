/**
 * @brief 
 * 
 * @file ROSHandle.hpp
 * @author Alex Au
 * @date 2018-03-27
 */
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

class ROSHandle
{
public:
  ROSHandle(int argc, char **argv);
  ~ROSHandle();
  //void setGimbalSpeed(const double &x, const double &y, const double &z);
  //void setTargetCoor(const double &x, const double &y, const double &z);
  void publishCoor(cv::Vec3d targetCoor);

private:
  void termProgram(const std_msgs::Empty &input);

  ros::NodeHandle *rosNodeHandle;
  ros::Publisher *cv_result_publisher;
  ros::Subscriber *terminate_sub;
  //ros::Rate loop_rate;

  //geometry_msgs::Point msg_Point_Target;
};
