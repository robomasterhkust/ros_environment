/**
  * @brief 
  * 
  * @file GimbalController.hpp
  * @author Alex Au
  * @date 2018-04-05
 */

#pragma once
#include "defines.hpp"
#include "ros/ros.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Vector3.h"
#include "std_msgs/Empty.h"
#include "interface.h"
#include <chrono>
#include <time.h>

/**
  * @brief
  * the gimbal controller that accepts an 3d coordinate as the target to shoot
  * or a direction and speed to search (2d vector of x = right, y = down) 
  * otherwise this will rotate around 
 */
class GimbalController
{
public:
  GimbalController(ros::NodeHandle *nh_ptr, bool enable_Serial);

  void updateParasCallback(const std_msgs::Empty &input);

  /**
    * @brief 
    * input point follow opencv coordinate system here,
    * right handed system
    * z forward, x right, y down
   */
  bool updateTarget(float x, float y, float z);
  bool updateParas();

  //void sendOutToSerial();
  void publishToCan();

  static void serialCallBack(const mavlink_message_t *message, const mavconn::Framing framing);

private:
  const ros::NodeHandle *nh_ptr;
  double yaw_kp, yaw_ki, yaw_kd,
      pitch_kp, pitch_ki, pitch_kd;
  double maxPitch, maxYaw;
  double maxPitch_i, maxYaw_i;
  double expireTime;

  timespec lastupdate_time;
  double pitch_i, yaw_i, last_pitch_e, last_yaw_e;

  float pitch_v_out, yaw_v_out;

  ros::Subscriber tuner_sub;
  ros::Publisher canPub;

  bool enable_Serial;
  mavconn::MAVConnInterface::Ptr fcu_link = NULL;
};
