#pragma once
#include "ros/ros.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Vector3.h"

class AutoAim{
public:
	AutoAim(ros::NodeHandle*);

private:
	ros::NodeHandle* nh;
	ros::Publisher gimbal_pub;
	ros::Subscriber cv_sub;

	void cvCallback(const geometry_msgs::Point&);
	void calculateError(void);
	void calculateAngularVel(void);
	void publishSpeed(void);

	void PID_Yaw(void);
	void PID_Pitch(void);

	float target_x;
	float target_y;
	float target_z;

	float err_yaw; 
	float err_pitch;

	float speed_yaw;
	float speed_pitch;
};
