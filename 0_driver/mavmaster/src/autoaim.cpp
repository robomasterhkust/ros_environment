#include "autoaim.h"

AutoAim::AutoAim(ros::NodeHandle*in_nh):nh(in_nh){
	gimbal_pub = 
		nh->advertise<geometry_msgs::Vector3>("gimbal_ang_vel",100);
	cv_sub = 
		nh->subscribe("cv_result",100,&AutoAim::cvCallback,this);
}

void AutoAim::cvCallback(const geometry_msgs::Point& p){
	target_x = p.x;
	target_y = p.y;
	target_z = p.z;

	ROS_INFO("CV result = %.3f, %.3f, %.3f",target_x,target_y,target_z);

	calculateError();
	calculateAngularVel();

	ROS_INFO("Speed = Yaw %.3f, Pitch %.3f",speed_yaw,speed_pitch);
	publishSpeed();
}

inline void AutoAim::calculateError(){
	err_yaw = -target_x;
	err_pitch = target_y;
}

inline void AutoAim::calculateAngularVel(){
	PID_Yaw();
	PID_Pitch();
}

void AutoAim::PID_Yaw(){
	static float integral = 0;
	static float prev_err = 0;

	static float yaw_kp, yaw_ki, yaw_kd;
	nh->param<float>("yaw_kp",yaw_kp,0.0002);
	nh->param<float>("yaw_ki",yaw_ki,yaw_kp/10);
	nh->param<float>("yaw_kd",yaw_kd,yaw_kp/10);

	integral += err_yaw;
	speed_yaw = yaw_kp*err_yaw + yaw_ki*integral + yaw_kd*(err_yaw - prev_err);
	prev_err = err_yaw;
}

void AutoAim::PID_Pitch(){
    static float integral = 0;
    static float prev_err = 0;

    static float pitch_kp, pitch_ki, pitch_kd;
    nh->param<float>("pitch_kp",pitch_kp,0.0002);
    nh->param<float>("pitch_ki",pitch_ki,pitch_ki/10);
    nh->param<float>("pitch_kd",pitch_kd,pitch_ki/10);

    integral += err_pitch;
    speed_pitch = pitch_kp*err_pitch + pitch_ki*integral + pitch_kd*(err_pitch - prev_err);
    prev_err = err_pitch;
}

inline void AutoAim::publishSpeed(){
	static geometry_msgs::Vector3 ang_vel;

	ang_vel.z = speed_yaw;
	ang_vel.y = speed_pitch;

	gimbal_pub.publish(ang_vel);
}
