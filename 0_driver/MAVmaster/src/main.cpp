/*
 * main.cpp
 *
 *  Created on: Mar 3, 2018
 *      Author: kyle
 */
#include "interface.h"
#include <unistd.h>
#include <cstdio>

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

mavconn::MAVConnInterface::Ptr fcu_link;
ros::Publisher att_pub;
ros::Subscriber cv_sub;

void myCB(const mavlink_message_t *message, const mavconn::Framing framing){
	static mavlink_heartbeat_t hb;
	static mavlink_attitude_t att;

	static geometry_msgs::Twist ros_att;
	switch(message->msgid){
		case MAVLINK_MSG_ID_HEARTBEAT:
			mavlink_msg_heartbeat_decode(message,&hb);
			printf("Decoded Heartbeat: %d,%d,%d,%d\n\n",hb.type,hb.base_mode,hb.custom_mode,hb.system_status);
			break;
		case MAVLINK_MSG_ID_ATTITUDE:
			mavlink_msg_attitude_decode(message,&att);
			printf("Decoded Attitude: %d, %f, %f, %f\n\n",att.time_boot_ms,att.roll,att.pitch,att.yaw);
			ros_att.angular.x=att.roll;
			ros_att.angular.y=att.pitch;
			ros_att.angular.z=att.yaw;
			att_pub.publish(ros_att);
			break;
		default:
			printf("Unknown message\n\n");
	}
}

void cvCallback(const geometry_msgs::Twist& cv_msg){
	static mavlink_message_t mav_msg;
	mavlink_msg_attitude_pack(21,78,&mav_msg,ros::Time::now().toSec(),
		cv_msg.linear.x,cv_msg.linear.y,cv_msg.linear.z,
		cv_msg.angular.x,cv_msg.angular.y,cv_msg.angular.z);
	mav_msg.magic = MAVLINK_STX_MAVLINK1;
	fcu_link->send_message(&mav_msg);
}

int main(int argc, char* argv[]){

	ros::init(argc,argv,"mavmaster");
	ros::NodeHandle n;
	att_pub = n.advertise<geometry_msgs::Twist>("Attitude",100);

	std::string serial_port;
	n.param<std::string>("serial_port", serial_port, "serial:///dev/ttyUSB0");

	int sysID, compID;
	n.param<int>("sysID", sysID, 21);
	n.param<int>("compID", compID, 78);

	fcu_link=mavconn::MAVConnInterface::open_url(serial_port,sysID,compID);
	fcu_link->message_received_cb = myCB;

	cv_sub = n.subscribe("cv_result",100,cvCallback);
    ros::spin();
}
