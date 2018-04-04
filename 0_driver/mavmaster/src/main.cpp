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
			printf("Decoded Attitude: %d, %f, %f, %f\n\n",att.time_boot_ms,att.rollspeed,att.pitchspeed,att.yawspeed);
			ros_att.angular.x=att.roll;
			ros_att.angular.y=att.pitch;
			ros_att.angular.z=att.yaw;
			att_pub.publish(ros_att);
			break;
		default:
			printf("Unknown message\n\n");
	}
}

void autoAimCallback(const geometry_msgs::Vector3& am_msg){
	static mavlink_message_t mav_msg;
	mavlink_msg_attitude_pack_chan(21,78,MAVLINK_COMM_0,&mav_msg,ros::Time::now().toSec(),
		0,0,0,0,am_msg.y,am_msg.z);
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
	fcu_link->set_protocol_version(mavconn::Protocol::V10);
	ROS_INFO("MAVLink Protocol Version [mavconn]: %u",
		static_cast<unsigned int>(fcu_link->get_protocol_version()));
	mavlink_set_proto_version(MAVLINK_COMM_0,1);
	ROS_INFO("MAVLink Protocol Version [main]: %u",
		mavlink_get_proto_version(MAVLINK_COMM_0));

	fcu_link->message_received_cb = myCB;
	cv_sub = n.subscribe("gimbal_ang_vel",100,autoAimCallback);
    ros::spin();
}
