#include <can_msgs/Frame.h>
#include <nav_msgs/Odometry.h>
#include <string>
#include <ros/ros.h>

ros::Publisher uwb_publisher;
bool uwb_start = false;
int uwb_index = 0;

void msgCallback(const can_msgs::Frame &f){
    if(f.dlc==6&&!uwb_start){
        uwb_start = true;
    }
    if(uwb_start){
        if(f.dlc==8){
            uwb_index++;
        }
	if(f.dlc==6){
	    uwb_index = 0;	
	}
	if(uwb_index==1){
	    nav_msgs::Odometry uwb_info;
            uwb_info.header.stamp = f.header.stamp;
            uwb_info.header.frame_id = "world";
            uwb_info.pose.pose.position.x = ((int16_t)((uint16_t)f.data[1]<<8 | (uint16_t)f.data[0]))/100.0; // in meter
            uwb_info.pose.pose.position.y = ((int16_t)((uint16_t)f.data[3]<<8 | (uint16_t)f.data[2]))/100.0; // in meter
            uwb_info.pose.pose.orientation.w = (uint16_t)((uint16_t)f.data[5]<<8 | (uint16_t)f.data[4]);
            uwb_publisher.publish(uwb_info);
	}
    }
}

int main(int argc, char *argv[]){
    ros::init(argc,argv,"uwb_driver_node");
    ros::NodeHandle nh("~");
    uwb_publisher = nh.advertise<nav_msgs::Odometry>("/uwb_odom", 100);
    ros::Subscriber can_subscriber = nh.subscribe("/received_messages", 100, msgCallback);

    ros::spin();
}
