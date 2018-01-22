//
// Created by Beck on 1/22/18.
//

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/PoseStamped.h>
#include <iostream>
#include <fstream>

ros::Publisher pub_comm;

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "comm_to_dev_board");
    ros::NodeHandle n("~");

    pub_comm = n.advertise<geometry_msgs::PoseStamped>("/gimbal/pose", 10);

    ros::spin();
}