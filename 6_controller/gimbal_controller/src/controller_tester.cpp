//
// Created by beck on 27/9/18.
//
#include <iostream>
#include <ros/ros.h>
#include "rm_cv/vertice.h"

ros::Publisher pub;
std::string publisher_topic;

void
pub_msg()
{
    rm_cv::vertice test_msg;
    test_msg.vertex[0].x = 288;
    test_msg.vertex[1].x = 200;
    test_msg.vertex[2].x = 0;
    test_msg.vertex[3].x = 576;
    test_msg.vertex[0].y = 230;
    test_msg.vertex[1].y = 200;
    test_msg.vertex[2].y = 0;
    test_msg.vertex[3].y = 480;
    pub.publish(test_msg);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "controller_tester");
    ros::NodeHandle nh("~");

    nh.param("publisher_topic", publisher_topic, std::string("/detected_vertice"));

    pub = nh.advertise<rm_cv::vertice>(publisher_topic, 10);

    ros::Rate r(10);

    while (ros::ok()) {
        pub_msg();
        r.sleep();
        ros::spinOnce();
    }
}
