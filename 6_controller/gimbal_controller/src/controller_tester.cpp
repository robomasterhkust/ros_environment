/**
 * Tester program to check runtime error
 */
#include <iostream>
#include <ros/ros.h>
#include "rm_cv/vertice.h"
#include <geometry_msgs/TwistStamped.h>

ros::Publisher visual_pub, omega_pub;
std::string publisher_topic, omega_topic;

void
pub_visual_msg()
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
    visual_pub.publish(test_msg);
}

void
pub_omega_msg()
{
    geometry_msgs::TwistStamped omega_msg;
    omega_msg.header.stamp = ros::Time::now();
    omega_msg.twist.angular.x = 0;
    omega_msg.twist.angular.y = 0;
    omega_msg.twist.angular.z = 0;
    omega_pub.publish(omega_msg);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "controller_tester");
    ros::NodeHandle nh("~");

    nh.param("publisher_topic", publisher_topic, std::string("/detected_vertice"));
    nh.param("omega_topic", omega_topic, std::string("/can_transimit/omega_cam"));

    visual_pub = nh.advertise<rm_cv::vertice>(publisher_topic, 10);
    omega_pub  = nh.advertise<geometry_msgs::TwistStamped>(omega_topic, 10);

    ros::Rate r(30);

    ros::Duration(1).sleep();

    while (ros::ok()) {
        pub_visual_msg();
        pub_omega_msg();
        r.sleep();
        ros::spinOnce();
    }
}
