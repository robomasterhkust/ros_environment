/**
 * Beck Pang, 20180509, hardcoded global planner to feed into the local path planner
 */

#include <iostream>
#include <ros/ros.h>
#include <ros/console.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <vector>

using namespace std;
ros::Publisher path_pub;
string odom_topic, publisher_topic;
vector<double> path_array;
double path_array_init[10] = {5.0, 7.0, 5.0, 7.0, 4.5, 7.0, 4.0, 7.0, 3.5, 7.0};

void odom_callback(const nav_msgs::Odometry::ConstPtr &odom)
{
    if (odom->header.stamp.toSec() > ros::Time::now().toSec() - 1)
    {
        path_array[0] = odom->pose.pose.position.x;
        path_array[1] = odom->pose.pose.position.y;
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "global_planner_hardcode");
    ros::NodeHandle n("~");

    // sleep for 11 seconds for the ekf to generate the odometry
//    ros::Duration(11).sleep();

    n.param("odom_topic", odom_topic, string("/ekf_odom"));
    n.param("publisher_topic", publisher_topic, string("/global_path_hardcode"));
    path_pub = n.advertise<nav_msgs::Path>(publisher_topic, 100);
    ros::Subscriber sub = n.subscribe(odom_topic, 10, odom_callback);

    // 100 Hz path update rate
    ros::Rate r(100);

    path_array.resize(10);
    for (unsigned int i = 0; i < path_array.size(); i++) {
        path_array[i] = path_array_init[i];
    }

    while (ros::ok())
    {
        nav_msgs::Path path;
        path.header.frame_id = "world";
        path.header.stamp = ros::Time::now();

        for (unsigned int i = 0; i < path_array.size() / 2; i++) {
            geometry_msgs::PoseStamped pose;
            pose.pose.position.x = path_array[i * 2];
            pose.pose.position.y = path_array[i * 2 + 1];
            pose.pose.position.z = 0;

            pose.pose.orientation.w = 1;
            pose.pose.orientation.x = 0;
            pose.pose.orientation.y = 0;
            pose.pose.orientation.z = 0;

            path.poses.push_back(pose);
        }

        path_pub.publish(path);

        r.sleep();
        ros::spinOnce();
    }
}
