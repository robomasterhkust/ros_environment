/**
 * Beck Pang, 20180613, convert frames from camera to gimbal
 */

#include <iostream>
#include <ros/ros.h>
#include <cmath>
#include <Eigen/Eigen>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
//#include "rm_cv/ArmorRecord.h"

using namespace std;
using namespace Eigen;

ros::Publisher cmd_pub, debug_pub;
string camera_topic, publisher_topic;
double lp_yaw_alpha;
double lp_pit_alpha;
double yaw_pre, pit_pre;

void twist_callback(const geometry_msgs::TwistStamped::ConstPtr &pnp)
{
    Matrix3d camera_R_chassis;
    camera_R_chassis <<  0, 0, 1,
                        -1, 0, 0,
                         0,-1, 0;

    // translation of the shield in camera and camera in gimbal, in millimeter
    Vector3d shield_T_camera(pnp->twist.linear.x, pnp->twist.linear.y, pnp->twist.linear.z);
    Vector3d shield_T_chassis = camera_R_chassis * shield_T_camera;
    Vector3d chassis_T_gimbal(200, 0, -350);
    Vector3d T = shield_T_chassis + chassis_T_gimbal; // shield_T_camera

    // Debug purpose
    geometry_msgs::PoseStamped debug;
    debug.pose.position.x = T(0);
    debug.pose.position.y = T(1);
    debug.pose.position.z = T(2);
    debug_pub.publish(debug);

    double yaw_target = atan2(T(1), T(0));
    double pit_target = asin(T(2) / T.norm());

    // low pass filter

    yaw_pre = lp_yaw_alpha * yaw_pre + (1 - lp_yaw_alpha) * yaw_target;
    pit_pre = lp_yaw_alpha * yaw_pre + (1 - lp_yaw_alpha) * pit_target;

    geometry_msgs::Twist cmd_vel;
    cmd_vel.angular.y = yaw_pre;
    cmd_vel.angular.z = pit_pre;
    cmd_pub.publish(cmd_vel);
}
/*

void armor_callback(const rm_cv::ArmorRecord::ConstPtr &pnp)
{
    Matrix3d camera_R_chassis;
    camera_R_chassis <<  0, 0, 1,
            -1, 0, 0,
            0,-1, 0;

    // translation of the shield in camera and camera in gimbal, in millimeter
    Vector3d shield_T_camera(pnp->armorCoordintae.linear.x, pnp->armorCoordintae.linear.x, pnp->armorCoordintae.linear.x);
    Vector3d shield_T_chassis = camera_R_chassis * shield_T_camera;
    Vector3d chassis_T_gimbal(200, 0, -350);
    Vector3d T = shield_T_chassis + chassis_T_gimbal; // shield_T_camera

    // Debug purpose
    geometry_msgs::PoseStamped debug;
    debug.pose.position.x = T(0);
    debug.pose.position.y = T(1);
    debug.pose.position.z = T(2);
    debug_pub.publish(debug);

    double yaw_target = atan2(T(1), T(0));
    double pit_target = asin(T(2) / T.norm());

    // low pass filter

    yaw_pre = lp_yaw_alpha * yaw_pre + (1 - lp_yaw_alpha) * yaw_target;
    pit_pre = lp_yaw_alpha * yaw_pre + (1 - lp_yaw_alpha) * pit_target;

    geometry_msgs::Twist cmd_vel;
    cmd_vel.angular.y = yaw_pre;
    cmd_vel.angular.z = pit_pre;
    cmd_pub.publish(cmd_vel);
}
*/

int main(int argc, char **argv) {
    ros::init(argc, argv, "gimbal_controller");
    ros::NodeHandle n("~");

    n.param("lp_yaw_alpha", lp_yaw_alpha, 0.9);
    n.param("lp_pit_alpha", lp_pit_alpha, 0.9);
    n.param("camera_twist", camera_topic, string("/gimbal_detected_armor"));
    n.param("publisher_topic", publisher_topic, string("/gimbal_cmd_vel"));

    ros::Subscriber sub = n.subscribe(camera_topic, 10, twist_callback);
//    ros::Subscriber sub = n.subscribe(camera_topic, 10, armor_callback);
    cmd_pub = n.advertise<geometry_msgs::Twist>(publisher_topic, 10);
    debug_pub = n.advertise<geometry_msgs::PoseStamped>(string("/gimbal_controller_debug"), 10);

    yaw_pre = 0;
    pit_pre = 0;

    ros::spin();
}

