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

using namespace std;
using namespace Eigen;

ros::Publisher cmd_pub, debug_pub;
string camera_topic, publisher_topic;
double lp_yaw_alpha;
double lp_pit_alpha;
double x_target_in_mm;
double yaw_pre, pit_pre; // for the filter
double prev_x_err, prev_z_err; // for the feedback controller

/**
 * This controller output a feedforward for the pitch angle, a feedback for the yaw angle
 *  and a feedback for position
 * @param pnp
 */
void twist_callback(const geometry_msgs::TwistStamped::ConstPtr &pnp)
{
    Matrix3d chassis_R_camera;  // camera point in chassis frame
    chassis_R_camera <<  0, 0, 1,
                        -1, 0, 0,
                         0,-1, 0;

    // translation of the shield in camera and camera in gimbal, in millimeter
    Vector3d camera_T_shield(pnp->twist.linear.x, pnp->twist.linear.y, pnp->twist.linear.z);
    Vector3d chassis_T_shield = chassis_R_camera * camera_T_shield;
    double euler[3] = {0, 0, 0};
    Vector3d T = {x_target_in_mm , 0, 0};
    if (chassis_T_shield[0] != 0)
    {
        Vector3d chassis_T_gimbal(100, 0, -260);
        T = chassis_T_shield + chassis_T_gimbal;

        Quaterniond q_shield_in_gimbal;
        Vector3d T_norm = T.normalized();
        Vector3d x_axis(1, 0, 0);
        Vector3d axis = x_axis.cross(T_norm).normalized();
        // acos(x_axis.dot(T) / (T.norm() * x_axis.norm()));
        double angle = acos( x_axis.dot(T_norm) );

        // Debug purpose
        geometry_msgs::PoseStamped debug;
        debug.pose.position.x = T(0);
        debug.pose.position.y = T(1);
        debug.pose.position.z = T(2);
        debug.pose.orientation.w = angle;
        debug.pose.orientation.x = axis(0);
        debug.pose.orientation.y = axis(1);
        debug.pose.orientation.z = axis(2);
        debug_pub.publish(debug);

        // Rotation matrix to ZYX Euler Angle
        Matrix3d R = AngleAxisd(angle, axis).toRotationMatrix();

        euler[0] = atan2(R(1,0), R(0,0));
        euler[1] = acos(R(1,0) / sin(euler[0]));
        euler[2] = atan2(R(2,1), R(2,2));
    }

    // low pass filter
    pit_pre = lp_pit_alpha * pit_pre + (1 - lp_pit_alpha) * euler[1];
    yaw_pre = lp_yaw_alpha * yaw_pre + (1 - lp_yaw_alpha) * euler[0];

    // feedback controller
    ros::NodeHandle n("~");
    double x_kp = 0, x_kd = 0, z_kp = 0, z_kd = 0;
    if (n.hasParam("/server_node/x_kp")) {
        n.getParam("/server_node/x_kp", x_kp);
        n.getParam("/server_node/x_kd", x_kd);
        n.getParam("/server_node/z_kp", z_kp);
        n.getParam("/server_node/z_kd", z_kd);
    }

    double x_err = (T[0] - x_target_in_mm) * 0.001; // convert to meter
    double z_err = yaw_pre;
    double vx = x_kp * x_err + x_kd * (x_err - prev_x_err);
    double vz = z_kp * z_err + z_kd * (z_err - prev_z_err);
    prev_x_err = x_err;
    prev_z_err = z_err;

    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = vx;
    // cmd_vel.angular.x = euler[2]; // DEBUG only
    cmd_vel.angular.y = pit_pre;
    cmd_vel.angular.z = vz;
    cmd_pub.publish(cmd_vel);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "gimbal_controller");
    ros::NodeHandle n("~");

    n.param("lp_yaw_alpha", lp_yaw_alpha, 0.9);
    n.param("lp_pit_alpha", lp_pit_alpha, 0.9);
    n.param("x_target_in_mm", x_target_in_mm, 1000.0);
    n.param("camera_twist", camera_topic, string("/beckThing"));
    n.param("publisher_topic", publisher_topic, string("/cmd_vel"));

    ros::Subscriber sub = n.subscribe(camera_topic, 10, twist_callback);
//    ros::Subscriber sub = n.subscribe(camera_topic, 10, armor_callback);
    cmd_pub = n.advertise<geometry_msgs::Twist>(publisher_topic, 10);
    debug_pub = n.advertise<geometry_msgs::PoseStamped>(string("/gimbal_controller_debug"), 10);

    yaw_pre = 0;
    pit_pre = 0;

    ros::spin();
}

