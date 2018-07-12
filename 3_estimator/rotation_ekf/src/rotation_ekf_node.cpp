/**
 * by Beck on 7/12/18.
 * rotation ekf used in the real system
 */
#include <iostream>
#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <queue>
#include "rm_cv/ArmorRecord.h"
#include "can_receive_msg/imu_16470.h"

using namespace std;
using namespace Eigen;

string predict_topic_in, preprocess_topic_in;
string publisher_topic, predict_pub_topic, preprocess_pub_topic;
ros::Publisher pose_pub, predict_pub, preprocess_pub;
Vector3d imu_T_camera = MatrixXd::Zero(3, 1);

/*
 * publish the predict result in angle axis
 */
void pub_pose(std_msgs::Header header, double angle, const Vector3d& axis, ros::Publisher &pub)
{
    geometry_msgs::PoseStamped pose;
    pose.header = header;
    pose.pose.orientation.w = angle;
    pose.pose.orientation.x = axis[0];
    pose.pose.orientation.y = axis[1];
    pose.pose.orientation.z = axis[2];
    pub.publish(pose);
}

void predict_callback(const geometry_msgs::TwistStamped::ConstPtr &pnp)
{
    Vector3d camera_T_shield_rot = MatrixXd::Zero(3, 1);
    camera_T_shield_rot <<  pnp->twist.linear.x,
                            pnp->twist.linear.y,
                            pnp->twist.linear.z;
    Vector3d imu_T_shield = imu_T_camera + camera_T_shield_rot;
    Vector3d T_norm = imu_T_shield.normalized();
    Vector3d x_axis = Vector3d::UnitX();
    Vector3d axis = T_norm.cross(x_axis).normalized();
    double angle = acos( T_norm.dot(x_axis) );
    AngleAxisd camera_R_shield(angle, axis);
    Quaterniond camera_q_shield(camera_R_shield);

    pub_pose(pnp->header, angle, axis, predict_pub);
}

void preprocess_callback(const geometry_msgs::TwistStamped::ConstPtr &pnp)
{
    Vector3d camera_T_shield_rot = MatrixXd::Zero(3, 1);
    camera_T_shield_rot <<  pnp->twist.linear.x,
            pnp->twist.linear.y,
            pnp->twist.linear.z;
    Vector3d imu_T_shield = imu_T_camera + camera_T_shield_rot;
    Vector3d T_norm = imu_T_shield.normalized();
    Vector3d x_axis = Vector3d::UnitX();
    Vector3d axis = T_norm.cross(x_axis).normalized();
    double angle = acos( T_norm.dot(x_axis) );
    AngleAxisd camera_R_shield(angle, axis);
    Quaterniond camera_q_shield(camera_R_shield);

    pub_pose(pnp->header, angle, axis, preprocess_pub);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rotation_ekf");
    ros::NodeHandle n("~");

    n.param("predict_result", predict_topic_in, string("/prediction_kf/predict"));
    n.param("preprocessed_result", preprocess_topic_in, string("/prediction_kf/preprocessed"));
    n.param("publisher_topic", publisher_topic, string("/rotation_ekf/filtered"));
    n.param("predict_topic", predict_pub_topic, string("/rotation_ekf/predict"));
    n.param("preprocess_topic", preprocess_pub_topic, string("/rotation_ekf/preprocess"));

    ros::Subscriber s1 = n.subscribe(predict_topic_in, 10, predict_callback);
    ros::Subscriber s2 = n.subscribe(preprocess_topic_in, 10, preprocess_callback);
    pose_pub = n.advertise<geometry_msgs::PoseStamped>(publisher_topic, 100);
    predict_pub = n.advertise<geometry_msgs::PoseStamped>(predict_pub_topic, 100);
    preprocess_pub = n.advertise<geometry_msgs::PoseStamped>(preprocess_pub_topic, 100);

    imu_T_camera << 150, 45, -30;

    ros::Rate r(100);
    ros::spin();
}

