//
// Created by Beck on 7/3/18.
// Mahony filter for attitude estimation
// http://www.olliw.eu/2013/imu-data-fusing/#mahonycode
//

#include <iostream>
#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Imu.h>
// #include <Eigen/Eigen>
#include <Eigen/Geometry>
#include <queue>

using namespace std;
using namespace Eigen;

ros::Publisher pose_pub;
string imu_topic, publisher_topic;


Quaterniond q = Quaterniond::Identity();  // state
double sampleFreq = 400.0;  // sample frequency in Hz
double Kp = 0.5;   // proportional gain
double Ki = 0.1;   // integral gain
Vector3d integral = {0.0, 0.0, 0.0};


void pub_fused_pose(std_msgs::Header header)
{
    geometry_msgs::PoseStamped pose;
    pose.header = header;
    pose.pose.orientation.w = q.w();
    pose.pose.orientation.x = q.x();
    pose.pose.orientation.y = q.y();
    pose.pose.orientation.z = q.z();

    pose_pub.publish(pose);
}


void imu_callback(const sensor_msgs::Imu::ConstPtr &imu)
{
    double halfvx, halfvy, halfvz;
    Vector3d halfe;

    Vector3d acc(imu->linear_acceleration.x, imu->linear_acceleration.y, imu->linear_acceleration.z);
    acc.normalize();
    Vector3d omg(imu->angular_velocity.x, imu->angular_velocity.y, imu->angular_velocity.z);
    omg.normalize();

    if (!((acc[0] == 0.0) && (acc[1] == 0.0) && (acc[2] == 0.0))) {
        halfvx = q.x() * q.z() - q.w() * q.y();
        halfvy = q.w() * q.x() + q.y() * q.z();
        halfvz = q.w() * q.w() - 0.5f + q.z() * q.z();

        halfe[0] = (acc[1] * halfvz - acc[2] * halfvy);
        halfe[1] = (acc[2] * halfvx - acc[0] * halfvz);
        halfe[2] = (acc[0] * halfvy - acc[1] * halfvx);
        
        if (Ki > 0.0) {
            integral += 2.0 * Ki * halfe * (1.0 / sampleFreq);
            omg += integral;
        }
        else {
            integral.setZero();
        }
        
        // Apply proportional feedback
        omg += 2.0 * Kp * halfe;
    }
    
    omg *= 0.5 * (1.0 / sampleFreq);

    q.w() += (-q.x() * omg[0] - q.y() * omg[1] - q.z() * omg[2]);
    q.x() += (q.w() * omg[0] + q.y() * omg[2] - q.z() * omg[1]);
    q.y() += (q.w() * omg[1] - q.x() * omg[2] + q.z() * omg[0]);
    q.z() += (q.w() * omg[2] + q.x() * omg[1] - q.y() * omg[0]);
    
    q.normalize();
    pub_fused_pose(imu->header);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "attitude_estimator_imu");
    ros::NodeHandle n("~");

    n.param("imu_raw", imu_topic, string("/dji_sdk/imu")); // 400Hz
    n.param("publisher_topic", publisher_topic, string("/attitude_estimator/pose"));

    ros::Subscriber s3 = n.subscribe(imu_topic, 400, imu_callback);
    pose_pub = n.advertise<geometry_msgs::PoseStamped>(publisher_topic, 100);
//    pose_pub = n.advertise<geometry_msgs::PoseWithCovarianceStamped>(publisher_topic, 100);

    ros::Rate r(400);
    ros::spin();
}
