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
#include <Eigen/Geometry>

using namespace std;
using namespace Eigen;

ros::Publisher pose_pub;
string imu_topic, publisher_topic;

Quaterniond q = Quaterniond::Identity();  // state
double sampleFreq = 400.0;  // sample frequency in Hz
double Kp = 0.5;   // proportional gain
double Ki = 0.1;   // integral gain
Vector3d integral = {0.0, 0.0, 0.0};


void pub_fused_pose(const sensor_msgs::Imu &msg)
{
    sensor_msgs::Imu imu_with_pose;
    imu_with_pose.header = msg.header;
    imu_with_pose.orientation.w = q.w();
    imu_with_pose.orientation.x = q.x();
    imu_with_pose.orientation.y = q.y();
    imu_with_pose.orientation.z = q.z();
    imu_with_pose.angular_velocity = msg.angular_velocity;
    imu_with_pose.angular_velocity_covariance = msg.angular_velocity_covariance;
    imu_with_pose.linear_acceleration = msg.linear_acceleration;
    imu_with_pose.linear_acceleration_covariance = msg.linear_acceleration_covariance;

    pose_pub.publish(imu_with_pose);
}


void imu_callback(const sensor_msgs::Imu::ConstPtr &imu)
{
    Vector3d halfd, halfe;

    Vector3d acc(imu->linear_acceleration.x, imu->linear_acceleration.y, imu->linear_acceleration.z);
    acc.normalize();
    Vector3d omg(imu->angular_velocity.x, imu->angular_velocity.y, imu->angular_velocity.z);
    // omg.normalize();
    double dt = 1.0 / sampleFreq;

    if (!((acc[0] == 0.0) && (acc[1] == 0.0) && (acc[2] == 0.0))) {
        // Estimate the gravity vector d from quaternion q
        // d = Im(q^-1 e_z q)
        halfd[0] = q.x() * q.z() - q.w() * q.y();
        halfd[1] = q.w() * q.x() + q.y() * q.z();
        halfd[2] = q.w() * q.w() - 0.5f + q.z() * q.z();

        // calculate error vector e = a x d
        halfe = acc.cross(halfd);
        
        if (Ki > 0.0) {
            // In = In-1 + Ki * dt * e
            integral += 2.0 * Ki * dt * halfe;
            omg += integral;
        }
        else {
            integral.setZero();
        }
        
        // Apply proportional feedback w' = w + Kp * e + In
        omg += 2.0 * Kp * halfe;
    }

    // Integrate rate of change using dq = 0.5 * q x w'
    omg *= 0.5 * dt;

    q.w() += (-q.x() * omg[0] - q.y() * omg[1] - q.z() * omg[2]);
    q.x() += (q.w() * omg[0] + q.y() * omg[2] - q.z() * omg[1]);
    q.y() += (q.w() * omg[1] - q.x() * omg[2] + q.z() * omg[0]);
    q.z() += (q.w() * omg[2] + q.x() * omg[1] - q.y() * omg[0]);
    
    q.normalize();
    pub_fused_pose(*imu);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "attitude_estimator_imu");
    ros::NodeHandle n("~");

    n.param("imu_raw", imu_topic, string("/dji_sdk/imu")); // 400Hz
    n.param("publisher_topic", publisher_topic, string("/attitude_estimator/imu"));

    ros::Subscriber s3 = n.subscribe(imu_topic, 400, imu_callback);
    pose_pub = n.advertise<sensor_msgs::Imu>(publisher_topic, 100);

    ros::Rate r(400);
    ros::spin();
}
