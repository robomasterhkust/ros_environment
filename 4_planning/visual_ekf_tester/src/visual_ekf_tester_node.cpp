/**
 * trajectory generator for testing the visual ekf
 */
#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TwistStamped.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "qp_generator.h"

using namespace std;
using namespace Eigen;
ros::Publisher imu_pub, visual_pub, debug_pub;

const int update_freq = 400; // 1 / dt
const double dt = 0.0025;

void pub_imu(const Vector3d& acc, const Quaterniond& orientation){
    sensor_msgs::Imu imu;
    imu.header.stamp = ros::Time::now();
    imu.orientation.w = orientation.w();
    imu.orientation.x = orientation.x();
    imu.orientation.y = orientation.y();
    imu.orientation.z = orientation.z();
    imu.linear_acceleration.x = acc[0];
    imu.linear_acceleration.y = acc[1];
    imu.linear_acceleration.z = acc[2] + 9.8;

    imu_pub.publish(imu);
}

/**
 * Change the world frame position to the camera frame, and publish in millimeter
 * @param pos
 */
void pub_visual(const Vector3d& pos) {
    geometry_msgs::TwistStamped pose;
    pose.header.stamp = ros::Time::now();

    MatrixXd imu_R_camera = MatrixXd::Identity(3, 3);
    imu_R_camera <<  0, 0, 1,
                    -1, 0, 0,
                     0,-1, 0;

    Vector3d pos_camera = imu_R_camera.transpose() * (1000 * pos);
    pose.twist.linear.x = pos_camera[0];
    pose.twist.linear.y = pos_camera[1];
    pose.twist.linear.z = pos_camera[2];

    visual_pub.publish(pose);
}

void pub_debug(const Vector3d& pos, const Vector3d& vel, const Vector3d& acc)
{
    nav_msgs::Odometry odom;
    odom.header.stamp = ros::Time::now();

    odom.child_frame_id = "world";
    odom.pose.pose.position.x = pos[0];
    odom.pose.pose.position.y = pos[1];
    odom.pose.pose.position.z = pos[2];
    odom.twist.twist.linear.x = vel[0];
    odom.twist.twist.linear.y = vel[1];
    odom.twist.twist.linear.z = vel[2];
    odom.twist.twist.angular.x= acc[0];
    odom.twist.twist.angular.y= acc[1];
    odom.twist.twist.angular.z= acc[2];

    debug_pub.publish(odom);
}

// get position from coefficient
void getPositionFromCoeff(Eigen::Vector3d &pos, const Eigen::MatrixXd &coeff,
                                             const int &index, const double &time)
{
    int s = index;
    double t = time;
    double x = coeff(s, 0) + coeff(s, 1) * t + coeff(s, 2) * pow(t, 2) + coeff(s, 3) * pow(t, 3) +
               coeff(s, 4) * pow(t, 4) + coeff(s, 5) * pow(t, 5);
    double y = coeff(s, 6) + coeff(s, 7) * t + coeff(s, 8) * pow(t, 2) + coeff(s, 9) * pow(t, 3) +
               coeff(s, 10) * pow(t, 4) + coeff(s, 11) * pow(t, 5);
    double z = coeff(s, 12) + coeff(s, 13) * t + coeff(s, 14) * pow(t, 2) + coeff(s, 15) * pow(t, 3) +
               coeff(s, 16) * pow(t, 4) + coeff(s, 17) * pow(t, 5);

    pos(0) = x;
    pos(1) = y;
    pos(2) = z;
}

// get velocity from cofficient
void getVelocityFromCoeff(Eigen::Vector3d &vel, const Eigen::MatrixXd &coeff,
                                             const int &index, const double &time)
{
    int s = index;
    double t = time;
    double vx = coeff(s, 1) + 2 * coeff(s, 2) * pow(t, 1) + 3 * coeff(s, 3) * pow(t, 2) +
                4 * coeff(s, 4) * pow(t, 3) + 5 * coeff(s, 5) * pow(t, 4);
    double vy = coeff(s, 7) + 2 * coeff(s, 8) * pow(t, 1) + 3 * coeff(s, 9) * pow(t, 2) +
                4 * coeff(s, 10) * pow(t, 3) + 5 * coeff(s, 11) * pow(t, 4);
    double vz = coeff(s, 13) + 2 * coeff(s, 14) * pow(t, 1) + 3 * coeff(s, 15) * pow(t, 2) +
                4 * coeff(s, 16) * pow(t, 3) + 5 * coeff(s, 17) * pow(t, 4);

    vel(0) = vx;
    vel(1) = vy;
    vel(2) = vz;
}

// get acceleration from coefficient
void getAccelerationFromCoeff(Eigen::Vector3d &acc, const Eigen::MatrixXd &coeff,
                                                 const int &index, const double &time)
{
    int s = index;
    double t = time;
    double ax = 2 * coeff(s, 2) + 6 * coeff(s, 3) * pow(t, 1) + 12 * coeff(s, 4) * pow(t, 2) +
               20 * coeff(s, 5) * pow(t, 3);
    double ay = 2 * coeff(s, 8) + 6 * coeff(s, 9) * pow(t, 1) + 12 * coeff(s, 10) * pow(t, 2) +
               20 * coeff(s, 11) * pow(t, 3);
    double az = 2 * coeff(s, 14) + 6 * coeff(s, 15) * pow(t, 1) + 12 * coeff(s, 16) * pow(t, 2) +
               20 * coeff(s, 17) * pow(t, 3);

    acc(0) = ax;
    acc(1) = ay;
    acc(2) = az;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "visual_ekf_tester");
    ros::NodeHandle n("~");

    imu_pub    = n.advertise<sensor_msgs::Imu>("/visual_ekf/test_imu", 400);
    visual_pub = n.advertise<geometry_msgs::TwistStamped>("/visual_ekf/test_visual", 50);
    debug_pub  = n.advertise<nav_msgs::Odometry>("/visual_ekf_tester/odom", 50);

    ros::Rate r(update_freq);

    MatrixXd Path = MatrixXd::Zero(3, 3);
    Path.block<1, 3>(0, 0) = Vector3d(0.6, 0.2, 0.1);
    Path.block<1, 3>(1, 0) = Vector3d(0.8, 0.2, 0.1);
    Path.block<1, 3>(2, 0) = Vector3d(1, 0, 0);

    Vector3d Vel_init(0, 0, 0);
    Vector3d Acc_init(0, 0, 0);
    VectorXd time_period = MatrixXd::Zero(2, 1);
    time_period << 5.0, 5.0;
    VectorXd time_int = time_period;

    // generate a smooth curve
    TrajectoryGenerator generator;
    MatrixXd coeff = generator.PolyQPGeneration(
            Path, Vel_init, Acc_init, time_period, 1);
    cout << "polynomial " << endl << coeff << endl;

    auto t_start = ros::Time::now().toSec();
    int iterator = 0;
    auto time_period_size = (int)time_period.size();

    // provide the orientation, rotate in Z axis first
    Vector3d axis_z = Vector3d::UnitZ();
    VectorXd theta = MatrixXd::Zero(3, 1);
    theta[0] = 0.0;
    theta[1] = 0.1 * M_PI;
    theta[2] = 0.25 * M_PI;

    while (ros::ok()) {

        double dt = ros::Time::now().toSec() - t_start;

        if (iterator < time_period_size) {
            if (dt > time_period[iterator]) {
                dt -= time_period[iterator];
                t_start += time_period[iterator];
                iterator++;
                ROS_INFO("In polynomial section %d", iterator);
            }
        }

        if (iterator < time_period_size) {
            Vector3d pos, vel, acc;
            getPositionFromCoeff(pos, coeff, iterator, dt);
            getVelocityFromCoeff(vel, coeff, iterator, dt);
            getAccelerationFromCoeff(acc, coeff, iterator, dt);

            double omg = (theta[iterator+1] - theta[iterator]) / time_period[iterator];
            double angle = omg * dt + theta[iterator];
            AngleAxisd orientation(angle, axis_z);
            Quaterniond q(orientation);

            Vector3d acc_imu = q.conjugate() * acc;
            pub_imu(acc_imu, q);

            Vector3d pos_imu = q.conjugate() * pos;
            pub_visual(pos_imu);

            Vector3d vel_imu = q.conjugate() * vel;
            pub_debug(pos_imu, vel_imu, acc_imu);
        }

        r.sleep();
        ros::spinOnce();
    }
}
