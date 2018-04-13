/**
 * Practice Extended Kalman Filter for eight states
 * Fusing an high precision gyroscope and a MPU6500 IMU as the process model,
 *   with an UWB and a magnetometer as the observation model.
 */
#include <iostream>
#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Range.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Eigen>
#include <queue>
#include <vector>
#include <cmath>

#define INIT_Q_R_BY_MEASURE

using namespace std;
using namespace Eigen;
ros::Publisher odom_pub;

/**
 * Define states:
 *      x = [pos_x, pos_y, angle_yaw, vel_x, vel_y, bias_gyro, bias_accel_x, bias_accel_y]
 * Define inputs:
 *      u = [angular_vel, accel_x, accel_y], all measurement
 * Define noises:
 *      n = [n_gyro, n_acc_x, n_acc_y, n_bias_gyro, n_bias_acc_x, n_bias_acc_y]
 */
VectorXd x(8);                          // state
MatrixXd P = MatrixXd::Identity(8, 8);  // covariance
MatrixXd Q = MatrixXd::Identity(6, 6);  // prediction noise covariance
MatrixXd R = MatrixXd::Identity(3, 3);  // observation noise covariance

// Buffers to save imu reading and the uwb reading
// for time synchronization
queue<sensor_msgs::Imu::ConstPtr> imu_buf;
queue<nav_msgs::Odometry::ConstPtr> odom_buf;
queue<Matrix<double, 8, 1>>     x_history;
queue<Matrix<double, 8, 8>>     P_history;

double t;       //  previous propagated time

// For initialization
int imu_count = 0;
const int IMU_INIT_COUNT = 30;
Vector3d imu_mean_buf[IMU_INIT_COUNT]; // gyro, accel_x, accel_y
queue<double> odom_mean_buf_x;
queue<double> odom_mean_buf_y;
queue<double> odom_mean_buf_theta;
bool imu_initialized = false;
bool odom_initialized= false;

/**
 * non-linear propagate for EKF
 * with linearization around the current state
 * @param imu_msg
 */
void propagate(const sensor_msgs::ImuConstPtr &imu_msg)
{
    double cur_t = imu_msg->header.stamp.toSec();
    double w    = imu_msg->angular_velocity.z;
    double ax   = imu_msg->linear_acceleration.x;
    double ay   = imu_msg->linear_acceleration.y;

    double dt   = cur_t - t;
//    if (dt <= 0) {  ROS_BREAK; }

    /**
     * f(x, u, n) =
     *      x4,
     *      x5,
     *      w_m - x6 - ng,
     *      cos(x3) (a_mx - x7 - n_ax) + sin(x3) (a_my - x8 - n_ay),
     *      -sin(x3) (a_mx - x7 - n_ax) + cos(x3) (a_my - x8 - n_ay),
     *      n_bg,
     *      n_bax,
     *      n_bay.
     * u_t_hat = u_t-1 + dt * f(u_t-1, u_t, 0)
     */
    x(0) += dt * x(3);
    x(1) += dt * x(4);
    x(2) += dt * (w - x(5));
    x(3) += dt * ( cos(x(2)) * (ax - x(6)) + sin(x(2)) * (ay - x(7)) );
    x(4) += dt * (-sin(x(2)) * (ax - x(6)) + cos(x(2)) * (ay - x(7)) );
//    x(5) += dt * 0;
//    x(6) += dt * 0;
//    x(7) += dt * 0;

    MatrixXd A = MatrixXd::Zero(8, 8);
    A(0, 3) =  1;
    A(1, 4) =  1;
    A(2, 5) = -1;
    // df4 / dx3
    A(3, 2) = -sin(x(2)) * (ax - x(6)) + cos(x(2)) * (ay - x(7));
    // df5 / dx3
    A(4, 2) = -cos(x(2)) * (ax - x(6)) - sin(x(2)) * (ay - x(7));
    A(3, 6) = -cos(x(2));
    A(4, 6) =  sin(x(2));
    A(3, 7) = -sin(x(2));
    A(4, 7) = -cos(x(2));

    MatrixXd U = MatrixXd::Zero(8, 6);
    U(2, 0) = -1;
    U(3, 1) = -cos(x(2));
    U(4, 1) = -sin(x(2));
    U(3, 2) =  sin(x(2));
    U(4, 2) = -cos(x(2));
    U.block<3,3>(5, 3) = MatrixXd::Identity(3, 3);

    MatrixXd F, V;
    F = dt * A + MatrixXd::Identity(8, 8);
    V = dt * U;
    /**
     * P_t_hat = F * P_t * F' + V * Q * V'
     */
    P = F * P * F.transpose() + V * Q * V.transpose();

    t = cur_t;
}

/**
 * linear update for EKF
 * @param msg
 */
void update(const nav_msgs::Odometry::ConstPtr &msg)
{
    MatrixXd C = MatrixXd::Zero(3, 8);
    C.block<3, 3>(0, 0) = Matrix3d::Identity();

    double pos_x = msg->pose.pose.position.x;
    double pos_y = msg->pose.pose.position.y;
    double angle_yaw = msg->pose.pose.orientation.z;
    Vector3d y(pos_x, pos_y, angle_yaw);

    MatrixXd K(8, 3);
    K = P * C.transpose() * (C * P * C.transpose() + R).inverse();
    x = x + K * (y - C * x);
    P = P - K * C * P;
}

void pub_odom(std_msgs::Header header)
{
    nav_msgs::Odometry odom;
    odom.header.stamp = header.stamp;
    odom.header.frame_id = "world";
    odom.pose.pose.position.x   = x(0);
    odom.pose.pose.position.y   = x(1);
    odom.pose.pose.orientation.z= x(2);
    odom.twist.twist.linear.x   = x(3);
    odom.twist.twist.linear.y   = x(4);
    odom.pose.covariance[0]     = P(0, 0);
    odom.pose.covariance[7]     = P(1, 1);
    odom.pose.covariance[35]    = P(2, 2);
    odom.twist.covariance[0]    = P(3, 3);
    odom.twist.covariance[7]    = P(4, 4);

    odom_pub.publish(odom);
}

void imu_callback(const sensor_msgs::Imu::ConstPtr &imu_msg)
{
    ROS_INFO("IMU callback, time: %f", imu_msg->header.stamp.toSec());

    if (!imu_initialized && imu_count < IMU_INIT_COUNT)
    {
        // calculate the imu covariance and mean, and initialize the gravity
        imu_mean_buf[imu_count](0) = imu_msg->angular_velocity.z;
        imu_mean_buf[imu_count](1) = imu_msg->linear_acceleration.x;
        imu_mean_buf[imu_count](2) = imu_msg->linear_acceleration.y;
        imu_count++;
    }
    else if (!imu_initialized && imu_count == IMU_INIT_COUNT)
    {
#ifdef INIT_Q_R_BY_MEASURE
        double imu_mean[3] = {0};
        double imu_cova[3]= {0};
        for (int i = 0; i < imu_count; ++i) {
            imu_mean[0] += imu_mean_buf[i](0);
            imu_mean[1] += imu_mean_buf[i](1);
            imu_mean[2] += imu_mean_buf[i](2);
            imu_cova[0] += pow( imu_mean_buf[i](0), 2 );
            imu_cova[1] += pow( imu_mean_buf[i](1), 2 );
            imu_cova[2] += pow( imu_mean_buf[i](2), 2 );
        }
        imu_mean[0] /= imu_count;
        imu_mean[1] /= imu_count;
        imu_mean[2] /= imu_count;
        imu_cova[0] /= imu_count;
        imu_cova[1] /= imu_count;
        imu_cova[2] /= imu_count;
        Q(0, 0) = imu_cova[0] - pow( imu_mean[0], 2 );
        Q(1, 1) = imu_cova[1] - pow( imu_mean[1], 2 );
        Q(2, 2) = imu_cova[2] - pow( imu_mean[2], 2 );
#endif
        imu_initialized = true;
    }
    else {
        imu_buf.push(imu_msg);
        propagate(imu_msg);
        x_history.push(x);
        P_history.push(P);
        pub_odom(imu_msg->header);
    }
}

void odom_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
    ROS_INFO("odom callback, time: %f", msg->header.stamp.toSec());

    if (!imu_initialized && !odom_initialized)
    {
        // calculate the uwb covariance and mean
        odom_mean_buf_x.push(msg->pose.pose.position.x);
        odom_mean_buf_y.push(msg->pose.pose.position.y);
        odom_mean_buf_theta.push(msg->pose.pose.orientation.z);
    }
    else if (imu_initialized && !odom_initialized)
    {
#ifdef INIT_Q_R_BY_MEASURE
        double odom_mean[3] = {0};
        double odom_cova[3] = {0};
        int odom_count = (int)odom_mean_buf_x.size();
        for (int i = 0; i < odom_count; ++i) {
            double temp_x = odom_mean_buf_x.front();
            double temp_y = odom_mean_buf_y.front();
            double temp_a = odom_mean_buf_theta.front();
            odom_mean[0] += temp_x;
            odom_mean[1] += temp_y;
            odom_mean[2] += temp_a;
            odom_cova[0] += pow( temp_x, 2 );
            odom_cova[1] += pow( temp_y, 2 );
            odom_cova[2] += pow( temp_a, 2 );
            odom_mean_buf_x.pop();
            odom_mean_buf_y.pop();
            odom_mean_buf_theta.pop();
        }
        odom_mean[0] /= odom_count;
        odom_mean[1] /= odom_count;
        odom_mean[2] /= odom_count;
        odom_cova[0] /= odom_count;
        odom_cova[1] /= odom_count;
        odom_cova[2] /= odom_count;
        R(0, 0) = odom_cova[0] - pow( odom_mean[0], 2 );
        R(1, 1) = odom_cova[1] - pow( odom_mean[1], 2 );
        R(2, 2) = odom_cova[2] - pow( odom_mean[2], 2 );
#endif
        odom_initialized = true;
    }
    else {
        while (!imu_buf.empty() && imu_buf.front()->header.stamp < msg->header.stamp)
        {
            ROS_INFO("throw state with time: %f", imu_buf.front()->header.stamp.toSec());
            // trace the time backwards to imu time
            t = imu_buf.front()->header.stamp.toSec();
            imu_buf.pop();
            x_history.pop();
            P_history.pop();
        }
        if (!x_history.empty())
        {
            // if x_history is empty then the odom is the same time as the imu
            x = x_history.front();
            P = P_history.front();
            // trace the time backwards to imu time
            t = imu_buf.front()->header.stamp.toSec();
            imu_buf.pop();
            x_history.pop();
            P_history.pop();
        }
        ROS_INFO("update state with time: %f", msg->header.stamp.toSec());
        update(msg);

        // clean the x and P history before new propagate
        while(!x_history.empty()) x_history.pop();
        while(!P_history.empty()) P_history.pop();

        queue<sensor_msgs::Imu::ConstPtr> temp_imu_buf;
        while (!imu_buf.empty())
        {
            ROS_INFO("propagate state with time: %f", imu_buf.front()->header.stamp.toSec());
            propagate(imu_buf.front());
            temp_imu_buf.push(imu_buf.front());
            x_history.push(x);
            P_history.push(P);
            imu_buf.pop();
        }
        std::swap(imu_buf, temp_imu_buf);
    }
}

String imu_topic, uwb_topic, publiser_topic;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ekf_8states");
    ros::NodeHandle n("~");

    n.param("imu_topic", imu_topic, "/dji_sdk/imu");
    n.param("uwb_topic", uwb_topic, "/uwb");
    n.param("publiser_topic", publiser_topic, "/ekf_odom");
    ros::Subscriber s1 = n.subscribe(imu_topic, 100, imu_callback);
    ros::Subscriber s2 = n.subscribe(uwb_topic, 10, odom_callback);
    odom_pub = n.advertise<nav_msgs::Odometry>(publiser_topic, 100);
    ros::Rate r(100);

    odom_initialized = true;

    // gyroscope noise n_g
    Q(0, 0) = 0.0001 * Q(0, 0);
    // accelerometer noise n_ax, n_ay
    Q(1, 1) = 0.01 * Q(1, 1);
    Q(2, 2) = 0.01 * Q(2, 2);
    // gyroscope bias noise n_bg
    Q(3, 3) = 0.000001 * Q(3, 3);
    // accelerometer bias noise n_ba
    Q(4, 4) = 0.0001 * Q(4, 4);
    Q(5, 5) = 0.0001 * Q(5, 5);

    // uwb noise n_x, n_y
    R(0, 0) = 0.001 * R(0, 0);
    R(1, 1) = 0.001 * R(1, 1);
    // magnetometer noise n_theta
    R(2, 2) = 0.01 * R(2, 2);

    ros::spin();
}