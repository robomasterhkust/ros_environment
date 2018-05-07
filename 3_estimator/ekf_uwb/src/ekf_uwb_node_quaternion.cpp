/**
 * Created by beck on 24/4/2018.
 * Extended Kalman Filter for 16 states, with quaternion for the orientation
 * Fused A3 flight controller IMU with UWB x and y position
 * With IMU raw reading comes in 400Hz and UWB raw reading in 50Hz
 */
#include <iostream>
#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>
#include <uwb_msgs/uwb.h>
#include <Eigen/Eigen>
#include <queue>

// #define INIT_Q_R_BY_MEASUREMENT

using namespace std;
using namespace Eigen;
ros::Publisher odom_pub;
string imu_topic, uwb_topic, publisher_topic;

/**
 * Define states:
 *      x = [rotation_quaternion, position, velocity, bias_accel, bias_gyro]
 * Define inputs:
 *      u = [omg, accel], all imu measurement
 * Define noises:
 *      n = [n_gyro, n_acc, n_bias_acc, n_bias_gyro]
 */
VectorXd x(16);                             // state
MatrixXd P = MatrixXd::Zero(15, 15);        // covariance
MatrixXd Q = MatrixXd::Identity(12, 12);    // prediction noise covariance
MatrixXd R = MatrixXd::Identity(6, 6);      // observation noise covariance

// buffers to save imu and uwb reading for time synchronization
queue <sensor_msgs::Imu::ConstPtr> imu_buf;
queue <nav_msgs::Odometry::ConstPtr> odom_buf;
queue <Matrix<double, 16, 1>> x_history;
queue <Matrix<double, 15, 15>> P_history;

// previous propagated time
double t_prev;

// Initialization and covariance estimation
// UWB wait until imu is initialized
const int IMU_INIT_COUNT = 40;
int imu_count = 0;
Vector3d g_init = Vector3d::Zero();
Vector3d G = Vector3d::Zero();
Vector3d accl_init_buf[IMU_INIT_COUNT];
Vector3d gyro_init_buf[IMU_INIT_COUNT];
queue<double> uwb_init_buf_w;
queue<double> uwb_init_buf_x;
queue<double> uwb_init_buf_y;
double theta_bias = 0;
bool imu_initialized = false;
bool odom_initialized = false;

// TODO: Calibrate sensor position
// imu frame to world frame rotation matrix, {[0, 0, -1], pi/2}
Matrix3d imu_R_world = Quaterniond(sqrt(1/2), 0, 0, -sqrt(1/2)).toRotationMatrix();
Matrix3d init_robot_pose = Quaterniond(sqrt(1/2), 0, 0, -sqrt(1/2)).toRotationMatrix();

void pub_odom_ekf(std_msgs::Header header) {
    nav_msgs::Odometry odom;
    odom.header.stamp = header.stamp;
    odom.header.frame_id = "world";
    odom.pose.pose.orientation.w = x(0);
    odom.pose.pose.orientation.x = x(1);
    odom.pose.pose.orientation.y = x(2);
    odom.pose.pose.orientation.z = x(3);
    odom.pose.pose.position.x = x(4);
    odom.pose.pose.position.y = x(5);
    odom.pose.pose.position.z = x(6);
    odom.twist.twist.linear.x = x(7);
    odom.twist.twist.linear.y = x(8);
    odom.twist.twist.linear.z = x(9);

    odom_pub.publish(odom);
}

// imu propagate in the world frame
void propagate(const sensor_msgs::Imu::ConstPtr &imu_msg) {
    double cur_t = imu_msg->header.stamp.toSec();
    double dt = cur_t - t_prev;
    VectorXd w_raw(3);
    VectorXd a_raw(3);
    a_raw(0) = imu_msg->linear_acceleration.x;
    a_raw(1) = imu_msg->linear_acceleration.y;
    a_raw(2) = imu_msg->linear_acceleration.z;
    w_raw(0) = imu_msg->angular_velocity.x;
    w_raw(1) = imu_msg->angular_velocity.y;
    w_raw(2) = imu_msg->angular_velocity.z;

    Vector3d a = a_raw - x.segment<3>(10);
    Vector3d omg = w_raw - x.segment<3>(13);
    Vector3d domg = 0.5 * dt * omg;

    // propagate the state with quaternion calculus
    Quaterniond dR(sqrt(1 - domg.squaredNorm()), domg(0), domg(1), domg(2));
    Quaterniond Rt(x(0), x(1), x(2), x(3));
    Quaterniond R_t = (Rt * dR).normalized();

    x.segment<4>(0) << R_t.w(), R_t.x(), R_t.y(), R_t.z();
    x.segment<3>(4) += x.segment<3>(7) * dt + (Rt * (a - G)) * 0.5 * dt * dt;
    x.segment<3>(7) += (Rt * (a - G)) * dt;

    // propagate the covariance with skew-symmetric matrix
    MatrixXd I = MatrixXd::Identity(3, 3);
    Matrix3d R_omg, R_a;
    R_omg << 0, -omg(2), omg(1),
            omg(2), 0, -omg(0),
            -omg(1), omg(0), 0;
    R_a << 0, -a(2), a(1),
            a(2), 0, -a(0),
            -a(1), a(0), 0;

    MatrixXd A = MatrixXd::Zero(15, 15);
    A.block<3, 3>(0, 0) = -R_omg;
    A.block<3, 3>(12, 0) = -1 * I;
    A.block<3, 3>(3, 6) = I;
    A.block<3, 3>(6, 0) = (-1 * Rt.toRotationMatrix()) * R_a;
    A.block<3, 3>(6, 9) = (-1 * Rt.toRotationMatrix());
    // cout << "DEBUG:: propagate A" << endl << A << endl;

    MatrixXd U = MatrixXd::Zero(15, 12);
    U.block<3, 3>(0, 0) = -1 * I;
    U.block<3, 3>(6, 3) = -1 * Rt.toRotationMatrix();
    U.block<3, 3>(9, 6) = I;
    U.block<3, 3>(12, 9) = I;
    // cout << "DEBUG:: propagate U" << endl << U << endl;

    MatrixXd F, V;
    F = MatrixXd::Identity(15, 15) + dt * A;
    V = dt * U;

    P = F * P * F.transpose() + V * Q * V.transpose();
    // cout << "DEBUG:: P after propagate" << endl << P << endl;

    t_prev = cur_t;
}

// Loosely coupled update in world frame, fusing the global position of UWB and angle
void update_loosely(const uwb_msgs::uwb &msg) {
    VectorXd T(6);
    T(0) = 0;
    T(1) = 0;
    T(2) = msg.pos_theta - theta_bias;
    T(3) = msg.pos_x;
    T(4) = msg.pos_y;
    T(5) = 0;

    MatrixXd C = MatrixXd::Zero(6, 15);
    C.block<3, 3>(0, 0) = Matrix3d::Identity();
    C.block<3, 3>(3, 3) = Matrix3d::Identity();

    MatrixXd K(15, 6);
    K = P * C.transpose() * (C * P * C.transpose() + R).inverse();
    cout << "DEBUG:: update K" << endl << K << endl;

    Matrix3d uwb_R_world = AngleAxisd(T(2), Vector3d::UnitZ()) * imu_R_world;
    cout << "DEBUG:: measured angle" << endl << uwb_R_world << endl;

    VectorXd r(6);
    Quaterniond qm(uwb_R_world);
    Quaterniond q = Quaterniond(x(0), x(1), x(2), x(3));
    Quaterniond dq = q.conjugate() * qm; // Hamilton style
//    cout << "DEBUG:: dq" << endl << dq << endl;
    r.head(3) = 2 * dq.vec();
    r.tail(3) = T.tail(3) - x.segment<3>(4);
    VectorXd _r = K * r;
    Vector3d dw(0.5 * _r(0), 0.5 * _r(1), 0.5 * _r(2));
    Quaterniond _dq = Quaterniond(1, dw(0), dw(1), dw(2)).normalized();
    q = q * _dq;

//    // remove unobservable quaternion
//    VectorXd x_ori(15);
//    x_ori.segment<12>(3) = x.segment<12>(4);
//    x_ori = x_ori + K * (T - C * x_ori);
//    cout << "DEBUG:: x 15 states after update" << endl << x_ori << endl;
    x(0) = q.w();
    x(1) = q.x();
    x(2) = q.y();
    x(3) = q.z();
    x.segment<12>(4) += _r.segment<12>(3);
    P = P - K * C * P;
    cout << "DEBUG:: P after update" << endl << P << endl;
}

/**
 * Initialize, handle and save imu messages
 * @param imu_msg
 */
void imu_callback(const sensor_msgs::Imu::ConstPtr &imu_msg) {
    if (!imu_initialized && imu_count < IMU_INIT_COUNT) {
        accl_init_buf[imu_count](0) = imu_msg->linear_acceleration.x;
        accl_init_buf[imu_count](1) = imu_msg->linear_acceleration.y;
        accl_init_buf[imu_count](2) = imu_msg->linear_acceleration.z;
        gyro_init_buf[imu_count](0) = imu_msg->angular_velocity.x;
        gyro_init_buf[imu_count](1) = imu_msg->angular_velocity.y;
        gyro_init_buf[imu_count](2) = imu_msg->angular_velocity.z;
        g_init(0) += accl_init_buf[imu_count](0);
        g_init(1) += accl_init_buf[imu_count](1);
        g_init(2) += accl_init_buf[imu_count](2);
        imu_count++;
    } else if (!imu_initialized && imu_count == IMU_INIT_COUNT) {
#ifdef INIT_Q_R_BY_MEASUREMENT
        double accl_mean[3] = {0};
        double gyro_mean[3] = {0};
        double accl_cova[3] = {0};
        double gyro_cova[3] = {0};
        for (int i = 0; i < IMU_INIT_COUNT; ++i) {
            for (int j = 0; j < 3; ++j) {
                accl_mean[j] += accl_init_buf[i](j);
                gyro_mean[j] += gyro_init_buf[i](j);
                accl_cova[j] += pow(accl_init_buf[i](j), 2);
                gyro_cova[j] += pow(gyro_init_buf[i](j), 2);
            }
        }
        for (int j = 0; j < 3; ++j) {
            accl_mean[j] /= IMU_INIT_COUNT;
            gyro_mean[j] /= IMU_INIT_COUNT;
            accl_cova[j] /= IMU_INIT_COUNT;
            gyro_cova[j] /= IMU_INIT_COUNT;

            // Q omg first, Q acceleration second
            Q(j, j) = gyro_cova[j] - pow(gyro_mean[j], 2);
            Q(j + 3, j + 3) = accl_cova[j] - pow(accl_mean[j], 2);
        }

        // Hardcoded initial IMU bias_a, bias_g
        Q.bottomRightCorner(6, 6) = 0.01 * Q.bottomRightCorner(6, 6);
        cout << "DEBUG: measured Q" << endl << Q << endl;
#else
        // Initialize the covariance
        Q.topLeftCorner(6, 6) = 0.001 * Q.topLeftCorner(6, 6);     // IMU omg, accel
        Q.bottomRightCorner(6, 6) = 0.001 * Q.bottomRightCorner(6, 6); // IMU bias_a, bias_g
#endif
        g_init /= IMU_INIT_COUNT;

        // Initialize the state and the gravity
        x.setZero();
        Quaterniond init_pose(init_robot_pose);
        x(0) = init_pose.w();
        x(1) = init_pose.x();
        x(2) = init_pose.y();
        x(3) = init_pose.z();

        G = imu_R_world * g_init;
        cout << "DEBUG: measured G" << endl << G << endl;
        t_prev = imu_msg->header.stamp.toSec();
        imu_initialized = true;
    } else if (imu_initialized && odom_initialized) {
        propagate(imu_msg);
        pub_odom_ekf(imu_msg->header);

        imu_buf.push(imu_msg);
        x_history.push(x);
        P_history.push(P);
    }
}

/**
 * initialize, handle and save uwb messages
 * Also doing time synchronization
 * @param uwb msg
 */
void odom_callback(const uwb_msgs::uwb &msg) {
    if (!imu_initialized && !odom_initialized) {
        uwb_init_buf_x.push(msg.pos_x);
        uwb_init_buf_y.push(msg.pos_y);
        uwb_init_buf_w.push(msg.pos_theta);
    } else if (imu_initialized && !odom_initialized) {
#ifdef INIT_Q_R_BY_MEASUREMENT
        double uwb_mean[3] = {0, 0, 0};
        double uwb_cova[3] = {0, 0, 0};
        int uwb_count = (int) uwb_init_buf_x.size();
        for (int i = 0; i < uwb_count; ++i) {
            uwb_mean[0] += uwb_init_buf_x.front();
            uwb_mean[1] += uwb_init_buf_y.front();
            uwb_mean[2] += uwb_init_buf_w.front();
            uwb_cova[0] += pow(uwb_init_buf_x.front(), 2);
            uwb_cova[1] += pow(uwb_init_buf_y.front(), 2);
            uwb_cova[2] += pow(uwb_init_buf_w.front(), 2);
            uwb_init_buf_x.pop();
            uwb_init_buf_y.pop();
            uwb_init_buf_w.pop();
        }
        uwb_mean[0] /= uwb_count;
        uwb_mean[1] /= uwb_count;
        uwb_mean[2] /= uwb_count;
        uwb_cova[0] /= uwb_count;
        uwb_cova[1] /= uwb_count;
        uwb_cova[2] /= uwb_count;

        // Initialize the position and bias
        x(4) = uwb_mean[0];
        x(5) = uwb_mean[1];
        x(6) = 0;
        theta_bias = uwb_mean[2];

        // Initialize the covariance R
        R.topLeftCorner(2, 2) = 0.01 * R.topLeftCorner(2, 2);
        R(2, 2) = uwb_cova[2] - pow(uwb_mean[2], 2);
        R(3, 3) = uwb_cova[0] - pow(uwb_mean[0], 2);
        R(4, 4) = uwb_cova[1] - pow(uwb_mean[1], 2);
        R(5, 5) = 0.01 * R(5, 5);
        cout << "DEBUG: measured R" << endl << R << endl;
#else
        x(4) = msg.pos_x;
        x(5) = msg.pos_y;
        x(6) = 0;
        theta_bias = msg.pos_theta;
        R.topLeftCorner(3, 3) = 0.01 * R.topLeftCorner(3, 3);
        R.bottomRightCorner(3, 3) = 0.01 * R.bottomRightCorner(3, 3); // Measure x, y
#endif
        t_prev = msg.header.stamp.toSec();
        odom_initialized = true;
    } else {
        // throw the state and covariance history before the uwb time
        while (!imu_buf.empty() && imu_buf.front()->header.stamp < msg.header.stamp) {
            // trace the time backwards to imu time
            t_prev = imu_buf.front()->header.stamp.toSec();
            ROS_INFO("throw state with time: %f", t_prev);
            imu_buf.pop();
            x_history.pop();
            P_history.pop();
        }
        // If x_history is empty then the uwb reading is the same as the last imu reading
        // And the current estimated x could be used.
        // If not, use the oldest time in the x_history
        if (!x_history.empty()) {
            x = x_history.front();
            P = P_history.front();
            t_prev = imu_buf.front()->header.stamp.toSec();
            imu_buf.pop();
            x_history.pop();
            P_history.pop();
        }

        ROS_INFO("update state with time: %f", msg.header.stamp.toSec());
        update_loosely(msg);

        // clean the x and P history since the new update corrects the previous propagate
        while (!x_history.empty()) x_history.pop();
        while (!P_history.empty()) P_history.pop();

        queue <sensor_msgs::Imu::ConstPtr> temp_imu_buf;
        while (!imu_buf.empty()) {
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

int main(int argc, char **argv) {
    ros::init(argc, argv, "ekf_quaternion_16states");
    ros::NodeHandle n("~");

    // sleep for 10 seconds in order to launch both sensors
    ros::Duration(10).sleep();

    n.param("imu_topic", imu_topic, string("/dji_sdk/imu"));
    n.param("uwb_topic", uwb_topic, string("/uwb_driver/info"));
    n.param("publisher_topic", publisher_topic, string("/ekf_odom"));

    ros::Subscriber s1 = n.subscribe(imu_topic, 100, imu_callback);
    ros::Subscriber s2 = n.subscribe(uwb_topic, 10, odom_callback);
    odom_pub = n.advertise<nav_msgs::Odometry>(publisher_topic, 100);

    // Running the odometry in 400Hz as the IMU update
    ros::Rate r(400);

//    Rimu = Quaterniond(0.7071, 0, 0, -0.7071).toRotationMatrix();
    cout << "imu_R_world" << endl << imu_R_world << endl;

    ros::spin();
}

/**
 *  0   q_w     body frame --> world frame
 *  1   q_x
 *  2   q_y
 *  3   q_z
 *  4   p_x     world frame
 *  5   p_y
 *  6   p_z
 *  7   v_x     world frame
 *  8   v_y
 *  9   v_z
 *  10  ba_x    accel_bias      body frame
 *  11  ba_y
 *  12  ba_z
 *  13  bw_x    gyro_bias       body frame
 *  14  bw_y
 *  15  bw_z
 */
