/**
 * Created by Beck on 18/6/2018
 * Indirect Extended Kalman Filter for 10 states
 * to get an estimate of the translation of the shield
 * fusing visual information with the imu
 * With IMU fused reading comes in 100Hz and camera translation in 30Hz
 */
#include <iostream>
#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Eigen>
#include <queue>
#include "rm_cv/ArmorRecord.h"

using namespace std;
using namespace Eigen;
ros::Publisher odom_pub, debug_pub;
string imu_topic, omg_topic, visual_topic, publisher_topic;
double acc_weight, gyro_weight;
double visual_q_weight, visual_t_weight;
int sleep_time;

/**
 * Define states:
 *      x = [rotation_quaternion, position, velocity]
 * Define inputs:
 *      u = [omg, acc], filtered angular velocity, raw acceleration
 * Define noises:
 *      n = [n_gyro, n_acc]
 */
VectorXd x(10);                        // state
MatrixXd P = MatrixXd::Zero(9, 9);     // covariance
MatrixXd Q = MatrixXd::Identity(6, 6); // prediction noise covariance
MatrixXd R = MatrixXd::Identity(6, 6); // observation noise covariance

// buffers to save imu and visual reading
queue<sensor_msgs::Imu::ConstPtr> imu_buf;
queue<geometry_msgs::TwistStamped::ConstPtr> visual_buf;
queue<Matrix<double, 10, 1>> x_history;
queue<Matrix<double, 9, 9>> P_history;

// previous propagated time
double t_prev;

// Initialization
// TODO: hand-eye calibration
Matrix3d imu_R_camera; // camera point in imu frame
Vector3d imu_T_camera;
const int IMU_INIT_COUNT = 40;
int imu_count  = 0;
Vector3d g_init = Vector3d::Zero();
Vector3d G = Vector3d::Zero();
Vector3d accl_init_buf[IMU_INIT_COUNT];
Vector3d gyro_init_buf[IMU_INIT_COUNT];
queue<Matrix<double, 3, 1>> pnp_T_buf;
queue<Matrix<double, 3, 1>> pnp_R_buf;
bool imu_initialized = false;
bool visual_initialized = false;

/**
 * publish the pose and velocity
 * @param header
 */
static void pub_fused_T_shield(std_msgs::Header header)
{
    nav_msgs::Odometry odom;
    odom.header.stamp = header.stamp;
    odom.header.frame_id = "camera";
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

/**
 * Extended Kalman Filter propagate, taking imu reading as actions
 * @param imu_msg
 */
static void propagate(const sensor_msgs::Imu::ConstPtr &imu_msg)
{
    double cur_t = imu_msg->header.stamp.toSec();
    double dt = cur_t - t_prev;
    Vector3d w_raw, a_raw;
    a_raw(0) = imu_msg->linear_acceleration.x;
    a_raw(1) = imu_msg->linear_acceleration.y;
    a_raw(2) = imu_msg->linear_acceleration.z;
    w_raw(0) = imu_msg->angular_velocity.x;
    w_raw(1) = imu_msg->angular_velocity.y;
    w_raw(2) = imu_msg->angular_velocity.z;

    // TODO: check bias and use filtered angular velocity
    Vector3d a   = a_raw;
    Vector3d omg = w_raw;
    Vector3d domg = 0.5 * dt * omg;

    // a better approximation for quaternion
    Quaterniond dR(sqrt(1 - domg.squaredNorm()), domg(0), domg(1), domg(2));
    Quaterniond R_old(x(0), x(1), x(2), x(3));
    Quaterniond R_new = (R_old * dR).normalized();

    x.segment<4>(0) << R_new.w(), R_new.x(), R_new.y(), R_new.z();
    x.segment<3>(4) += x.segment<3>(7) * dt + (R_old * a - G) * 0.5 * dt * dt;
    x.segment<3>(7) += (R_old * a - G) * dt;

    // propagate the covariance with skew-symmetric matrix
    Matrix3d I = MatrixXd::Identity(3, 3);
    Matrix3d w_omg, w_a;
    w_omg <<     0 , -omg(2),  omg(1),
             omg(2),      0 , -omg(0),
            -omg(1),  omg(0),      0;
    w_a   <<  0 , -a(2),  a(1),
            a(2),    0 , -a(0),
           -a(1),  a(0),    0;
    MatrixXd A = MatrixXd::Zero(9, 9);
    A.block<3, 3>(0, 0) = -w_omg;
    A.block<3, 3>(3, 6) = I;
    A.block<3, 3>(6, 0) = (-1 * R_old.toRotationMatrix()) * w_a;
    // cout << "DEBUG:: propagate A" << endl << A << endl;

    MatrixXd U = MatrixXd::Zero(9, 6);
    U.block<3, 3>(0, 0) = -1 * I;
    U.block<3, 3>(6, 3) = -1 * R_old.toRotationMatrix();
    // cout << "DEBUG:: propagate U" << endl << U << endl;

    MatrixXd F, V; // Discretize
    F = MatrixXd::Identity(9, 9) + dt * A;
    V = dt * U;

    P = F * P * F.transpose() + V * Q * V.transpose();

    t_prev = cur_t;
}

/**
 * Extended Kalman Filter update, assume the stable solution from pnp
 * @param pnp
 */
static void update(const geometry_msgs::TwistStamped::ConstPtr &pnp)
{
    Matrix3d camera_R_shield;
    Vector3d camera_T_shield;

    camera_T_shield[0] = pnp->twist.linear.x;
    camera_T_shield[1] = pnp->twist.linear.y;
    camera_T_shield[2] = pnp->twist.linear.z;
    Vector3d Rodrigues(pnp->twist.angular.x,
                       pnp->twist.angular.y,
                       pnp->twist.angular.z);
    camera_R_shield = AngleAxisd(Rodrigues.norm(), Rodrigues.normalized())
            .toRotationMatrix();
    Matrix3d shield_R_imu =  camera_R_shield.transpose() *  imu_R_camera.transpose();
    Vector3d shield_T_imu = -camera_R_shield.transpose() * (imu_R_camera.transpose() * imu_T_camera + camera_T_shield);
    Quaterniond shield_q_imu(shield_R_imu);

    geometry_msgs::PoseStamped debug;
    debug.header.stamp = pnp->header.stamp;
    debug.header.frame_id = "shield";
    debug.pose.position.x = shield_T_imu(0);
    debug.pose.position.y = shield_T_imu(1);
    debug.pose.position.z = shield_T_imu(2);
    debug.pose.orientation.w = shield_q_imu.w();
    debug.pose.orientation.x = shield_q_imu.x();
    debug.pose.orientation.y = shield_q_imu.y();
    debug.pose.orientation.z = shield_q_imu.z();
    debug_pub.publish(debug);

    MatrixXd C = MatrixXd::Zero(6, 9);
    C.block<3, 3>(0, 0) = Matrix3d::Identity();
    C.block<3, 3>(3, 3) = Matrix3d::Identity();

    MatrixXd K(9, 6);
    K = P * C.transpose() * (C * P* C.transpose() + R).inverse();

    VectorXd r(6); // residual
    Vector3d T = shield_T_imu;
    Quaterniond qm = shield_q_imu;
    Quaterniond q  = Quaterniond(x(0), x(1), x(2), x(3));
    Quaterniond dq = q.conjugate() * qm;
    r.head<3>() = 2 * dq.vec();
    r.tail<3>() = T - x.segment<3>(4);
    VectorXd _r = K * r;
    Vector3d dw(_r(0) * 0.5, _r(1) * 0.5, _r(2) * 0.5);
    //    dq = Quaterniond(1, dw(0), dw(1), dw(2)).normalized();
    dq = Quaterniond(sqrt(1 - dw.squaredNorm()), dw(0), dw(1), dw(2)).normalized();
    q  = q * dq;

    x(0) = q.w();
    x(1) = q.x();
    x(2) = q.y();
    x(3) = q.z();

    x.segment<6>(4) += _r.tail(6);
    P = P - K * C * P;
}

/**
 * initialization of the imu
 * @param imu_msg
 */
static void initialize_imu(const sensor_msgs::Imu::ConstPtr &imu_msg)
{
    if (imu_count < IMU_INIT_COUNT)
    {
        Vector3d a_raw;
        a_raw(0) = imu_msg->linear_acceleration.x;
        a_raw(1) = imu_msg->linear_acceleration.y;
        a_raw(2) = imu_msg->linear_acceleration.z;
        imu_count++;
        g_init += a_raw;
    }
    else if (imu_count == IMU_INIT_COUNT)
    {
        g_init /= IMU_INIT_COUNT;
        imu_initialized = true;
    }
}

/**
 * initialization of the state and convariance from visual
 * @param pnp
 */
static void initialize_visual(const geometry_msgs::TwistStamped::ConstPtr &pnp)
{
    double cur_t = pnp->header.stamp.toSec();
    Matrix3d camera_R_shield;
    Vector3d camera_T_shield;

    camera_T_shield[0] = pnp->twist.linear.x;
    camera_T_shield[1] = pnp->twist.linear.y;
    camera_T_shield[2] = pnp->twist.linear.z;
    Vector3d Rodrigues(pnp->twist.angular.x,
                       pnp->twist.angular.y,
                       pnp->twist.angular.z);
    camera_R_shield = AngleAxisd(Rodrigues.norm(), Rodrigues.normalized())
            .toRotationMatrix();
    cout << "DEBUG: camera_R_shield " << endl << camera_R_shield << endl;
    Matrix3d shield_R_imu =  camera_R_shield.transpose() *  imu_R_camera.transpose();
    Vector3d shield_T_imu = -camera_R_shield.transpose() * (imu_R_camera.transpose() * imu_T_camera + camera_T_shield);
    cout << "DEBUG: shield_R_imu " << endl << shield_R_imu << endl;
    cout << "DEBUG: shield_T_imu " << endl << shield_T_imu << endl;
    Quaterniond shield_q_imu(shield_R_imu);

    x.setZero();
    P.setZero();

    x.head<4>() << shield_q_imu.w(), shield_q_imu.x(), shield_q_imu.y(), shield_q_imu.z();
    x.segment<3>(4) << shield_T_imu;

    t_prev = cur_t;
    if (imu_initialized) {
        G = shield_R_imu * g_init;
        cout << "gravity vector in shield frame " << endl << G.transpose() << endl;
        visual_initialized = true;
    }
}

/**
 * handle, save, and process imu messages
 * @param imu_msg
 */
void imu_callback(const sensor_msgs::Imu::ConstPtr &imu_msg)
{
    if (!imu_initialized && imu_count <= IMU_INIT_COUNT) {
        initialize_imu(imu_msg);
    }
    else if (imu_initialized && visual_initialized) {
        propagate(imu_msg);

        x_history.push(x);
        P_history.push(P);

        // Only publish the topic in imu callback
        if (visual_initialized)
            pub_fused_T_shield(imu_msg->header);
    }
    imu_buf.push(imu_msg);
}

/**
 * handle, save, and process visual messages
 * @param pnp
 */
void visual_callback(const geometry_msgs::TwistStamped::ConstPtr &pnp)
//void visual_callback(const rm_cv::ArmorRecord &pnp)
{
    if (pnp->twist.linear.x != 0) {
        if (!visual_initialized) {
            initialize_visual(pnp);
        }
        visual_buf.push(pnp);
    }
}

static void state_machine_process(void) {
    if (!imu_initialized || !visual_initialized)
        return;

    if (!visual_buf.empty()) {
        while(!imu_buf.empty() &&
              imu_buf.front()->header.stamp < visual_buf.front()->header.stamp)
        {
            // trace backward the time to the imu timestamp
            t_prev = imu_buf.front()->header.stamp.toSec();
            // ROS_INFO("throw state with time: %f", t_prev);
            imu_buf.pop();
            x_history.pop();
            P_history.pop();
        }

        // If x_history is empty then the visual reading
        // is the same as the last imu reading
        // and the current estimated x could be used
        // If not, use the oldest time in the x_history
        if (!x_history.empty()) {
            x = x_history.front();
            P = P_history.front();
            t_prev = imu_buf.front()->header.stamp.toSec();
            imu_buf.pop();
            x_history.pop();
            P_history.pop();
        }

        geometry_msgs::TwistStamped::ConstPtr visual_msg = visual_buf.front();
        if (visual_msg->twist.linear.x == 0)
        {
            visual_initialized = false;
        }
        else {
            update(visual_msg);
        }

        visual_buf.pop();

        while (!x_history.empty()) x_history.pop();
        while (!P_history.empty()) P_history.pop();

        queue <sensor_msgs::Imu::ConstPtr> temp_imu_buf;
        while (!imu_buf.empty())
        {
            propagate(imu_buf.front());
            temp_imu_buf.push(imu_buf.front());
            x_history.push(x);
            P_history.push(P);
            imu_buf.pop();
        }
        swap(imu_buf, temp_imu_buf);
    }


}

int main(int argc, char **argv) {
    ros::init(argc, argv, "visual_ekf_no_bias");
    ros::NodeHandle n("~");

    n.param("imu_topic", imu_topic, string("/dji_sdk/imu")); // 400Hz
    n.param("angular_fused", omg_topic, string("/dji_sdk/angular_velocity_fused")); // 100Hz
    n.param("visual_topic", visual_topic, string("/pnp_twist"));
//    n.param("visual_topic", visual_topic, string("/gimbal_detected_armor"));
    n.param("publisher_topic", publisher_topic, string("/T_shield_fused"));
    n.param("gyroscope_noise_weight", gyro_weight, 0.1);
    n.param("accelerometer_noise_weight", acc_weight, 0.1);
    n.param("visual_orientation_weight", visual_q_weight, 0.1);
    n.param("visual_translation_weight", visual_t_weight, 0.1);
    n.param("node_sleep_time", sleep_time, 10);

    // sleep for 10 seconds to wait for both sensors
    // ros::Duration(sleep_time).sleep();

    ros::Subscriber s1 = n.subscribe(imu_topic, 100, imu_callback);
    ros::Subscriber s2 = n.subscribe(visual_topic, 10, visual_callback);
    odom_pub = n.advertise<nav_msgs::Odometry>(publisher_topic, 100);
    debug_pub= n.advertise<geometry_msgs::PoseStamped>(string("/visual_ekf_debug"), 100);

    // TODO: initalize the Q and R matrix
    Q.topLeftCorner(3, 3)     = gyro_weight * MatrixXd::Identity(3, 3); // gyro noise
    Q.bottomRightCorner(3, 3) = acc_weight  * MatrixXd::Identity(3, 3); // accl noise
    R.topLeftCorner(3, 3)     = visual_q_weight * MatrixXd::Identity(3, 3); // measurement orientation
    R.bottomRightCorner(3, 3) = visual_t_weight * MatrixXd::Identity(3, 3); // measurement position

    // TODO: check rotation matrix
    imu_R_camera <<  0, 0, 1,
                    -1, 0, 0,
                     0,-1, 0;

    imu_T_camera << 160, 240, -40; // in millimeter

    cout << "imu_R_camera" << endl << imu_R_camera << endl;

    ros::Rate r(100);
    while (ros::ok())
    {
        ros::spinOnce();
        state_machine_process();
        r.sleep();
    }
}
/**
 *  0   q_w     shield frame --> camera frame
 *  1   q_x
 *  2   q_y
 *  3   q_z
 *  4   p_x     camera frame
 *  5   p_y
 *  6   p_z
 *  7   v_x     camera frame
 *  8   v_y
 *  9   v_z
 */
