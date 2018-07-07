/**
 * Created by Beck on 3/7/2018
 * Indirect Extended Kalman Filter for the translation of the camera
 * fusing visual information with the accelerometer, given the pose;
 * With imu in 400Hz and camera translation in 30Hz
 * V1: estimation for rotation only
 * V2: give the translation of the shield
 */
#include <iostream>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <queue>
//#include "rm_cv/ArmorRecord.h"

using namespace std;
using namespace Eigen;

ros::Publisher pose_pub, debug_pub, odom_pub, debug_propagate_pub;
string imu_topic, pose_topic, visual_topic;
string debug_topic, publisher_topic;
double acc_weight;
double visual_q_weight, visual_t_weight;
int sleep_time;

/**
 * Define states:
 *      x = [translation_x, y, z; velocity_x, y, z]
 * Define inputs:
 *      u = [acc], raw accelerometer
 * Define noises:
 *      n = [n_acc]
 */
VectorXd x(6);         // state
MatrixXd P = MatrixXd::Identity(6, 6); // covariance
MatrixXd R = MatrixXd::Identity(3, 3); // prediction noise covariance
MatrixXd Q = MatrixXd::Identity(3, 3); // observation noise covariance

// buffers to save gyro and visual reading
queue<sensor_msgs::Imu::ConstPtr> imu_buf;
queue<geometry_msgs::TwistStamped::ConstPtr> visual_buf;
queue<Matrix<double, 6, 1>> x_history;
queue<Matrix<double, 6, 6>> P_history;
Vector3d G = {0, 0, 9.8}; // Consider to add initialization later

double t_prev;          // previous propagated time

// Initialization
const int IMU_INIT_COUNT = 10;
const int MAX_GYRO_QUEUE_SIZE = 400;
const double IMU_UPDATE_TIME = 0.0025; // 1 / 400Hz
int imu_count  = 0;
bool imu_initialized = false;
bool visual_initialized = false;
bool visual_valid = false;
MatrixXd imu_R_camera = MatrixXd::Identity(3, 3); // rotation matrix from camera to imu
Vector3d imu_T_camera = MatrixXd::Zero(3, 1);

//// DEBUG only
double t_prev_update;   // previous update time
Vector3d world_T_shield_prev = MatrixXd::Zero(3, 1); // previous visual translation

void pub_shield_odom(const std_msgs::Header& header)
{
    nav_msgs::Odometry odom;
    odom.header = header;
    odom.child_frame_id = "world";
    odom.pose.pose.position.x = x(0);
    odom.pose.pose.position.y = x(1);
    odom.pose.pose.position.z = x(2);
    odom.twist.twist.linear.x = x(3);
    odom.twist.twist.linear.y = x(4);
    odom.twist.twist.linear.z = x(5);

    odom.pose.covariance[0]  = P(0, 0);
    odom.pose.covariance[7]  = P(1, 1);
    odom.pose.covariance[14] = P(2, 2);
    odom.pose.covariance[21] = P(3, 3);
    odom.pose.covariance[28] = P(4, 4);
    odom.pose.covariance[35] = P(5, 5);
    odom.pose.covariance[3]  = P(0, 3);
    odom.pose.covariance[10] = P(1, 4);
    odom.pose.covariance[17] = P(2, 5);
    odom.pose.covariance[18] = P(3, 0);
    odom.pose.covariance[25] = P(4, 1);
    odom.pose.covariance[32] = P(5, 2);
    odom_pub.publish(odom);
}

void pub_debug_update(const std_msgs::Header& header,
                      const Ref<const Vector3d> p,
                      const Quaterniond& q,
                      double t_update)
{
    nav_msgs::Odometry odom;
    odom.header = header;
    odom.child_frame_id = "world";
    odom.pose.pose.position.x = p[0];
    odom.pose.pose.position.y = p[1];
    odom.pose.pose.position.z = p[2];
    odom.pose.pose.orientation.w = q.w();
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    if (visual_initialized) {
        double dt_update = t_update - t_prev_update;
        odom.twist.twist.linear.x = (p[0] - world_T_shield_prev[0]) / dt_update;
        odom.twist.twist.linear.y = (p[1] - world_T_shield_prev[1]) / dt_update;
        odom.twist.twist.linear.z = (p[2] - world_T_shield_prev[2]) / dt_update;

        world_T_shield_prev = p;
        t_prev_update = t_update;
    }
    else {
        odom.twist.twist.linear.x = 0;
        odom.twist.twist.linear.y = 0;
        odom.twist.twist.linear.z = 0;
        world_T_shield_prev = p;
        t_prev_update = t_update;
    }

    debug_pub.publish(odom);
}

void pub_debug_propagate(const std_msgs::Header& header,
                         const Ref<const Vector3d> v,
                         const Ref<const Vector3d> a_int)
{
    geometry_msgs::TwistStamped acc_compare;
    acc_compare.header = header;
    acc_compare.twist.linear.x = v[0];
    acc_compare.twist.linear.y = v[1];
    acc_compare.twist.linear.z = v[2];
    acc_compare.twist.angular.x= a_int[0];
    acc_compare.twist.angular.y= a_int[1];
    acc_compare.twist.angular.z= a_int[2];
    debug_propagate_pub.publish(acc_compare);
}

// DEBUG only
Vector3d a_int = MatrixXd::Zero(3, 1);
void propagate(const sensor_msgs::Imu &imu)
{
    double cur_t = imu.header.stamp.toSec();
    Vector3d a, acc_wo_g;
    a(0) = imu.linear_acceleration.x;
    a(1) = imu.linear_acceleration.y;
    a(2) = imu.linear_acceleration.z;
    Quaterniond world_R_imu(imu.orientation.w, imu.orientation.x, imu.orientation.y, imu.orientation.z);

    double dt = cur_t - t_prev;
    dt = (dt < IMU_UPDATE_TIME * 5) ? dt : IMU_UPDATE_TIME * 5;
    ROS_INFO("dt in propagate is %f, at the cur_t %f", dt, cur_t);
    acc_wo_g = world_R_imu.toRotationMatrix() * a - G;
    x.segment<3>(0) += x.segment<3>(3) * dt + 0.5 * acc_wo_g * dt * dt;
    x.segment<3>(3) += acc_wo_g * dt;
    a_int += acc_wo_g * dt;
    pub_debug_propagate(imu.header, acc_wo_g, a_int);

    MatrixXd A = MatrixXd::Zero(6, 6);
    A.block<3, 3>(0, 3) = MatrixXd::Identity(3, 3);

    MatrixXd U = -MatrixXd::Zero(6, 3);
    U.block<3, 3>(3, 0) = -world_R_imu.toRotationMatrix();

    MatrixXd F, V;
    F = MatrixXd::Identity(6, 6) + dt * A;
    V = dt * U;
//    cout << "F " << endl << F << endl;
//    cout << "R " << endl << R << endl;
//    cout << "V " << endl << V << endl;
    P = F * P * F.transpose() + V * R * V.transpose();
	
    t_prev = cur_t;
//    cout << "P " << endl << P << endl;
//    cout << "x " << endl << x.transpose() << endl;
}

static void throwState(double current_time)
{
    while(imu_buf.size() > 1 &&
          imu_buf.front()->header.stamp.toSec() < current_time)
    {
        // trace backward the time to the imu timestamp
        t_prev = imu_buf.front()->header.stamp.toSec();
        ROS_INFO("throw state with time: %f", t_prev);
        imu_buf.pop();
        x_history.pop();
        P_history.pop();
    }
}

static void repropagate()
{
    while (!x_history.empty()) x_history.pop();
    while (!P_history.empty()) P_history.pop();

    queue <sensor_msgs::Imu::ConstPtr> temp_imu_buf;
    while (!imu_buf.empty() && imu_buf.size() > 1)
    {
        propagate(*imu_buf.front());
        temp_imu_buf.push(imu_buf.front());
        x_history.push(x);
        P_history.push(P);
        imu_buf.pop();
    }
    if (!temp_imu_buf.empty()) {
        pub_shield_odom(temp_imu_buf.back()->header);
    }
    swap(imu_buf, temp_imu_buf);
}

static void update(const geometry_msgs::TwistStamped &pnp)
{
    double cur_t = pnp.header.stamp.toSec();
    Vector3d camera_T_shield;

    camera_T_shield[0] = pnp.twist.linear.x;
    camera_T_shield[1] = pnp.twist.linear.y;
    camera_T_shield[2] = pnp.twist.linear.z;
    Vector3d imu_T_shield = imu_R_camera * camera_T_shield + imu_T_camera;

    imu_T_shield *= 0.001; // Convert millimeter to meter

    throwState(cur_t);

    ROS_INFO("Update, at time %f", cur_t);
    Quaterniond world_R_imu;

    if (imu_buf.empty()) {
        world_R_imu.setIdentity();
    }
    else {
        world_R_imu = Quaterniond(imu_buf.front()->orientation.w,
                                  imu_buf.front()->orientation.x,
                                  imu_buf.front()->orientation.y,
                                  imu_buf.front()->orientation.z);
    }
    Vector3d world_T_shield = world_R_imu.toRotationMatrix() * imu_T_shield;
    pub_debug_update(pnp.header, world_T_shield, world_R_imu, cur_t);

    MatrixXd C = MatrixXd::Zero(3, 6);
    C.block<3, 3>(0, 0) = MatrixXd::Identity(3, 3);

    Matrix3d W = MatrixXd::Identity(3, 3);

    MatrixXd K(6, 3);
    K = P * C.transpose() * (C * P * C.transpose() + W * Q * W.transpose()).inverse();
//    cout << "C " << endl << C << endl;
    cout << "K " << endl << K << endl;
//    cout << "Q " << endl << Q << endl;
    x = x + K * (world_T_shield - C * x);
    P = P - K * C * P;
//    cout << "P " << endl << P << endl;
//    cout << "x " << endl << x.transpose() << endl;

    if (imu_buf.size() > 1) {
        // repropagate();
    }
}

/**
 * initialize the imu messages
 * @param imu
 */
static void initialize_imu(const sensor_msgs::Imu::ConstPtr &imu)
{
    t_prev = imu->header.stamp.toSec();
    x_history.push(x);
    P_history.push(P);
    imu_buf.push(imu);
    imu_count++;
    if (imu_count == IMU_INIT_COUNT) {
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
    Vector3d camera_T_shield;
    ROS_INFO("visual init at %f", cur_t);

    camera_T_shield[0] = pnp->twist.linear.x;
    camera_T_shield[1] = pnp->twist.linear.y;
    camera_T_shield[2] = pnp->twist.linear.z;
    Vector3d imu_T_shield = imu_R_camera * camera_T_shield + imu_T_camera;

    imu_T_shield *= 0.001; // Convert millimeter to meter

    throwState(cur_t);

    Quaterniond world_R_imu;

    if (imu_buf.empty()) {
        world_R_imu.setIdentity();
    }
    else {
        world_R_imu = Quaterniond(imu_buf.front()->orientation.w,
                                  imu_buf.front()->orientation.x,
                                  imu_buf.front()->orientation.y,
                                  imu_buf.front()->orientation.z);
    }
    Vector3d world_T_shield = world_R_imu.toRotationMatrix() * imu_T_shield;
    pub_debug_update(pnp->header, world_T_shield, world_R_imu, cur_t);

    x.setZero();
    x.segment<3>(0) = world_T_shield;
    x.segment<3>(3) << 0, 0, 0;

    cout << "DEBUG: x initialized with " << endl << x.transpose() << endl;

    visual_initialized = true;

    // repropagate();
}

/**
 * handle, save, and process visual messages
 * @param pnp
 */
void visual_callback(const geometry_msgs::TwistStamped::ConstPtr &pnp)
{
    visual_valid = !(pnp->twist.linear.x == 0 &&
        pnp->twist.linear.y == 0 &&
        pnp->twist.linear.z == 0 );

    if (visual_valid) {
        if (visual_initialized) {
            update(*pnp);
        }
        else {
            initialize_visual(pnp);
        }
    }
}

/**
 * handle 400Hz raw imu data
 * @param imu
 */
void imu_callback(const sensor_msgs::Imu::ConstPtr &imu)
{
    if (!imu_initialized) {
        initialize_imu(imu);
    }
    else {
        propagate(*imu);
        x_history.push(x);
        P_history.push(P);
        imu_buf.push(imu);
		if (imu_buf.size() > MAX_GYRO_QUEUE_SIZE) {
			x_history.pop();
			P_history.pop();
			imu_buf.pop();
		}
        pub_shield_odom(imu->header);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "visual_gyro_fused");
    ros::NodeHandle n("~");

//    n.param("imu_raw", imu_topic, string("/dji_sdk/imu")); // 400Hz
    n.param("imu_pose", pose_topic, string("/attitude_estimator/imu")); // 400Hz
    n.param("visual_topic", visual_topic, string("/pnp_twist"));
    n.param("publisher_topic", publisher_topic, string("/visual_ekf/shield_T_world"));
    n.param("debug_topic", debug_topic, string("/visual_ekf/debug_odom"));
    n.param("accelerometer_noise_weight", acc_weight, 1000.0);
    n.param("visual_pose_weight", visual_q_weight, 10.0);
    n.param("node_sleep_time", sleep_time, 0);

    ros::Duration(sleep_time).sleep();

    // TODO: initalize the R and Q matrix
    R =      acc_weight * MatrixXd::Identity(3, 3); // accelerometer noise
    Q = visual_q_weight * MatrixXd::Identity(3, 3); // observation noise

    x.setZero();

    imu_R_camera <<  0, 0, 1,
                    -1, 0, 0,
                     0,-1, 0;

    imu_T_camera <<  200, 50, 0; // in millimeter

    ros::Subscriber s2 = n.subscribe(visual_topic, 10, visual_callback);
    ros::Subscriber s3 = n.subscribe(pose_topic, 100, imu_callback);
//    pose_pub = n.advertise<geometry_msgs::PoseStamped>(publisher_topic, 100);
    odom_pub = n.advertise<nav_msgs::Odometry>(publisher_topic, 100);
    debug_pub= n.advertise<nav_msgs::Odometry>(debug_topic, 100);
    debug_propagate_pub = n.advertise<geometry_msgs::TwistStamped>(string("/visual_ekf/propagate"), 100);

    ros::Rate r(100);
    ros::spin();
}

/**
 *  0   T_x     translation of the shield in world frame
 *  1   T_y
 *  2   T_z
 *  3   v_x     velocity of the shield in world frame
 *  4   v_y
 *  5   v_z
 */
