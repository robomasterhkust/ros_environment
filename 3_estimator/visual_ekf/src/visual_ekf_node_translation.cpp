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
//#include <ros/console.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <queue>
//#include "rm_cv/ArmorRecord.h"

using namespace std;
using namespace Eigen;
ros::Publisher pose_pub, debug_pub, debug_gyro_pub, odom_pub;
string imu_topic, pose_topic, visual_topic, publisher_topic;
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

// previous propagated time
double t_prev;

// Initialization
const int IMU_INIT_COUNT = 10;
const int MAX_GYRO_QUEUE_SIZE = 2000;
int imu_count  = 0;
bool imu_initialized = false;
bool visual_initialized = false;
bool visual_valid = false;
MatrixXd imu_R_camera = MatrixXd::Identity(3, 3); // rotation matrix from camera to imu
Vector3d imu_T_camera = MatrixXd::Zero(3, 1);

void pub_fused_pose(std_msgs::Header header)
{
    geometry_msgs::PoseWithCovarianceStamped pose;
    pose.header = header;
    pose.pose.pose.orientation.w = x(0);
    pose.pose.pose.orientation.x = x(1);
    pose.pose.pose.orientation.y = x(2);
    pose.pose.pose.orientation.z = x(3);
    pose.pose.covariance[0] = P(0, 0);
    pose.pose.covariance[1] = P(1, 1);
    pose.pose.covariance[2] = P(2, 2);

    pose_pub.publish(pose);
}

void pub_debug_pose(std_msgs::Header header, const Quaterniond& q)
{
    geometry_msgs::PoseStamped pose;
    pose.header = header;
    pose.pose.orientation.w = q.w();
    pose.pose.orientation.x = q.x();
    pose.pose.orientation.y = q.y();
    pose.pose.orientation.z = q.z();
    debug_pub.publish(pose);
}

void pub_fused_angleAxis(std_msgs::Header header)
{
    geometry_msgs::PoseWithCovarianceStamped angleAxis;
    angleAxis.header = header;

    Quaterniond pose_state(x(0), x(1), x(2), x(3));
    AngleAxisd angle_state(pose_state);

    angleAxis.pose.pose.orientation.w = angle_state.angle();
    angleAxis.pose.pose.orientation.x = angle_state.axis()[0];
    angleAxis.pose.pose.orientation.y = angle_state.axis()[1];
    angleAxis.pose.pose.orientation.z = angle_state.axis()[2];
    angleAxis.pose.covariance[0] = P(0, 0);
    angleAxis.pose.covariance[1] = P(1, 1);
    angleAxis.pose.covariance[2] = P(2, 2);
    pose_pub.publish(angleAxis);
}

void pub_debug_angleAxis(std_msgs::Header header, const Eigen::Vector3d axis, const double angle)
{
    geometry_msgs::PoseStamped angleAxis;
    angleAxis.header = header;
    angleAxis.pose.orientation.w = angle;
    angleAxis.pose.orientation.x = axis.x();
    angleAxis.pose.orientation.y = axis.y();
    angleAxis.pose.orientation.z = axis.z();
    debug_pub.publish(angleAxis);
}

void pub_shield_odom(const std_msgs::Header &header)
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

    odom_pub.publish(odom);
}

void propagate(const sensor_msgs::Imu &imu)
{
    double cur_t = imu.header.stamp.toSec();
    // ROS_INFO("propagate");
    Vector3d w;
    Vector3d a;
    w(0) = imu.angular_velocity.x;
    w(1) = imu.angular_velocity.y;
    w(2) = imu.angular_velocity.z;
    a(0) = imu.linear_acceleration.x;
    a(1) = imu.linear_acceleration.y;
    a(2) = imu.linear_acceleration.z;
    Quaterniond world_R_imu(imu.orientation.w, imu.orientation.x, imu.orientation.y, imu.orientation.z);

    double dt = cur_t - t_prev;
    ROS_INFO("dt in propagate is %f, at the cur_t %f", dt, cur_t);
	// ROS_INFO("propagate, with dt %f", dt);

    x.segment<3>(0) += x.segment<3>(3) * dt + 0.5 * (world_R_imu * (a - G)) * dt * dt;
    x.segment<3>(3) += (world_R_imu * (a - G)) * dt;

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
    // Repropagate
    while (!x_history.empty()) x_history.pop();
    while (!P_history.empty()) P_history.pop();

    queue <sensor_msgs::Imu::ConstPtr> temp_imu_buf;
    while (!imu_buf.empty())
    {
        propagate(*imu_buf.front());
        temp_imu_buf.push(imu_buf.front());
        x_history.push(x);
        P_history.push(P);
        imu_buf.pop();
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

    throwState(cur_t);

    ROS_INFO("Update, at time %f", cur_t);
    Quaterniond world_R_imu(imu_buf.front()->orientation.w,
                            imu_buf.front()->orientation.x,
                            imu_buf.front()->orientation.y,
                            imu_buf.front()->orientation.z);
    Vector3d world_T_shield = world_R_imu * imu_T_shield;

//    VectorXd z(6);
//    z.setZero();
//    z.segment<3>(0) = world_T_shield;
//    cout << "z " << endl << z << endl;

    MatrixXd C = MatrixXd::Zero(3, 6);
    C.block<3, 3>(0, 0) = MatrixXd::Identity(3, 3);
    cout << "C " << endl << C << endl;

    MatrixXd K(6, 3);
    K = P * C.transpose() * (C * P* C.transpose() + Q).inverse();
    cout << "K " << endl << K << endl;

    x = x + K * (world_T_shield - C * x);
    P = P - K * C * P;
    cout << "P " << endl << P << endl;
    cout << "x " << endl << x.transpose() << endl;

    repropagate();
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

    throwState(cur_t);

    Quaterniond world_R_imu(imu_buf.front()->orientation.w,
                            imu_buf.front()->orientation.x,
                            imu_buf.front()->orientation.y,
                            imu_buf.front()->orientation.z);
    Vector3d world_T_shield = world_R_imu * imu_T_shield;

    x.segment<3>(0) = world_T_shield;
    x.segment<3>(3) << 0, 0, 0;

    cout << "DEBUG: x initialized with " << endl << x.transpose() << endl;

    t_prev = cur_t;

    visual_initialized = true;

    repropagate();
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
        // pub_shield_odom(pnp->header);
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
    debug_pub= n.advertise<geometry_msgs::PoseStamped>(string("/visual_ekf/visual_ekf_debug"), 100);
//    debug_gyro_pub= n.advertise<geometry_msgs::Vector3Stamped>(string("/visual_ekf/gyro_debug"), 100);

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
