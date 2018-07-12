/**
 * July 9th, 2018
 * source: https://zhuanlan.zhihu.com/p/38745950
 * Constant velocity prediction Kalman filter test
 */

#include <iostream>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/TwistStamped.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include "rm_cv/ArmorRecord.h"

using namespace std;
using namespace Eigen;

string visual_topic, publisher_topic, debug_topic, real_visual_topic;
ros::Publisher filter_pub, debug_pub;
MatrixXd imu_R_camera = MatrixXd::Identity(3, 3); // rotation matrix from camera to imu
Vector3d imu_T_camera = MatrixXd::Zero(3, 1);

VectorXd x(6);      // state
MatrixXd P = MatrixXd::Identity(6, 6); // covariance
MatrixXd Q = MatrixXd::Identity(6, 6); // prediction noise covariance
MatrixXd R = MatrixXd::Identity(6, 6); // observation noise covariance
MatrixXd A = MatrixXd::Identity(6, 6); // state transfer function

bool visual_initialized = false, visual_valid = false;

ros::Time t_prev;
double pos_weight, vel_weight;
const double CV_UPDATE_TIME_MAX = 0.1; // maxium allowed update time
const int ROS_FREQ = 100;
int REPROPAGATE_TIME = 4; // repropagate after the update
int propagate_count = 0;
double OUTLIER_THRESHOLD = 10000.0;
Vector3d imu_T_shield_prev = MatrixXd::Zero(3, 1);
bool vel_is_outlier = false;
double chi_square = 0;
double outlier_l2_norm_ratio = 1.5;

static void pub_result(const ros::Time &stamp)
{
    geometry_msgs::TwistStamped odom;
    odom.header.stamp = stamp;
    odom.twist.linear.x  = x(0);
    odom.twist.linear.y  = x(1);
    odom.twist.linear.z  = x(2);
    odom.twist.angular.x = x(3);
    odom.twist.angular.y = x(4);
    odom.twist.angular.z = chi_square / OUTLIER_THRESHOLD; // DEBUG_only
    filter_pub.publish(odom);
}

static void pub_preprocessed(const std_msgs::Header &header,
                      const Ref<const Vector3d> p,
                      const Ref<const Vector3d> v)
{
    geometry_msgs::TwistStamped debug;
    debug.header = header;
    debug.twist.linear.x  = p[0];
    debug.twist.linear.y  = p[1];
    debug.twist.linear.z  = p[2];
    debug.twist.angular.x = v[0];
    debug.twist.angular.y = v[1];
    debug.twist.angular.z = v[2];
    debug_pub.publish(debug);
}

// Chi-square test for outlier rejection
static bool velocity_is_outlier(const Vector3d &pos,
                                const Vector3d &vel)
{
    VectorXd r = MatrixXd::Zero(6, 1);
    VectorXd z = MatrixXd::Zero(6, 1);
    MatrixXd S = MatrixXd::Zero(6, 6);
    z << pos[0], pos[1], pos[2], vel[0], vel[1], vel[2];

    double pos_norm   = z.segment<3>(0).norm();
    double state_norm = x.segment<3>(0).norm();
    ROS_INFO("pos_norm %f, state_norm %f", pos_norm, state_norm);
    if (pos_norm > state_norm * outlier_l2_norm_ratio) {
        return true;
    }

    // r = z - H * x, residual
    // S = H P H' + R, residual covariance
    MatrixXd H = MatrixXd::Identity(6, 6); // observation matrix

//    MatrixXd K_next = (H * P * H.transpose() + R).ldlt().solve(P * H.transpose());
//    MatrixXd x_next = x + K_next * (z - H * x);
//    MatrixXd P_next = P - K_next * H * P;

    r = z - H * x;
    S = H * P * H.transpose() + R;
    chi_square = r.transpose() * S * r;
    cout << "chi_square " << endl << chi_square << endl;

    return (chi_square > OUTLIER_THRESHOLD);
}

static void preprocess_visual(const std_msgs::Header &header,
                              const geometry_msgs::Twist &twist,
                              Vector3d &pos, Vector3d &vel)
{
    ros::Time t_update = header.stamp;
    double dt_update = (t_update - t_prev).toSec();
    Vector3d camera_T_shield, imu_T_shield, imu_vel_shield;

    camera_T_shield[0] = twist.linear.x;
    camera_T_shield[1] = twist.linear.y;
    camera_T_shield[2] = twist.linear.z;
    imu_T_shield = imu_R_camera * camera_T_shield + imu_T_camera;
    imu_T_shield *= 0.001; // Convert millimeter to meter

    // calculate and check the velocity
    dt_update = (dt_update < CV_UPDATE_TIME_MAX) ? dt_update : CV_UPDATE_TIME_MAX;
    imu_vel_shield = (imu_T_shield - imu_T_shield_prev) / dt_update;

    vel_is_outlier = velocity_is_outlier(imu_T_shield, imu_vel_shield);

    if (vel_is_outlier) {
        pos = x.segment<3>(0);
        vel = x.segment<3>(3);
    }
    else {
        pos = imu_T_shield;
        vel = imu_vel_shield;
    }


    // store the state
    pub_preprocessed(header, imu_T_shield, imu_vel_shield);
    imu_T_shield_prev = imu_T_shield;
    t_prev = t_update;
}

static void propagate(const double &dt) {
    A.topRightCorner(3, 3) = dt * MatrixXd::Identity(3, 3);
    // cout << "A " << endl << A << endl;
    x = A * x;
    P = A * P * A.transpose() + Q;
}

static void update(const Vector3d &pos,
                   const Vector3d &vel)
{
    MatrixXd H, K, z;

    H = MatrixXd::Identity(6, 6); // observation matrix
    // MatrixXd K = P * H.transpose() * (H * P * H.transpose() + R).inverse();
    K = (H * P * H.transpose() + R).ldlt().solve(P * H.transpose());
    z = MatrixXd::Zero(6, 1);
    z << pos[0], pos[1], pos[2], vel[0], vel[1], vel[2];

    x = x + K * (z - H * x);
    P = P - K * H * P;
}

static void repropagate() {
    for (int i = 0; i < REPROPAGATE_TIME; ++i) {
        propagate(1.0 / ROS_FREQ);
    }
}

/**
 * initialization of the state and convariance from visual
 * @param pnp
 */
static void initialize_visual(const std_msgs::Header &header,
                              const geometry_msgs::Twist &twist)
{
    ros::Time t_update = header.stamp;
    ROS_INFO("visual init at %f", t_update.toSec());

    Vector3d camera_T_shield, imu_T_shield;
    camera_T_shield[0] = twist.linear.x;
    camera_T_shield[1] = twist.linear.y;
    camera_T_shield[2] = twist.linear.z;
    imu_T_shield = imu_R_camera * camera_T_shield + imu_T_camera;
    imu_T_shield *= 0.001; // Convert millimeter to meter

    x.segment<3>(0) = imu_T_shield; // init velocity with zero
    x.segment<3>(3).setZero(); // init velocity with zero

    cout << "DEBUG: x initialized with " << endl << x.transpose() << endl;
    imu_T_shield_prev = imu_T_shield;
    t_prev = t_update;
    propagate_count = 0;
    visual_initialized = true;
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
        if (!visual_initialized) {
            initialize_visual(pnp->header, pnp->twist);
        }
        else {
//            double pos_x = 0, pos_y = 0, vel_x = 0, vel_y = 0;
            Vector3d pos, vel;

            preprocess_visual(pnp->header, pnp->twist, pos, vel);

            update(pos, vel);
            propagate_count = 0;
            repropagate();

            pub_result(pnp->header.stamp);
        }
    }
}


/**
 * handle, save, and process visual messages
 * @param armor
 */
void real_visual_cb(const rm_cv::ArmorRecord::ConstPtr &armor)
{
    visual_valid = !(armor->armorPose.linear.x == 0 &&
                     armor->armorPose.linear.y == 0 &&
                     armor->armorPose.linear.z == 0 );

    if (visual_valid) {
        if (!visual_initialized) {
            initialize_visual(armor->header, armor->armorPose);
        }
        else {
//            double pos_x = 0, pos_y = 0, vel_x = 0, vel_y = 0;
            Vector3d pos, vel;

            preprocess_visual(armor->header, armor->armorPose, pos, vel);

            update(pos, vel);
            propagate_count = 0;
            repropagate();

            pub_result(armor->header.stamp);
        }
    }

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "prediction_kalman_filter");
    ros::NodeHandle n("~");

    n.param("visual_topic", visual_topic, string("/pnp_twist"));
    n.param("real_visual_topic", real_visual_topic, string("/detected_armor"));
    n.param("publisher_topic", publisher_topic, string("/prediction_kf/predict"));
    n.param("debug_topic", debug_topic, string("/prediction_kf/preprocessed"));
    n.param("repropagate_time", REPROPAGATE_TIME, 5);
    n.param("chi_square_threshold", OUTLIER_THRESHOLD, 10000.0);
    n.param("outlier_l2_norm_ratio", outlier_l2_norm_ratio, 1.5);
    n.param("position_weight", pos_weight, 2000.0);
    n.param("velocity_weight", vel_weight, 10000.0);
/*
    // For chassis reading only
    imu_R_camera <<  0, 0, 1,
                    -1, 0, 0,
                     0,-1, 0;
    imu_T_camera <<  200, 50, 0; // in millimeter
    */


    x.setZero();
    R.topLeftCorner(3, 3)     = pos_weight * MatrixXd::Identity(3, 3);
    R.bottomRightCorner(3, 3) = vel_weight * MatrixXd::Identity(3, 3);

    ros::Subscriber s1 = n.subscribe(visual_topic, 40, visual_callback);
    ros::Subscriber s2 = n.subscribe(real_visual_topic, 40, real_visual_cb);
    filter_pub = n.advertise<geometry_msgs::TwistStamped>(publisher_topic, 40);
    debug_pub  = n.advertise<geometry_msgs::TwistStamped>(debug_topic, 40);

    ros::Rate r(ROS_FREQ);
    while (ros::ok()) {
        if (visual_initialized) {
            propagate(1.0 / ROS_FREQ);
            propagate_count++;
            ros::Duration delta_propagate(1.0 / ROS_FREQ * propagate_count);
            ros::Time t_propagate = t_prev + delta_propagate;
            // ROS_INFO("propagate at %f", t_propagate.toSec());
            pub_result(t_propagate);

        }
        r.sleep();
        ros::spinOnce();
    }
}