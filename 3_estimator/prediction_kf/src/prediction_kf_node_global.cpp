/**
 * August 8th, 2018
 * source: https://zhuanlan.zhihu.com/p/38745950
 * Constant velocity prediction Kalman filter for shield global position
 */

#include <iostream>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include "rm_cv/ArmorRecord.h"

using namespace std;
using namespace Eigen;

string attitude_topic, publisher_topic, debug_topic, real_visual_topic, transform_topic;
ros::Publisher filter_pub, debug_pub, transform_pub;
MatrixXd imu_R_camera = MatrixXd::Identity(3, 3); // rotation matrix from camera to imu
MatrixXd init_R_gimbal= MatrixXd::Identity(3, 3);
Vector3d imu_T_camera = MatrixXd::Zero(3, 1);

VectorXd x(6);      // state
MatrixXd P = MatrixXd::Identity(6, 6); // covariance
MatrixXd Q = MatrixXd::Identity(6, 6); // prediction noise covariance
MatrixXd R = MatrixXd::Identity(3, 3); // observation noise covariance
MatrixXd A = MatrixXd::Identity(6, 6); // state transfer function

bool visual_initialized = false, visual_valid = false;

ros::Time t_prev;
double R_pos, Q_pos, Q_vel, P_weight;
const double CV_UPDATE_TIME_MAX = 0.1; // maxium allowed update time
const double DELAY_MAX = 0.05;
const int ROS_FREQ = 100;
double OUTLIER_THRESHOLD = 10000.0;

Vector3d init_T_shield_prev = MatrixXd::Zero(3, 1);
Vector3d imu_T_shield_prev = MatrixXd::Zero(3, 1);
bool pos_is_outlier = false;
double chi_square = 0;
double outlier_l2_norm_ratio = 1.5;
double yaw_delay = 0.0;
double pitch_delay = 0.0;
Vector3d OUTPUT_BOUND = MatrixXd::Zero(3, 1);;

static void pub_result(const ros::Time &stamp, double delay_dt)
{
//    geometry_msgs::TwistStamped odom;
    rm_cv::ArmorRecord odom;
    odom.header.stamp = stamp;
    double predict_x = x(0) + (delay_dt + yaw_delay)   * x(3);
    double predict_y = x(1) + (delay_dt + pitch_delay) * x(4);
    double predict_z = x(2) + delay_dt * x(5);
    odom.armorPose.linear.x  = (predict_x < OUTPUT_BOUND[0]) ? predict_x * 1000 : OUTPUT_BOUND[0] * 1000;
    odom.armorPose.linear.y  = (predict_y < OUTPUT_BOUND[1]) ? predict_y * 1000 : OUTPUT_BOUND[1] * 1000;
    odom.armorPose.linear.z  = (predict_z < OUTPUT_BOUND[2]) ? predict_z * 1000 : OUTPUT_BOUND[2] * 1000;
    odom.armorPose.angular.x = x(3) * 1000;
    odom.armorPose.angular.y = x(4) * 1000;
    odom.armorPose.angular.z = x(5) * 1000;
    filter_pub.publish(odom);
}

static void pub_preprocessed(const std_msgs::Header &header,
                      const Ref<const Vector3d> p,
                      const Ref<const Vector3d> v,
                      ros::Publisher &publisher)
{
    geometry_msgs::TwistStamped debug;
    debug.header = header;
    debug.twist.linear.x  = p[0];
    debug.twist.linear.y  = p[1];
    debug.twist.linear.z  = p[2];
    debug.twist.angular.x = v[0];
    debug.twist.angular.y = v[1];
    debug.twist.angular.z = v[2];
    publisher.publish(debug);
}

// Chi-square test for outlier rejection
static bool translation_is_outlier(const Vector3d &pos)
{
    VectorXd r = MatrixXd::Zero(3, 1);
    VectorXd z = MatrixXd::Zero(3, 1);
    MatrixXd S = MatrixXd::Zero(3, 3);
    z << pos[0], pos[1], pos[2];

    double pos_norm   = z.segment<3>(0).norm();
    double state_norm = x.segment<3>(0).norm();
    ROS_INFO("pos_norm %f, state_norm %f", pos_norm, state_norm);
    if (pos_norm > state_norm * outlier_l2_norm_ratio) {
        return true;
    } else if (pos_norm < state_norm / outlier_l2_norm_ratio) {
        return true;
    }

    // r = z - H * x, residual
    // S = H P H' + R, residual covariance
    MatrixXd H = MatrixXd::Identity(3, 6); // observation matrix

//    MatrixXd K_next = (H * P * H.transpose() + R).ldlt().solve(P * H.transpose());
//    MatrixXd x_next = x + K_next * (z - H * x);
//    MatrixXd P_next = P - K_next * H * P;

    r = z - H * x;
    S = H * P * H.transpose() + R;
    chi_square = r.transpose() * S * r;
    // cout << "chi_square " << endl << chi_square << endl;

    return (chi_square > OUTLIER_THRESHOLD);
}

static void preprocess_visual(const std_msgs::Header &header,
                              const geometry_msgs::Twist &twist,
                              Vector3d &pos, double dt_update)
{
    Vector3d camera_T_shield, gimbal_T_shield, init_T_shield;
    Vector3d camera_vel_shield, gimbal_vel_shield, init_vel_shield;
    camera_T_shield[0] = twist.linear.x;
    camera_T_shield[1] = twist.linear.y;
    camera_T_shield[2] = twist.linear.z;
    gimbal_T_shield = imu_R_camera * camera_T_shield + imu_T_camera;
    gimbal_T_shield *= 0.001; // Convert millimeter to meter
    init_T_shield = init_R_gimbal * gimbal_T_shield;

    // calculate and check the velocity
    init_vel_shield = (init_T_shield - init_T_shield_prev) / dt_update;
    init_T_shield_prev = init_T_shield;

    pos_is_outlier = translation_is_outlier(init_T_shield);

    if (pos_is_outlier) {
        pos = x.segment<3>(0);
        ROS_INFO("outlier rejected");
    }
    else {
        pos = init_T_shield;
    }

    // store the state
    pub_preprocessed(header, camera_T_shield, camera_vel_shield, debug_pub);

    pub_preprocessed(header, init_T_shield, init_vel_shield, transform_pub);
}

static void propagate(const double &dt) {
    A.topRightCorner(3, 3) = dt * MatrixXd::Identity(3, 3);
    // cout << "A " << endl << A << endl;
    x = A * x;
    P = A * P * A.transpose() + Q;
    // cout << "P " << endl << P << endl;
}

static void update(const Vector3d &pos)
{
    MatrixXd H, K, z;

    H = MatrixXd::Identity(3, 6); // observation matrix
    K = P * H.transpose() * (H * P * H.transpose() + R).inverse();
    z = MatrixXd::Zero(3, 1);
    z << pos[0], pos[1], pos[2];

    x = x + K * (z - H * x);
    P = P - K * H * P;
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

    Vector3d camera_T_shield, gimbal_T_shield, init_T_shield;
    camera_T_shield[0] = twist.linear.x;
    camera_T_shield[1] = twist.linear.y;
    camera_T_shield[2] = twist.linear.z;
    gimbal_T_shield = imu_R_camera * camera_T_shield + imu_T_camera;
    gimbal_T_shield *= 0.001; // Convert millimeter to meter
    init_T_shield = init_R_gimbal * gimbal_T_shield;

    x.segment<3>(0) = init_T_shield; // init velocity with zero
    x.segment<3>(3).setZero(); // init velocity with zero

    cout << "DEBUG: x initialized with " << endl << x.transpose() << endl;

    init_T_shield_prev = init_T_shield;
    t_prev = t_update;
    visual_initialized = true;
}

void attitude_cb(const geometry_msgs::QuaternionStamped::ConstPtr &imu)
{
    // TODO: synchronize the timestamp
    Quaterniond q_now = Quaterniond( imu->quaternion.w, imu->quaternion.x, imu->quaternion.y, imu->quaternion.z );
    init_R_gimbal = q_now.toRotationMatrix();
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

            double running_time = ros::Time::now().toSec();
            double delay_dt = running_time - armor->header.stamp.toSec();
            delay_dt = (delay_dt < DELAY_MAX) ? delay_dt : DELAY_MAX;

            ros::Time t_update = armor->header.stamp;
            double dt_update = (t_update - t_prev).toSec();
            dt_update = (dt_update < CV_UPDATE_TIME_MAX) ? dt_update : CV_UPDATE_TIME_MAX;

            Vector3d pos;
            preprocess_visual(armor->header, armor->armorPose, pos, dt_update);

            update(pos);

            propagate(dt_update);

            pub_result(armor->header.stamp, delay_dt);
            t_prev = t_update;
        }
    }

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "prediction_kalman_filter");
    ros::NodeHandle n("~");

    n.param("attitude_topic", attitude_topic, string("/can_receive_node/attitude"));
    n.param("real_visual_topic", real_visual_topic, string("/detected_armor"));
    n.param("publisher_topic", publisher_topic, string("/prediction_kf/predict"));
    n.param("debug_topic", debug_topic, string("/prediction_kf/preprocessed"));
    n.param("transform_topic", transform_topic, string("/prediction_kf/transformed"));
    n.param("chi_square_threshold", OUTLIER_THRESHOLD, 10000.0);
    n.param("outlier_l2_norm_ratio", outlier_l2_norm_ratio, 1.5);
    n.param("R_pos", R_pos, 16.0);
    n.param("Q_pos", Q_pos, 0.2);
    n.param("Q_vel", Q_vel, 1.0);
    n.param("P_matrix_weight", P_weight, 1.0);
    n.param("yaw_delay", yaw_delay, 0.0);
    n.param("pitch_delay", pitch_delay, 0.0);

    // For chassis reading only
    imu_R_camera <<  0, 0, 1,
                    -1, 0, 0,
                     0,-1, 0;
    imu_T_camera <<  150, 0, -50; // in millimeter

    OUTPUT_BOUND << 10000.0, 10000.0, 10000.0;

    x.setZero();
    R = R_pos * MatrixXd::Identity(3, 3);

    Q.topLeftCorner(3, 3)     = Q_pos * MatrixXd::Identity(3, 3);
    Q.bottomRightCorner(3, 3) = Q_vel * MatrixXd::Identity(3, 3);
    P.topLeftCorner(3, 3)     = P_weight * MatrixXd::Identity(3, 3);

    P.bottomRightCorner(3, 3) = 2 * P_weight * MatrixXd::Identity(3, 3);
    cout << "R " << endl << R << endl;
    cout << "Q " << endl << Q << endl;
    cout << "P " << endl << P << endl;

    ros::Subscriber s1 = n.subscribe(attitude_topic, 100, attitude_cb);
    ros::Subscriber s2 = n.subscribe(real_visual_topic, 40, real_visual_cb);

    filter_pub = n.advertise<rm_cv::ArmorRecord>(publisher_topic, 40);
    debug_pub  = n.advertise<geometry_msgs::TwistStamped>(debug_topic, 40);
    transform_pub = n.advertise<geometry_msgs::TwistStamped>(transform_topic, 40);
    ros::Rate r(ROS_FREQ);
    ros::spin();

}
