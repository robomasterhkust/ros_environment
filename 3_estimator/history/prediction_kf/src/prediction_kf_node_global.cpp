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
#include <queue>
#include "rm_cv/ArmorRecord.h"

using namespace std;
using namespace Eigen;

string attitude_topic, publisher_topic, debug_topic, real_visual_topic, transform_topic;
string debug_angle_topic;
ros::Publisher filter_pub, debug_pub, transform_pub, debug_angle_pub;
MatrixXd gimbal_R_camera = MatrixXd::Identity(3, 3); // rotation matrix from camera to imu
MatrixXd init_R_gimbal= MatrixXd::Identity(3, 3);
Vector3d gimbal_T_camera = MatrixXd::Zero(3, 1);

VectorXd x(6);      // state
MatrixXd P = MatrixXd::Identity(6, 6); // covariance
MatrixXd Q = MatrixXd::Identity(6, 6); // prediction noise covariance
MatrixXd R = MatrixXd::Identity(3, 3); // observation noise covariance
MatrixXd A = MatrixXd::Identity(6, 6); // state transfer function

bool visual_initialized = false, visual_valid = false;

ros::Time t_prev;
double R_pos, Q_pos, Q_vel, P_weight;
const double CV_UPDATE_TIME_MAX = 0.2; // maxium allowed update time
const double DELAY_MAX = 0.05;
const int ROS_FREQ = 100;
const int MAX_VISUAL_QUEUE_SIZE = 5;
const int MAX_IMU_QUEUE_SIZE = 1000;
double OUTLIER_THRESHOLD = 10000.0;

Vector3d init_T_shield_prev = MatrixXd::Zero(3, 1);
Vector3d imu_T_shield_prev = MatrixXd::Zero(3, 1);
bool pos_is_outlier = false;
double chi_square = 0;
double outlier_l2_norm_ratio = 1.5;
double yaw_delay = 0.0;
double pitch_delay = 0.0;
Vector3d OUTPUT_BOUND = MatrixXd::Zero(3, 1);

// synchronization
queue<geometry_msgs::QuaternionStamped::ConstPtr> imu_queue;
queue<Vector3d> visual_queue;
int imu_back_time = 10;


double angle_diff = 0;
bool angle_diff_not_inited = true;

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

static void pub_angle_debug(const std_msgs::Header &header,
                            const double yaw,
                            const Ref<const Vector3d> p,
                            const double imuz)
{
    geometry_msgs::TwistStamped debug_angle;
    debug_angle.header = header;
    debug_angle.twist.angular.x = yaw;
    debug_angle.twist.angular.y = -atan2(p[1], p[0]);
    debug_angle.twist.angular.z = imuz;
    debug_angle_pub.publish(debug_angle);
}

static double imu_to_yaw_angle(const geometry_msgs::QuaternionStamped::ConstPtr &imu)
{
    // Quaterniond q = Quaterniond( imu->quaternion.w, imu->quaternion.x, imu->quaternion.y, imu->quaternion.z );
    double qw = imu->quaternion.w;
    double qx = imu->quaternion.x;
    double qy = imu->quaternion.y;
    double qz = imu->quaternion.z;

    // convert to ZYX Euler angle yaw angle
    // double euler_angle_0 = atan2(2.0 * (qw * qx + qy * qz), 1.0 - 2.0 * (qx * qx + qy * qy) );
    // double euler_angle_1 = asin( 2.0 * (qw * qy - qz * qx) );
    return atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz) );
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
    // ROS_INFO("pos_norm %f, state_norm %f", pos_norm, state_norm);
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
    gimbal_T_shield = gimbal_R_camera * camera_T_shield + gimbal_T_camera;
    gimbal_T_shield *= 0.001; // Convert millimeter to meter

    Matrix3d world_R_gimbal;// = MatrixXd::Identity(3, 3);
    world_R_gimbal << 1, 0, 0, 0, 1, 0, 0, 0, 1;
    double yaw = 0;
    double imuz= 0;

/*    int pop_time = 0;
    if (!imu_queue.empty()) {
        pop_time = imu_queue.size() - imu_back_time;

        if (pop_time > 0) {
            for (int i = 0; i < pop_time; ++i) {
                imu_queue.pop();
            }
            imuz = imu_queue.front()->quaternion.z;
            yaw = imu_to_yaw_angle(imu_queue.front());
            world_R_gimbal << cos(yaw), -sin(yaw), 0, sin(yaw), cos(yaw), 0, 0, 0, 1;

            if (angle_diff_not_inited) {
                angle_diff =  yaw + atan2(gimbal_T_shield[1], gimbal_T_shield[0]);
                angle_diff_not_inited = false;
                ROS_INFO("angle_diff is, %f", angle_diff);
            }
        }
    }*/
    init_T_shield = world_R_gimbal * gimbal_T_shield;

    // calculate and check the velocity
    init_vel_shield = (init_T_shield - init_T_shield_prev) / dt_update;
    init_T_shield_prev = init_T_shield;

    pos_is_outlier = translation_is_outlier(init_T_shield);

    if (pos_is_outlier) {
        pos = x.segment<3>(0);
        x.segment<3>(3) *= 0.5;
        ROS_INFO("outlier rejected");
    }
    else {
        pos = init_T_shield;
    }

    // store the state
    pub_preprocessed(header, gimbal_T_shield, gimbal_vel_shield, debug_pub);

    pub_preprocessed(header, init_T_shield_prev, init_vel_shield, transform_pub);

    pub_angle_debug(header, yaw - angle_diff, gimbal_T_shield, imuz);
}

/**
 * Core Kalman Filter math, propagate and update
 */
static void propagate(const double &dt) {
    A.topRightCorner(3, 3) = dt * MatrixXd::Identity(3, 3);
    x = A * x;
    P = A * P * A.transpose() + Q;
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

    gimbal_T_shield = gimbal_R_camera * camera_T_shield + gimbal_T_camera;
    gimbal_T_shield *= 0.001; // Convert millimeter to meter

    Matrix3d world_R_gimbal;// = MatrixXd::Identity(3, 3);
    world_R_gimbal << 1, 0, 0, 0, 1, 0, 0, 0, 1;
    double yaw = 0;

/*    int pop_time = 0;
    if (!imu_queue.empty()) {
        pop_time = imu_queue.size() - imu_back_time;

        if (pop_time > 0) {
            for (int i = 0; i < pop_time; ++i) {
                imu_queue.pop();
            }
            yaw = imu_to_yaw_angle(imu_queue.front());
            // double euler_angle_2 = imu_to_yaw_angle(imu_queue.back());
            world_R_gimbal << cos(yaw), -sin(yaw), 0, sin(yaw), cos(yaw), 0, 0, 0, 1;
            double DEBUG_imu_delay_time = header.stamp.toSec() - imu_queue.front()->header.stamp.toSec();
            cout << "imu to visual time difference " << DEBUG_imu_delay_time << endl;
        }
        cout << "final poped times " << pop_time << endl;
    }*/

    cout << "world_R_gimbal " << endl << world_R_gimbal << endl;

    init_T_shield = world_R_gimbal * gimbal_T_shield;

    // TODO: using RANSAC instead of averaging filter
    visual_queue.push(init_T_shield);
    if (visual_queue.size() >= MAX_VISUAL_QUEUE_SIZE) {
        Vector3d T_sum;
        T_sum.setZero();

        for (int i = 0; i < MAX_VISUAL_QUEUE_SIZE; ++i) {
            T_sum += visual_queue.front();
            visual_queue.pop();
        }

        x.segment<3>(0) = T_sum / MAX_VISUAL_QUEUE_SIZE; // init velocity with zero
        x.segment<3>(3).setZero(); // init velocity with zero

        P.setZero();
        P.topLeftCorner(3, 3)     = P_weight * MatrixXd::Identity(3, 3);
        P.bottomRightCorner(3, 3) = 2 * P_weight * MatrixXd::Identity(3, 3);

        cout << "DEBUG: x initialized with " << endl << x.transpose() << endl;
        cout << "P " << endl << P << endl;

        init_T_shield_prev = init_T_shield;
        visual_initialized = true;
    }
    else {
        cout << "DEBUG: current x reading: " << init_T_shield.transpose() << endl;
    }
}


/**
 * handle and save attitude of the gimbal
 * @param imu
 */
void attitude_cb(const geometry_msgs::QuaternionStamped::ConstPtr &imu)
{
    // synchronize the timestamp
    imu_queue.push(imu);
    if (imu_queue.size() > MAX_IMU_QUEUE_SIZE) imu_queue.pop();
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
        ros::Time t_update = armor->header.stamp;
        double dt_update = (t_update - t_prev).toSec();

        if (!visual_initialized) {
            initialize_visual(armor->header, armor->armorPose);
        }
        else {
            if (dt_update < CV_UPDATE_TIME_MAX) {
                Vector3d pos;

                preprocess_visual(armor->header, armor->armorPose, pos, dt_update);

                update(pos);

                propagate(dt_update);

                double running_time = ros::Time::now().toSec();
                double delay_dt = running_time - armor->header.stamp.toSec();
                delay_dt = (delay_dt < DELAY_MAX) ? delay_dt : DELAY_MAX;

                pub_result(armor->header.stamp, delay_dt);
            }
            else {
                visual_queue.push(x.segment<3>(0));

                visual_initialized = false;

                initialize_visual(armor->header, armor->armorPose);
            }
        }
        t_prev = t_update;
    }

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "prediction_kalman_filter");
    ros::NodeHandle n("~");

    n.param("attitude_topic", attitude_topic, string("/can_receive_node/attitude"));
    n.param("real_visual_topic", real_visual_topic, string("/detected_armor"));
    n.param("publisher_topic", publisher_topic, string("/prediction_kf_global/predict"));
    n.param("debug_topic", debug_topic, string("/prediction_kf_global/preprocessed"));
    n.param("transform_topic", transform_topic, string("/prediction_kf_global/transformed"));
    n.param("debug_angle_topic", debug_angle_topic, string("/prediction_kf/debug_angle"));
    n.param("chi_square_threshold", OUTLIER_THRESHOLD, 10000.0);
    n.param("outlier_l2_norm_ratio", outlier_l2_norm_ratio, 1.5);
    n.param("imu_back_time", imu_back_time, 10);
    n.param("R_pos", R_pos, 16.0);
    n.param("Q_pos", Q_pos, 0.2);
    n.param("Q_vel", Q_vel, 1.0);
    n.param("P_matrix_weight", P_weight, 1.0);
    n.param("yaw_delay", yaw_delay, 0.0);
    n.param("pitch_delay", pitch_delay, 0.0);

    // For chassis reading only
    gimbal_R_camera <<  0, 0, 1,
                    -1, 0, 0,
                     0,-1, 0;
    gimbal_T_camera <<  30, 0, -120; // in millimeter

    OUTPUT_BOUND << 10.0, 10.0, 20.0;

    x.setZero();
    R = R_pos * MatrixXd::Identity(3, 3);
    Q.topLeftCorner(3, 3)     = Q_pos * MatrixXd::Identity(3, 3);
    Q.bottomRightCorner(3, 3) = Q_vel * MatrixXd::Identity(3, 3);
    cout << "R " << endl << R << endl;
    cout << "Q " << endl << Q << endl;

    ros::Subscriber s1 = n.subscribe(attitude_topic, 100, attitude_cb);
    ros::Subscriber s2 = n.subscribe(real_visual_topic, 40, real_visual_cb);

    filter_pub = n.advertise<rm_cv::ArmorRecord>(publisher_topic, 40);
    debug_pub  = n.advertise<geometry_msgs::TwistStamped>(debug_topic, 40);
    transform_pub = n.advertise<geometry_msgs::TwistStamped>(transform_topic, 40);
    debug_angle_pub = n.advertise<geometry_msgs::TwistStamped>(debug_angle_topic, 100);
    ros::Rate r(ROS_FREQ);

    while (ros::ok()) {
        if (visual_initialized) {

        }

        r.sleep();
        ros::spinOnce();
    }
}
