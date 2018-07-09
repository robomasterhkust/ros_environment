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

using namespace std;
using namespace Eigen;

string visual_topic, publisher_topic, debug_topic;
ros::Publisher filter_pub, debug_pub;
MatrixXd imu_R_camera = MatrixXd::Identity(3, 3); // rotation matrix from camera to imu
Vector3d imu_T_camera = MatrixXd::Zero(3, 1);

VectorXd x(4);      // state
MatrixXd P = MatrixXd::Identity(4, 4); // covariance
MatrixXd Q = MatrixXd::Identity(4, 4); // prediction noise covariance
MatrixXd R = MatrixXd::Identity(4, 4); // observation noise covariance
MatrixXd A = MatrixXd::Identity(4, 4); // state transfer function

bool visual_initialized = false, visual_valid = false;

ros::Time t_prev;
double pos_weight, vel_weight;
const double CV_UPDATE_TIME_MAX = 0.1; // maxium allowed update time
const int ROS_FREQ = 100;
int REPROPAGATE_TIME = 4; // repropagate after the update
int propagate_count = 0;
double velocity_threshold = 100;
Vector3d imu_T_shield_prev = MatrixXd::Zero(3, 1);
bool vel_is_outlier = false;

static void pub_result(const ros::Time &stamp)
{
    geometry_msgs::TwistStamped odom;
    odom.header.stamp = stamp;
    odom.twist.linear.x  = x(0);
    odom.twist.linear.y  = x(1);
    odom.twist.angular.x = x(2);
    odom.twist.angular.y = x(3);
    filter_pub.publish(odom);
}

static void pub_debug(const std_msgs::Header &header,
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
static bool velocity_is_outlier(const double *vel_x, const double *vel_y)
{
    Vector2d r = MatrixXd(2, 1);
    MatrixXd S = MatrixXd(2, 2);
    // r = z - H * x, residual
    // S = H P H' + R, residual covariance
    r << *vel_x - x(2), *vel_y - x(3);
    S = P.bottomRightCorner(2, 2) + R.bottomRightCorner(2, 2);
    MatrixXd chi_square = r.transpose() * S * r;
    cout << "chi_square " << endl << chi_square << endl;

    return (chi_square(0, 0) > velocity_threshold);
}

static void preprocess_visual(const geometry_msgs::TwistStamped &pnp,
                              double *pos_x, double *pos_y,
                              double *vel_x, double *vel_y)
{
    ros::Time t_update  = pnp.header.stamp;
    double dt_update = (t_update - t_prev).toSec();
    Vector3d camera_T_shield, imu_T_shield, imu_vel_shield;

    camera_T_shield[0] = pnp.twist.linear.x;
    camera_T_shield[1] = pnp.twist.linear.y;
    camera_T_shield[2] = pnp.twist.linear.z;
    imu_T_shield = imu_R_camera * camera_T_shield + imu_T_camera;
    imu_T_shield *= 0.001; // Convert millimeter to meter

    // calculate and check the velocity
    dt_update = (dt_update < CV_UPDATE_TIME_MAX) ? dt_update : CV_UPDATE_TIME_MAX;
    imu_vel_shield = (imu_T_shield - imu_T_shield_prev) / dt_update;
    vel_is_outlier = velocity_is_outlier(vel_x, vel_y);
    if (vel_is_outlier) { imu_vel_shield.setZero(); }

    *pos_x = imu_T_shield(0);
    *pos_y = imu_T_shield(1);
    *vel_x = imu_vel_shield(0);
    *vel_y = imu_vel_shield(1);

    // store the state
    pub_debug(pnp.header, imu_T_shield, imu_vel_shield);
    imu_T_shield_prev = imu_T_shield;
    t_prev = t_update;
}

static void propagate(const double &dt) {
    A.topRightCorner(2, 2) = dt * MatrixXd::Identity(2, 2);
    // cout << "A " << endl << A << endl;
    x = A * x;
    P = A * P * A.transpose() + Q;
}

static void update(const double *pos_x, const double *pos_y,
                   const double *vel_x, const double *vel_y)
{
    MatrixXd H, K, z;
    // if (!*vel_is_outlier) {
    H = MatrixXd::Identity(4, 4); // observation matrix
    // MatrixXd K = P * H.transpose() * (H * P * H.transpose() + R).inverse();
    K = (H * P * H.transpose() + R).ldlt().solve(P * H.transpose());
    z = MatrixXd::Zero(4, 1);
    z << *pos_x, *pos_y, *vel_x, *vel_y;
    // }

/*
    else {
        H = MatrixXd::Identity(2, 4); // observation matrix
        cout << "H in outlier" << endl << H << endl;
        K = P * H.transpose() * (H * P * H.transpose() + R.topLeftCorner(2, 2)).inverse();
        cout << "K in outlier" << endl << K << endl;
        z = MatrixXd::Zero(2, 1);
        z << *pos_x, *pos_y;
    }
*/

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
static void initialize_visual(const geometry_msgs::TwistStamped::ConstPtr &pnp)
{
    ros::Time t_update = pnp->header.stamp;
    ROS_INFO("visual init at %f", t_update.toSec());

    Vector3d camera_T_shield, imu_T_shield;
    camera_T_shield[0] = pnp->twist.linear.x;
    camera_T_shield[1] = pnp->twist.linear.y;
    camera_T_shield[2] = pnp->twist.linear.z;
    imu_T_shield = imu_R_camera * camera_T_shield + imu_T_camera;
    imu_T_shield *= 0.001; // Convert millimeter to meter

    x(0) = imu_T_shield(0);
    x(1) = imu_T_shield(1);
    x(2) = 0.0; // init velocity with zero
    x(3) = 0.0;

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
            initialize_visual(pnp);
        }
        else {
            double pos_x = 0, pos_y = 0, vel_x = 0, vel_y = 0;
            preprocess_visual(*pnp, &pos_x, &pos_y, &vel_x, &vel_y);
            if (!vel_is_outlier) {
                update(&pos_x, &pos_y, &vel_x, &vel_y);
                propagate_count = 0;
                repropagate();
            }
            pub_result(pnp->header.stamp);
        }
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "prediction_kalman_filter");
    ros::NodeHandle n("~");

    n.param("visual_topic", visual_topic, string("/pnp_twist"));
    n.param("publisher_topic", publisher_topic, string("/prediction_kf/predict"));
    n.param("debug_topic", debug_topic, string("/prediction_kf/debug"));
    n.param("repropagate_time", REPROPAGATE_TIME, 5);
    n.param("chi_square_threshold", velocity_threshold, 100.0);
    n.param("position_weight", pos_weight, 2000.0);
    n.param("velocity_weight", vel_weight, 10000.0);
    imu_R_camera <<  0, 0, 1,
                    -1, 0, 0,
                     0,-1, 0;

    imu_T_camera <<  200, 50, 0; // in millimeter

    x.setZero();
    R.topLeftCorner(2, 2)     = pos_weight * MatrixXd::Identity(2, 2);
    R.bottomRightCorner(2, 2) = vel_weight * MatrixXd::Identity(2, 2);

    ros::Subscriber s1 = n.subscribe(visual_topic, 40, visual_callback);
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