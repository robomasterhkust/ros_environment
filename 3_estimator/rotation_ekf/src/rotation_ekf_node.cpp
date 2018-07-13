/**
 * by Beck on 7/12/18.
 * rotation ekf used in the real system
 */
#include <iostream>
#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <queue>
#include "rm_cv/ArmorRecord.h"
#include "can_receive_msg/imu_16470.h"

using namespace std;
using namespace Eigen;

string predict_topic_in, preprocess_topic_in;
string publisher_topic, predict_pub_topic, preprocess_pub_topic;
string imu_topic;
ros::Publisher pose_pub, predict_pub, preprocess_pub;

/**
 * Kalman filter related
 * Define states:
 *      x = [rotation_quaternion]
 * Define inputs:
 *      u = [dq], quaternion change on pose
 * Define noises:
 *      n = [n_gyro]
 */
VectorXd x(4);                          // state
MatrixXd P = MatrixXd::Identity(3, 3);  // covariance
MatrixXd R = MatrixXd::Identity(3, 3);  // prediction noise covariance
MatrixXd Q = MatrixXd::Identity(3, 3);  // observation noise covariance

MatrixXd imu_R_camera = MatrixXd::Identity(3, 3);
Vector3d imu_T_camera = MatrixXd::Zero(3, 1);

/**
 * Sensor processing related
 */
bool gyro_initialized = false;
bool visual_initialized = false;
Quaterniond q_last = Quaterniond::Identity();

/*
 * publish the predict result in angle axis
 */
static void pub_pose(std_msgs::Header header, const AngleAxisd& angle_axis, ros::Publisher &pub)
{
    geometry_msgs::PoseStamped pose;
    pose.header = header;
    pose.pose.orientation.w = angle_axis.angle();
    pose.pose.orientation.x = angle_axis.axis()[0];
    pose.pose.orientation.y = angle_axis.axis()[1];
    pose.pose.orientation.z = angle_axis.axis()[2];
    pub.publish(pose);
}


static void propagate(const Quaterniond& dq)
{
    // Quaterniond dq(1, domg(0), domg(1), domg(2));
    // ROS_INFO("dq w x y z %f %f %f %f", dq.w(), dq.x(), dq.y(), dq.z() );
    Quaterniond q_state(x(0), x(1), x(2), x(3));
    Quaterniond q = (q_state * dq).normalized();

    x(0) = q.w();
    x(1) = q.x();
    x(2) = q.y();
    x(3) = q.z();

    // TODO: compare the hardcoded dt with real message dt
    double dt = 0.01;
    Vector3d w = dq.vec() * 2 / dt;
    Matrix3d w_hat;
    w_hat << 0,-w(2), w(1),
            w(2), 0, -w(0),
            -w(1), w(0), 0;
    Matrix3d A = -w_hat;

    MatrixXd U = -MatrixXd::Identity(3, 3);

    MatrixXd F, V;
    F = MatrixXd::Identity(3, 3) + dt * A;
    V = dt * U;

    P = F * P * F.transpose() + V * R * V.transpose();
    // cout << "P " << endl << P << endl;
    // cout << "x " << endl << x.transpose() << endl;
}

static void update(const Quaterniond& qm)
{
    Matrix3d C = Matrix3d::Identity();

    MatrixXd K(3, 3);
    K = P * C.transpose() * (C * P * C.transpose() + Q).inverse();

    VectorXd r(3); // residual
    // Quaterniond qm = camera_q_shield;
    Quaterniond q  = Quaterniond(x(0), x(1), x(2), x(3));
    Quaterniond dq = q.conjugate() * qm;
    // ROS_INFO("dq w x y z %f %f %f %f", dq.w(), dq.x(), dq.y(), dq.z() );
    r = 2 * dq.vec();
    Vector3d _r = K * r;
    // ROS_INFO("dr x y z %f %f %f", _r[0], _r[1], _r[2] );
    Vector3d dw(_r(0) * 0.5, _r(1) * 0.5, _r(2) * 0.5);
    dq = Quaterniond(sqrt(1 - dw.squaredNorm()), dw(0), dw(1), dw(2)).normalized();
    q  = q * dq;

    x(0) = q.w();
    x(1) = q.x();
    x(2) = q.y();
    x(3) = q.z();

    P = P - K * C * P;
    // cout << "P " << endl << P << endl;
    // cout << "x " << endl << x.transpose() << endl;
}

//void predict_callback(const geometry_msgs::TwistStamped::ConstPtr &pnp)
void predict_callback(const rm_cv::ArmorRecord::ConstPtr &armor)
{
    Vector3d camera_T_shield = MatrixXd::Zero(3, 1);
    camera_T_shield <<  armor->armorPose.linear.x,
                        armor->armorPose.linear.y,
                        armor->armorPose.linear.z;
    Vector3d imu_T_shield = imu_T_camera + imu_R_camera * camera_T_shield;
    Vector3d T_norm = imu_T_shield.normalized();
    Vector3d x_axis = Vector3d::UnitX();
    Vector3d axis = T_norm.cross(x_axis).normalized();
    double angle = acos( T_norm.dot(x_axis) );
    AngleAxisd camera_R_shield(angle, axis);
    Quaterniond camera_q_shield(camera_R_shield);

    pub_pose(armor->header, camera_R_shield, predict_pub);

    // update
    update(camera_q_shield);

    Quaterniond q_state(x(0), x(1), x(2), x(3));
    AngleAxisd angle_axis_state(q_state);
    pub_pose(armor->header, angle_axis_state, pose_pub);
}


void preprocess_callback(const geometry_msgs::TwistStamped::ConstPtr &pnp)
{
    Vector3d camera_T_shield_rot = MatrixXd::Zero(3, 1);
    camera_T_shield_rot <<  pnp->twist.linear.x,
                            pnp->twist.linear.y,
                            pnp->twist.linear.z;
    Vector3d imu_T_shield = imu_T_camera + camera_T_shield_rot;
    Vector3d T_norm = imu_T_shield.normalized();
    Vector3d x_axis = Vector3d::UnitX();
    Vector3d axis = T_norm.cross(x_axis).normalized();
    double angle = acos( T_norm.dot(x_axis) );
    AngleAxisd camera_R_shield(angle, axis);
    Quaterniond camera_q_shield(camera_R_shield);

    pub_pose(pnp->header, camera_R_shield, preprocess_pub);
}


void imu_callback(const can_receive_msg::imu_16470::ConstPtr &pose)
{
    Vector4d raw_q(pose->quaternion[0],
                   pose->quaternion[1],
                   pose->quaternion[2],
                   pose->quaternion[3]);
    Quaterniond q = Quaterniond::Identity();

    bool raw_q_valid = false;
    if (raw_q.norm() <= 1 &&
        raw_q[0] <= 1 && raw_q[1] <= 1 &&
        raw_q[2] <= 1 && raw_q[3] <= 1 ) {
        raw_q_valid = true;
        Vector4d q_normalized = raw_q.normalized();
        q.w() = q_normalized[0];
        q.x() = q_normalized[1];
        q.y() = q_normalized[2];
        q.z() = q_normalized[3];
    }

    // init imu, store the pose
    if (raw_q_valid) {
        if (!gyro_initialized) {
            q_last = q;
            gyro_initialized = true;
        }
        else {
            // chi-square test

            // propagate
            propagate(q_last.conjugate() * q);
        }
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rotation_ekf");
    ros::NodeHandle n("~");

    n.param("predict_result", predict_topic_in, string("/prediction_kf/predict"));
    n.param("preprocessed_result", preprocess_topic_in, string("/prediction_kf/preprocessed"));
    n.param("publisher_topic", publisher_topic, string("/rotation_ekf/filtered"));
    n.param("predict_topic", predict_pub_topic, string("/rotation_ekf/predict"));
    n.param("preprocess_topic", preprocess_pub_topic, string("/rotation_ekf/preprocess"));

    n.param("imu_topic", imu_topic, string("/can_receive_node/imu_16470"));

    ros::Subscriber s1 = n.subscribe(predict_topic_in, 10, predict_callback);
    ros::Subscriber s2 = n.subscribe(preprocess_topic_in, 10, preprocess_callback);
    ros::Subscriber s3 = n.subscribe(imu_topic, 10, imu_callback);
    pose_pub       = n.advertise<geometry_msgs::PoseStamped>(publisher_topic, 100);
    predict_pub    = n.advertise<geometry_msgs::PoseStamped>(predict_pub_topic, 100);
    preprocess_pub = n.advertise<geometry_msgs::PoseStamped>(preprocess_pub_topic, 100);

    imu_R_camera << 0, 0, 1,
                   -1, 0, 0,
                    0,-1, 0;
    imu_T_camera << 150, 45, -30;

    ros::Rate r(100);
    ros::spin();
}

