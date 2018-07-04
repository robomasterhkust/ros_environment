/**
 * Created by Beck on 18/6/2018
 * Indirect Extended Kalman Filter for the pose
 * to get an estimate of the pose of the shield
 * fusing visual information with the gyroscope
 * With IMU fused reading comes in 100Hz and camera translation in 30Hz
 * V1: works for rotation only
 */
#include <iostream>
#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
// #include <Eigen/Eigen>
#include <Eigen/Geometry>
#include <queue>
//#include "rm_cv/ArmorRecord.h"

using namespace std;
using namespace Eigen;
ros::Publisher pose_pub, debug_pub, debug_gyro_pub;
string omg_topic, visual_topic, publisher_topic;
double gyro_weight;
double visual_q_weight, visual_t_weight;
int sleep_time;

/**
 * Define states:
 *      x = [rotation_quaternion]
 * Define inputs:
 *      u = [omg], filtered angular velocity
 * Define noises:
 *      n = [n_gyro]
 */
VectorXd x(4);                        // state
// MatrixXd P = MatrixXd::Zero(3, 3);     // covariance
MatrixXd P = MatrixXd::Identity(3, 3); // covariance
MatrixXd R = MatrixXd::Identity(3, 3); // prediction noise covariance
MatrixXd Q = MatrixXd::Identity(3, 3); // observation noise covariance

// buffers to save gyro and visual reading
queue<geometry_msgs::Vector3Stamped::ConstPtr> gyro_buf;
queue<geometry_msgs::TwistStamped::ConstPtr> visual_buf;
queue<Matrix<double, 4, 1>> x_history;
queue<Matrix<double, 3, 3>> P_history;
Vector3d G_body = {0, 0, 9.8}; // Consider to add initialization later

// previous propagated time
double t_prev;

// Initialization
const int GYRO_INIT_COUNT = 10;
const int MAX_GYRO_QUEUE_SIZE = 500;
int gyro_count  = 0;
bool gyro_initialized = false;
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

void propagate(const geometry_msgs::Vector3Stamped &gyro)
{
    double cur_t = gyro.header.stamp.toSec();
    // ROS_INFO("propagate");
    Vector3d w;
    w(0) = gyro.vector.x;
    w(1) = gyro.vector.y;
    w(2) = gyro.vector.z;

    double dt = cur_t - t_prev;
    // ROS_INFO("dt in propagate is %f, at the cur_t %f, with t_prev at %f", dt, cur_t, t_prev);
	// ROS_INFO("propagate, with dt %f", dt);
    Vector3d domg = 0.5 * dt * w;

    Quaterniond dq(sqrt(1 - domg.squaredNorm()), domg(0), domg(1), domg(2));
    // Quaterniond dq(1, domg(0), domg(1), domg(2));
	// ROS_INFO("dq w x y z %f %f %f %f", dq.w(), dq.x(), dq.y(), dq.z() );
    Quaterniond q_state(x(0), x(1), x(2), x(3));
    Quaterniond q = (q_state * dq).normalized();

    x(0) = q.w();
    x(1) = q.x();
    x(2) = q.y();
    x(3) = q.z();

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
	
    t_prev = cur_t;
	// cout << "P " << endl << P << endl;
	// cout << "x " << endl << x.transpose() << endl;
}

static void update(const geometry_msgs::TwistStamped::ConstPtr &pnp)
{
    Vector3d camera_T_shield;
    ROS_INFO("Update, at time %f", pnp->header.stamp.toSec());

    camera_T_shield[0] = pnp->twist.linear.x;
    camera_T_shield[1] = pnp->twist.linear.y;
    camera_T_shield[2] = pnp->twist.linear.z;
    Vector3d imu_T_shield = imu_R_camera * camera_T_shield + imu_T_camera;

    Vector3d T_norm = imu_T_shield.normalized();
    Vector3d x_axis = Vector3d::UnitX();
    // Vector3d axis = x_axis.cross(T_norm).normalized();
    // double angle = acos( x_axis.dot(T_norm) );
    Vector3d axis = T_norm.cross(x_axis).normalized();
    double angle = acos( T_norm.dot(x_axis) );
    AngleAxisd camera_R_shield(angle, axis);
    Quaterniond camera_q_shield(camera_R_shield);

/*
    Quaterniond camera_q_shield;
    camera_q_shield.w() = x_axis.dot(T_norm);
    camera_q_shield.x() = sin(0.5 * angle) * axis(0);
    camera_q_shield.y() = sin(0.5 * angle) * axis(1);
    camera_q_shield.z() = sin(0.5 * angle) * axis(2);
    camera_q_shield.normalize();
    // Debug original pose from camera
    pub_debug_pose(pnp->header, camera_q_shield);

    */

    pub_debug_angleAxis(pnp->header, axis, angle);

    Matrix3d C = Matrix3d::Identity();

    MatrixXd K(3, 3);
    K = P * C.transpose() * (C * P* C.transpose() + Q).inverse();

    VectorXd r(3); // residual
    Quaterniond qm = camera_q_shield;
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

/**
 * initialization of the gyroscope
 * @param gyro
 */
static void initialize_gyro(const geometry_msgs::Vector3Stamped::ConstPtr &gyro)
{
    t_prev = gyro->header.stamp.toSec();
    x_history.push(x);
    P_history.push(P);
    gyro_buf.push(gyro);
    gyro_count++;
    if (gyro_count == GYRO_INIT_COUNT) {
        gyro_initialized = true;
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

    camera_T_shield[0] = pnp->twist.linear.x;
    camera_T_shield[1] = pnp->twist.linear.y;
    camera_T_shield[2] = pnp->twist.linear.z;
    Vector3d imu_T_shield = imu_R_camera * camera_T_shield + imu_T_camera;

    Vector3d T_norm = imu_T_shield.normalized();
    Vector3d x_axis = Vector3d::UnitX();
    // Vector3d axis = x_axis.cross(T_norm).normalized();
    // double angle = acos( x_axis.dot(T_norm) );
    Vector3d axis = T_norm.cross(x_axis).normalized();
    double angle = acos( T_norm.dot(x_axis) );
    AngleAxisd camera_R_shield(angle, axis);
    Quaterniond camera_q_shield(camera_R_shield);
/*
    Quaterniond camera_q_shield;
    camera_q_shield.w() = x_axis.dot(T_norm);
    camera_q_shield.x() = sin(0.5 * angle) * axis(0);
    camera_q_shield.y() = sin(0.5 * angle) * axis(1);
    camera_q_shield.z() = sin(0.5 * angle) * axis(2);
    camera_q_shield.normalize();
*/

    x(0) = camera_q_shield.w();
    x(1) = camera_q_shield.x();
    x(2) = camera_q_shield.y();
    x(3) = camera_q_shield.z();
    P = MatrixXd::Identity(3, 3);
	ROS_INFO("visual init at %f", cur_t);
    cout << "DEBUG: x initialized with " << endl << x.transpose() << endl;

    // pub_fused_pose(pnp->header);
    pub_fused_angleAxis(pnp->header);
    t_prev = cur_t;

    visual_initialized = true;
}

/**
 * Processing the valid visual and gyro information
 */
static void state_machine_process(void)
{
    if (!gyro_initialized)
        return;
    //if (!visual_initialized)
    if (!visual_valid)
        return;

    while (!visual_buf.empty()) {
        while(!gyro_buf.empty() &&
              gyro_buf.front()->header.stamp < visual_buf.front()->header.stamp)
        {
            // trace backward the time to the imu timestamp
            t_prev = gyro_buf.front()->header.stamp.toSec();
            ROS_INFO("throw state with time: %f", t_prev);
            gyro_buf.pop();
            x_history.pop();
            P_history.pop();
        }

        geometry_msgs::TwistStamped::ConstPtr visual_msg = visual_buf.front();

        update(visual_msg);
        visual_buf.pop();
        // pub_fused_angleAxis(visual_msg->header);

        // Repropagate
        while (!x_history.empty()) x_history.pop();
        while (!P_history.empty()) P_history.pop();

        queue <geometry_msgs::Vector3Stamped::ConstPtr> temp_gyro_buf;
        while (!gyro_buf.empty())
        {
            propagate(*gyro_buf.front());
            temp_gyro_buf.push(gyro_buf.front());
            x_history.push(x);
            P_history.push(P);
            gyro_buf.pop();
        }
        swap(gyro_buf, temp_gyro_buf);

        // pub_fused_angleAxis();
    }
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

    if (visual_valid && visual_initialized) {

        visual_buf.push(pnp);
    }
    else if (visual_valid && !visual_initialized) {
        initialize_visual(pnp);
        visual_buf.push(pnp);
    }

    state_machine_process();
}

/**
 * handle 100Hz angular filtered gyroscope
 * @param gyro
 */
void gyro_callback(const geometry_msgs::Vector3Stamped::ConstPtr &gyro)
{
    if (!gyro_initialized) {
        initialize_gyro(gyro);
    }
    else {
        propagate(*gyro);
        // pub_fused_pose(gyro->header);
        pub_fused_angleAxis(gyro->header);
        x_history.push(x);
        P_history.push(P);
        gyro_buf.push(gyro);
		if (gyro_buf.size() > MAX_GYRO_QUEUE_SIZE) {
			x_history.pop();
			P_history.pop();
			gyro_buf.pop();
		}
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "visual_gyro_fused");
    ros::NodeHandle n("~");

    n.param("angular_fused", omg_topic, string("/dji_sdk/angular_velocity_fused")); // 100Hz
    n.param("visual_topic", visual_topic, string("/pnp_twist"));
    n.param("publisher_topic", publisher_topic, string("/visual_ekf/shield_pose_fused"));
    n.param("gyroscope_noise_weight", gyro_weight, 0.1);
    n.param("visual_pose_weight", visual_q_weight, 1.0);
    n.param("node_sleep_time", sleep_time, 10);

    // TODO: initalize the R and Q matrix
    R =     gyro_weight * MatrixXd::Identity(3, 3); // gyro noise
    Q = visual_q_weight * MatrixXd::Identity(3, 3); // observation noise

    x.setZero();
    x(0) = 1; // set quaternion to identity
    P = 1.0 * P;

    imu_R_camera <<  0, 0, 1,
                    -1, 0, 0,
                     0,-1, 0;

    imu_T_camera <<  200, 50, 0; // in millimeter

    ros::Subscriber s2 = n.subscribe(visual_topic, 10, visual_callback);
    ros::Subscriber s3 = n.subscribe(omg_topic, 100, gyro_callback);
//    pose_pub = n.advertise<geometry_msgs::PoseStamped>(publisher_topic, 100);
    pose_pub = n.advertise<geometry_msgs::PoseWithCovarianceStamped>(publisher_topic, 100);
    debug_pub= n.advertise<geometry_msgs::PoseStamped>(string("/visual_ekf/visual_ekf_debug"), 100);
//    debug_gyro_pub= n.advertise<geometry_msgs::Vector3Stamped>(string("/visual_ekf/gyro_debug"), 100);

    ros::Rate r(100);
    ros::spin();
}

/**
 *  0   q_w     shield frame --> camera frame
 *  1   q_x
 *  2   q_y
 *  3   q_z
 */




/*
        // DEBUG geometry relationship in visual_callback()
        update(pnp);
        pub_fused_angleAxis(pnp->header);

        geometry_msgs::TwistStamped::ConstPtr prev_pnp = visual_buf.front();

        Vector3d camera_T_shield;
        camera_T_shield[0] = prev_pnp->twist.linear.x;
        camera_T_shield[1] = prev_pnp->twist.linear.y;
        camera_T_shield[2] = prev_pnp->twist.linear.z;
        Vector3d imu_T_shield = imu_R_camera * camera_T_shield;

        Vector3d T_norm = imu_T_shield.normalized();
        Vector3d x_axis = Vector3d::UnitX();
    //    Vector3d axis = x_axis.cross(T_norm).normalized();
    //    double angle = acos( x_axis.dot(T_norm) );
        Vector3d axis = T_norm.cross(x_axis).normalized();
        double angle = acos( T_norm.dot(x_axis) );
        AngleAxisd R_prev(angle, axis);
        Quaterniond q_prev(R_prev);

        camera_T_shield[0] = pnp->twist.linear.x;
        camera_T_shield[1] = pnp->twist.linear.y;
        camera_T_shield[2] = pnp->twist.linear.z;
        imu_T_shield = imu_R_camera * camera_T_shield;

        T_norm = imu_T_shield.normalized();
    //    axis = x_axis.cross(T_norm).normalized();
    //    angle = acos( x_axis.dot(T_norm) );
        axis = T_norm.cross(x_axis).normalized();
        angle = acos( T_norm.dot(x_axis) );
        AngleAxisd R_now(angle, axis);
        Quaterniond q_now(R_now);

        Quaterniond dq = q_prev.conjugate() * q_now;
        // Quaterniond dq = q_now.conjugate() * q_prev;
        Vector3d d_theta = 2 * dq.vec();
        double dt_omg = pnp->header.stamp.toSec() - prev_pnp->header.stamp.toSec();
        Vector3d omg = d_theta / dt_omg;

        geometry_msgs::Vector3Stamped gyro_debug;
        gyro_debug.header = pnp->header;
        gyro_debug.vector.x = omg[0];
        gyro_debug.vector.y = omg[1];
        gyro_debug.vector.z = omg[2];
        propagate( gyro_debug );
        t_prev = pnp->header.stamp.toSec();
        debug_gyro_pub.publish(gyro_debug);

        visual_buf.pop();

        */
