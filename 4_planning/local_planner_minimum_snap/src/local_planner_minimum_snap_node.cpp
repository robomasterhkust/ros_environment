/**
 * Beck Pang, 20180510, the local path planner
 * Mellinger, Daniel, and Vijay Kumar. "Minimum snap trajectory generation and control for quadrotors."
 * Robotics and Automation (ICRA), 2011 IEEE International Conference on. IEEE, 2011.
 */
#include <iostream>
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Eigen>
#include <geometry_msgs/Twist.h>

using namespace std;
using namespace Eigen;
ros::Publisher local_path_pub;
string pub_topic, path_topic;

// trajectory coefficients
unsigned int derivatives = 4;
unsigned int order = derivatives * 2;
double max_velocity = 10.0; // m/s

// stage I calculate the coefficients if new global path arrived
void path_callback(const nav_msgs::Path::ConstPtr &global_path) {
    unsigned long periods = global_path->poses.size() - 1;

    // T and T accumulative, time vector
    VectorXd T = VectorXd::Zero(periods, 1);
    VectorXd T_acc = VectorXd::Zero(periods + 1, 1);
    for (int i = 0; i < periods; ++i) {
        geometry_msgs::Pose pose_old = global_path->poses[i].pose;
        geometry_msgs::Pose pose_new = global_path->poses[i + 1].pose;
        double dx = pose_new.position.x - pose_new.position.x;
        double dy = pose_new.position.y - pose_new.position.y;
        double dz = pose_new.position.z - pose_new.position.z;
        T[i] = sqrt(pow(dx, 2) + pow(dy, 2) + pow(dz, 2)) / max_velocity;
        T_acc[i + 1] = T_acc[i] + T[i];
    }

    // A, derivative constraints matrix
    MatrixXd A = MatrixXd::Zero(periods * order, periods * order);
    for (int i = 0; i < periods; ++i) {
        double ii = i * order;
        A(ii, ii) = 1;
        A(ii + 1, ii + 1) = 1;
        A(ii + 2, ii + 2) = 1;
        A(ii + 3, ii + 3) = 1;
        for (int j = 0; j < order; ++j) { A(ii + 4, ii + j) = pow(T(i), j); }
        for (int j = 1; j < order; ++j) { A(ii + 5, ii + j) = pow(T(i), j - 1) * j; }
        for (int j = 2; j < order; ++j) { A(ii + 6, ii + j) = pow(T(i), j - 2) * j * (j - 1); }
        for (int j = 3; j < order; ++j) { A(ii + 7, ii + j) = pow(T(i), j - 3) * j * (j - 1) * (j - 2); }
    }

    // Q, cost function matrix
    MatrixXd Q = MatrixXd::Zero(periods * order, periods * order);
    for (int i = 0; i < periods; ++i) {
        double ii = i * order;
        for (int j = derivatives; j <= order; ++j) {
            for (int k = derivatives; k <= order; ++k) {
                Q(ii + j - 1, ii + k - 1) =
                        i * (i - 1) * (i - 2) * (i - 3) * j * (j - 1) * (j - 2) * (j - 3) / (i + j + 7) *
                        pow(T(k - 1), i + j - 7);
            }
        }
    }

    // C, selection matrix, remove free variables and
    // break down the variables to fixed and process variables
    MatrixXd C = MatrixXd::Zero(periods * order, (periods + 1) * derivatives);
    C(0, 0) = 1;
    for (int i = 0; i < periods - 2; ++i) {
        
    }

    // df, fixed derivative constraints
    // dp, process derivative constraints
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "local_planner_minimum_snap");
    ros::NodeHandle n("~");

//    ros::Duration(11).sleep();

    n.param("path_topic", path_topic, string("/global_path_hardcode"));
    n.param("pub_topic", pub_topic, string("cmd_vel"));
    ros::Subscriber sub = n.subscribe(path_topic, 10, path_callback);
    local_path_pub = n.advertise<geometry_msgs::Twist>(pub_topic, 100);

    // 400 Hz local path planner command
    while (ros::ok()) {
        // stage II excute the trajectory to the motor
    }
}