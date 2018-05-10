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
    VectorXd T_acc = VectorXd::Zero(periods, 1);

    // A, derivative constraints matrix
    MatrixXd A = MatrixXd::Zero(periods * order, periods * order);

    // Q, cost function matrix
    MatrixXd Q = MatrixXd::Zero(periods * order, periods * order);

    // C, selection matrix, remove free variables and
    // break down the variables to fixed and process variables
    MatrixXd C = MatrixXd::Zero(periods * order,(periods + 1) * derivatives);

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
    while (ros::ok())
    {
        // stage II excute the trajectory to the motor
    }
}