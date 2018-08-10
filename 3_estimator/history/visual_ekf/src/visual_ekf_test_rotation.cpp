#include <iostream>
#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <Eigen/Eigen>
#include <queue>

using namespace Eigen;
using namespace std;
ros::Publisher twist_pub, debug_pub;
const double interval = 1000.0;
const double dt = 0.01;

int main(int argc, char **argv) {
    ros::init(argc, argv, "visual_ekf_test");
    ros::NodeHandle n("~");

    twist_pub = n.advertise<geometry_msgs::Vector3Stamped>("/visual_ekf/test_twist", 100);
    debug_pub = n.advertise<geometry_msgs::PoseStamped>("/visual_ekf/test_pose_debug", 100);

    ros::Rate r(1 / dt);

    Quaterniond start(1, 0, 0, 0);
    Quaterniond end(1, 2, 3, 100);
    end = end.normalized();

    Quaterniond q = end * start.conjugate();
    Quaterniond q_state = start;
    AngleAxisd R(q);
    double theta  = R.angle();
    Vector3d axis = R.axis();

    auto t_start = ros::Time::now();
    double t_total = interval * dt;
    // theta = theta / t_total;

    while (ros::ok()) {
        auto cur_t = ros::Time::now();
        if ((cur_t.toSec() - t_start.toSec()) < t_total) {
            geometry_msgs::Vector3Stamped twist;
            twist.header.stamp = cur_t;
            twist.vector.x = 0;
            twist.vector.y = 0;
            twist.vector.z = 0;
            twist_pub.publish(twist);

            geometry_msgs::PoseStamped pose;
            pose.header.stamp = cur_t;
            pose.pose.orientation.w = start.w();
            pose.pose.orientation.x = start.x();
            pose.pose.orientation.y = start.y();
            pose.pose.orientation.z = start.z();
            debug_pub.publish(pose);
        }
        else if ((cur_t.toSec() - t_start.toSec()) < t_total * 2) {
            geometry_msgs::Vector3Stamped twist;
            twist.header.stamp = cur_t;
            twist.vector.x = theta * axis(0);
            twist.vector.y = theta * axis(1);
            twist.vector.z = theta * axis(2);
            twist_pub.publish(twist);


            // AngleAxisd dR(d_theta, axis);
            Quaterniond dq;
            dq.w() = cos(0.5 * theta * dt);
            dq.x() = sin(0.5 * theta * dt) * axis(0);
            dq.y() = sin(0.5 * theta * dt) * axis(1);
            dq.z() = sin(0.5 * theta * dt) * axis(2);
            q_state = (q_state * dq).normalized();

            geometry_msgs::PoseStamped pose;
            pose.header.stamp = cur_t;
            pose.pose.orientation.w = q_state.w();
            pose.pose.orientation.x = q_state.x();
            pose.pose.orientation.y = q_state.y();
            pose.pose.orientation.z = q_state.z();
            debug_pub.publish(pose);
        }

        r.sleep();
        ros::spinOnce();
    }
}