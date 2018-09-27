/**
 * Beck Pang, 20180926, trying to use the four points directly to build a robuster controller
 * Reference: Chaumette, François, and Seth Hutchinson. "Visual servo control. I. Basic approaches."
 */

#include <iostream>
#include <ros/ros.h>
#include <cmath>
#include <Eigen/Dense>
#include <geometry_msgs/Twist.h>
// #include "rm_cv/vertice.h"
#include "VisualServoController.h"

using namespace std;
using namespace Eigen;

ros::Publisher cmd_pub, debug_pub;
string cv_topic, publisher_topic;

/*
void
visual_servo_cb(const rm_cv::vertice::ConstPt cv_ptr)
{
    // check outlier


    // convert the pixel value to image coordinate value
    // TODO: class PinholeCamera

    MatrixXd input_image_frame(n, m);
    MatrixXd target_image_frame(n, m);

    // use the image frame coordinate value to control
    // TODO: class VisualServoController

}
*/

int main(int argc, char **argv) {
    ros::init(argc, argv, "four_point_visual_servo");
    ros::NodeHandle nh("~");

    nh.param("cv_topic", cv_topic, string("/detected_vertice"));
    nh.param("publisher_topic", publisher_topic, string("/cmd_vel"));

    // ros::Subscriber sub = nh.subscribe(cv_topic, 10, visual_servo_cb);
    cmd_pub = nh.advertise<geometry_msgs::Twist>(publisher_topic, 10);

    int n = 4;
    int m = 2;
    MatrixXd input_image_frame(n, m);
    MatrixXd target_image_frame(n, m);
    target_image_frame <<
            0.5, 0.2,
            0.5, -0.2,
            -0.5, 0.2,
            -0.5, -0.2;
    input_image_frame <<
            0.8, 0.3,
            0.8, -0.3,
            -0.2, 0.3,
            -0.2, -0.3;

    VisualServoController ctl(1, target_image_frame);
    Eigen::VectorXd ctl_val = ctl.control(input_image_frame);

    ros::spin();
}
