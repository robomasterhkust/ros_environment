/**
 * Beck Pang, 20181022, the ROS node to add the perspective controller with the target velocity feedforward
 * Reference:
 *  Chaumette, François, and Seth Hutchinson. "Visual servo control. I. Basic approaches.
 *  Chaumette, François, and Seth Hutchinson. "Visual servo control. II. Advanced approaches.
 */

#include <iostream>
#include <ros/ros.h>
#include <cmath>
#include <Eigen/Dense>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include "rm_cv/vertice.h"
#include "VisualServoControllerWithFeedForward.h"
#include "camera_model/camera_models/CameraFactory.h"

using namespace std;
using namespace Eigen;

ros::Publisher cmd_pub;
ros::Publisher omega_pub;
string cv_topic;
string omega_topic;
string publisher_topic;
string omega_pub_topic;

// Camera
std::string cfg_file_name;

camera_model::CameraPtr m_camera;

// Controller
int n = 4;
int m = 2;
double Kp = 1.0;
double Kd = 0.0;
MatrixXd target_image_frame(n, m);

VisualServoControllerWithFeedForward ctl;

void
publish_cmd(const double wz, const ros::Publisher& pub)
{
    geometry_msgs::Twist vel_msg;
    vel_msg.angular.z = wz;
    pub.publish(vel_msg);
}

void
publish_estimated_velocity(const Eigen::VectorXd &estimated_omega)
{
    geometry_msgs::TwistStamped omega_msg;
    omega_msg.header.stamp = ros::Time::now();
    omega_msg.twist.angular.x = estimated_omega[0];
    omega_msg.twist.angular.y = estimated_omega[1];
    omega_msg.twist.angular.z = estimated_omega[2];
    omega_pub.publish(omega_msg);
}

void
visual_feature_cb(const rm_cv::vertice::ConstPtr cv_ptr)
{
    // convert the pixel value to image coordinate value
    MatrixXd input_pixel(n, m);
    Vector3d input_pixel_output[n];
    MatrixXd input_image_frame(n, m);
	int i;
    for ( i = 0; i < n; ++i) {
        input_pixel(i, 0) = cv_ptr->vertex[i].x;
        input_pixel(i, 1) = cv_ptr->vertex[i].y;
        m_camera->liftSphere(input_pixel.row(i), input_pixel_output[i] );
        input_image_frame.row(i) << input_pixel_output[i](0), input_pixel_output[i](1);
    }
    std::cout << "input in image frame " << std::endl << input_image_frame << std::endl;

    // use the image frame coordinate value to control
    ctl.updateFeatures(input_image_frame);
}

void
omega_cam_cb(const geometry_msgs::TwistStamped::ConstPtr omega_ptr){
    MatrixXd input_omega(3, 1);
    input_omega(0, 0) = omega_ptr->twist.angular.x;
    input_omega(1, 0) = omega_ptr->twist.angular.y;
    input_omega(2, 0) = omega_ptr->twist.angular.z;

    std::cout << "input in omega" << std::endl << input_omega << std::endl;

    ctl.updateOmega(input_omega);
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "four_point_visual_servo");
    ros::NodeHandle nh("~");

    nh.param("Kp", Kp, -1.0);
    nh.param("Kd", Kd, 0.0);
    nh.param("cv_topic", cv_topic, string("/detected_vertice"));
    nh.param("omega_topic", omega_topic, string("/can_transimit/omega_cam"));
    nh.param("publisher_topic", publisher_topic, string("/cmd_vel"));
    nh.param("omega_pub_topic", omega_pub_topic, string("/visual_estimate"));
    nh.param("cfg_file_name", cfg_file_name, string("/home/ros/ws/src/6_controller/gimbal_controller/cfg/camera_tracking_camera_calib.yaml"));



    ros::Subscriber sub1 = nh.subscribe(cv_topic, 10, visual_feature_cb);
    ros::Subscriber sub2 = nh.subscribe(omega_topic, 10, omega_cam_cb);
    cmd_pub = nh.advertise<geometry_msgs::Twist>(publisher_topic, 10);
    omega_pub = nh.advertise<geometry_msgs::TwistStamped>(omega_pub_topic, 10);

    // create a camera model
    m_camera = camera_model::CameraFactory::instance()->generateCameraFromYamlFile(cfg_file_name);

	// setup the target coordinate
	MatrixXd target_pixel(n, m);
	MatrixXd target_image_frame(n, m);
	target_pixel << 595, 496,
                    595, 528,
                    685, 496,
                    685, 528; // 1650 mm
	
	Vector3d target_pixel_output[n];

	for	(int i=0; i < n; ++i) {
	    m_camera->liftSphere(target_pixel.row(i), target_pixel_output[i] );
        target_image_frame.row(i) << target_pixel_output[i](0), target_pixel_output[i](1);
	}
	std::cout << "target in image frame " << std::endl << target_image_frame << std::endl;
	
	ctl.setTarget(target_image_frame);

    ros::Rate rate(30);

    while (ros::ok()) {
        // publish the command in camera y axis

        ctl.setKp(Kp);
        ctl.setKd(Kd);
        VectorXd ctl_val = ctl.control();
        publish_cmd(ctl_val(1), cmd_pub);

        // publish the estimated velocity
        VectorXd estimated_vel = ctl.estimatePartialError();
        publish_estimated_velocity(estimated_vel);

        rate.sleep();
        ros::spinOnce();
    }
}
