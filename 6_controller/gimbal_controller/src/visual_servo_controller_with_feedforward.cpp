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
#include "rm_cv/vertice.h"
#include "VisualServoControllerWithFeedForward.h"
#include "camera_model/camera_models/CameraFactory.h"

using namespace std;
using namespace Eigen;

ros::Publisher cmd_pub, debug_pub_typeI, debug_pub_typeII;
string cv_topic, publisher_topic, debug_typeI_topic, debug_typeII_topic;

// Camera
std::string cfg_file_name = "/home/nvidia/ws/src/6_controller/gimbal_controller/cfg/camera_tracking_camera_calib.yaml";
camera_model::CameraPtr m_camera;

// Controller
int n = 4;
int m = 2;
double Kp = 1.0;
MatrixXd target_image_frame(n, m);

VisualServoControllerWithFeedForward ctl;

void
publish_message(const double wz, const ros::Publisher& pub)
{
    geometry_msgs::Twist vel_msg;
    vel_msg.angular.z = wz;
    pub.publish(vel_msg);
}


// No need to reject noise
void
visual_servo_cb(const rm_cv::vertice::ConstPtr cv_ptr)
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
    ctl.setKp(Kp);
    VectorXd ctl_val = ctl.control();

    // publish the message in camera y axis
    publish_message(ctl_val(1), cmd_pub);
}



int main(int argc, char **argv) {
    ros::init(argc, argv, "four_point_visual_servo");
    ros::NodeHandle nh("~");

    nh.param("Kp", Kp, 1.0);
    nh.param("cv_topic", cv_topic, string("/detected_vertice"));
    nh.param("publisher_topic", publisher_topic, string("/cmd_vel"));

    ros::Subscriber sub = nh.subscribe(cv_topic, 10, visual_servo_cb);
    cmd_pub = nh.advertise<geometry_msgs::Twist>(publisher_topic, 10);

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

    ros::spin();
}
