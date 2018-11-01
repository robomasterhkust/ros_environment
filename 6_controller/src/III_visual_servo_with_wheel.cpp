/**
 * Beck Pang, 20181029, the ROS node to add the perspective controller with wheel command
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
//#include "rm_cv/vertice.h"
#include "filt.h"
#include "rm_cv/ArmorRecord.h"
#include "III_VisualServoController.h"
#include "camera_model/camera_models/CameraFactory.h"

#include <dynamic_reconfigure/server.h>
#include <visual_servo_control/tuningConfig.h>

using namespace std;
using namespace Eigen;

ros::Publisher cmd_pub;
ros::Publisher kalman_output_pub;
ros::Publisher omega_raw_pub;
ros::Publisher omega_visual_pub;
ros::Publisher kalman_input_pub;
ros::Publisher delayed_gyro_pub;
string cv_topic;
string omega_input_topic;
string kalman_input_topic;
string kalman_output_topic;
string omega_raw_topic;
string omega_visual_topic;
string publisher_topic;
string delayed_gyro_topic;

// Camera
std::string cfg_file_name;

camera_model::CameraPtr m_camera;

// Controller
int n = 4;
int m = 2;
double Kp = 1.0;
double Kd = 0.0;
double Kf_r0 = 0.01;
double Kf_q0 = 1.0;
double ctrl_freq = 30;

double target_Z = 1;
double pixel_x_max = 640;
double pixel_y_max = 512;
double pixel_dx = 68;
double pixel_dy = 25;

// low pass Filter for distance
Filter *z_low_pass;

// Handle acyronized observer and control
VectorXd prev_ctl_val(6);
int cv_state_machine = 0;

bool visual_updated = false;
bool gyro_updated = false;
MatrixXd target_image_frame(n, m);

VisualServoController ctl;

enum class fsm {idle = 0, once = 1, multi = 2};

MatrixXd last_input_image_frame(n, m);

/**
 * Publish the linear and angular command to the chassis
 * @param cmd
 * @param pub
 */
void
publish_cmd(const Eigen::VectorXd &cmd, const ros::Publisher& pub)
{
    geometry_msgs::Twist vel_msg;
    vel_msg.linear.x  = cmd[0];
    vel_msg.linear.y  = cmd[1];
    vel_msg.linear.z  = cmd[2];
    vel_msg.angular.x = cmd[3];
    vel_msg.angular.y = cmd[4];
    vel_msg.angular.z = cmd[5];
    pub.publish(vel_msg);
}

void
publish_angular_velocity(const Eigen::VectorXd &estimated_omega, const ros::Publisher& pub)
{
    geometry_msgs::TwistStamped omega_msg;
    omega_msg.header.stamp = ros::Time::now();
    omega_msg.twist.angular.x = estimated_omega[0];
    omega_msg.twist.angular.y = estimated_omega[1];
    omega_msg.twist.angular.z = estimated_omega[2];
    pub.publish(omega_msg);
}

void
visual_handle_feature(const Eigen::MatrixXd &input_pixel)
{
    visual_updated = true;

    Vector3d input_pixel_output[n];
    MatrixXd input_image_frame(n, m);

    for (int i = 0; i < n; ++i) {
        m_camera->liftSphere(input_pixel.row(i), input_pixel_output[i] );
        input_image_frame.row(i) << input_pixel_output[i](0), input_pixel_output[i](1);
    }
    // std::cout << "input in image frame " << std::endl << input_image_frame << std::endl;

    last_input_image_frame = input_image_frame;
}

/*
void
visual_feature_cb(const rm_cv::vertice::ConstPtr cv_ptr)
{
    // convert the pixel value to image coordinate value
    MatrixXd input_pixel(n, m);

    for (int i = 0; i < n; ++i) {
        input_pixel(i, 0) = cv_ptr->vertex[i].x;
        input_pixel(i, 1) = cv_ptr->vertex[i].y;
    }

    visual_handle_feature(input_pixel);
}
 */


void
visual_feature_cb_with_z(const rm_cv::ArmorRecord::ConstPtr cv_ptr)
{
    MatrixXd input_pixel(n, m);

    for (int i = 0; i < n; ++i) {
        input_pixel(i, 0) = cv_ptr->vertex[i].x;
        input_pixel(i, 1) = cv_ptr->vertex[i].y;
    }

    visual_handle_feature(input_pixel);

    double z_filt = z_low_pass->do_sample(cv_ptr->armorPose.linear.z);
    ctl.setZ(z_filt);
}

void
omega_cam_cb(const geometry_msgs::TwistStamped::ConstPtr omega_ptr){
    gyro_updated = true;

    MatrixXd input_omega(3, 1);
    input_omega(0, 0) = omega_ptr->twist.angular.x;
    input_omega(1, 0) = omega_ptr->twist.angular.y;
    input_omega(2, 0) = omega_ptr->twist.angular.z;

    Eigen::MatrixXd end_R_cam(3, 3);

    end_R_cam <<
        0, -1, 0,
        0, 0, -1,
        1, 0, 0;

    VectorXd input_omega_cam = end_R_cam * input_omega;
    // std::cout << "input in omega" << std::endl << input_omega << std::endl;
    publish_angular_velocity(input_omega_cam, omega_raw_pub);

    ctl.updateOmega(input_omega_cam);
}

/**
 * callback function for dynamic reconfigure
 */
void configCallback(visual_servo_control::tuningConfig &config, uint32_t level __attribute__((unused)))
{
    Kp = config.Kp;
    Kd = config.Kd;
    Kf_r0 = config.Kf_r0;
    Kf_q0 = config.Kf_q0;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "four_point_visual_servo");
    ros::NodeHandle nh("~");

    nh.param("Kp", Kp, -1.0);
    nh.param("Kd", Kd, 0.0);
    nh.param("Kf_r0", Kf_r0, 0.01);
    nh.param("Kf_r0", Kf_r0, 1.0);
    nh.param("cv_topic", cv_topic, string("/detected_armor"));
    nh.param("omega_input_topic", omega_input_topic, string("/can_receive_1/end_effector_omega"));

    nh.param("publisher_topic", publisher_topic, string("/cmd_vel"));
    nh.param("kalman_input_topic",  kalman_input_topic,  string("/visual_servo/kalman_input"));
    nh.param("kalman_output_topic", kalman_output_topic, string("/visual_servo/kalman_output"));
    nh.param("omega_raw_topic", omega_raw_topic, string("/visual_servo/raw_omega_cam"));
    nh.param("omega_visual_topic", omega_visual_topic, string("/visual_servo/omega_visual"));
    nh.param("delayed_gyro_topic", delayed_gyro_topic, string("/visual_servo/delayed_gyro"));

    nh.param("cfg_file_name", cfg_file_name, string("/home/nvidia/ws/src/6_controller/gimbal_controller/cfg/camera_tracking_camera_calib.yaml"));

    // dynamic reconfigure server
    dynamic_reconfigure::Server<visual_servo_control::tuningConfig> dr_server;
    dynamic_reconfigure::Server<visual_servo_control::tuningConfig>::CallbackType dr_callback;
    dr_callback = boost::bind(&configCallback, _1, _2);
    dr_server.setCallback(dr_callback);

    ros::Subscriber sub1 = nh.subscribe(cv_topic, 10, visual_feature_cb_with_z);
    ros::Subscriber sub2 = nh.subscribe(omega_input_topic, 10, omega_cam_cb);
    cmd_pub = nh.advertise<geometry_msgs::Twist>(publisher_topic, 10);
    omega_raw_pub     = nh.advertise<geometry_msgs::TwistStamped>(omega_raw_topic, 10);
    omega_visual_pub  = nh.advertise<geometry_msgs::TwistStamped>(omega_visual_topic, 10);
    kalman_input_pub  = nh.advertise<geometry_msgs::TwistStamped>(kalman_input_topic, 10);
    kalman_output_pub = nh.advertise<geometry_msgs::TwistStamped>(kalman_output_topic, 10);
    delayed_gyro_pub  = nh.advertise<geometry_msgs::TwistStamped>(delayed_gyro_topic, 10);

    // create a camera model
    m_camera = camera_model::CameraFactory::instance()->generateCameraFromYamlFile(cfg_file_name);

    fsm finite_state = fsm::idle;

    // create the low pass filter
    z_low_pass = new Filter(LPF, 4, 30, 3);

    // setup the target coordinate
    MatrixXd target_pixel(n, m);
    MatrixXd target_image_frame(n, m);

    double pixel_x_down= (pixel_x_max - pixel_dx) * 0.5;
    double pixel_x_top_= (pixel_x_max + pixel_dx) * 0.5;
    double pixel_y_down= (pixel_y_max - pixel_dy) * 0.5;
    double pixel_y_top_= (pixel_y_max + pixel_dy) * 0.5;

    target_pixel <<
          pixel_x_down, pixel_y_down,
    	    pixel_x_down, pixel_y_top_,
    	    pixel_x_top_, pixel_y_down,
    	    pixel_x_top_, pixel_y_top_; // 1000 mm

    Vector3d target_pixel_output[n];

    for	(int i=0; i < n; ++i) {
        m_camera->liftSphere(target_pixel.row(i), target_pixel_output[i] );
        target_image_frame.row(i) << target_pixel_output[i](0), target_pixel_output[i](1);
    }
    std::cout << "target in image frame " << std::endl << target_image_frame << std::endl;

    // set up the controller
    ctl.setTarget(target_image_frame);

    ctl.setTargetZ(target_Z);

    ctl.setZ(target_Z);

    ctl.setControlFrequency(ctrl_freq);

    ctl.initKalmanFilter(Kf_r0, Kf_q0, 1 / ctrl_freq);

    prev_ctl_val.setZero();

    Eigen::MatrixXd cam_R_end(6, 6);
    cam_R_end.setIdentity();

    // linear velocity vx, vy, and vz also need to switch to chassis frame
    cam_R_end.block(0, 0, 3, 3) <<
           0, -1, 0,
           -1, 0, 0,
           0,  0, 1;

    cam_R_end.block(3, 3, 3, 3) <<
         0, 0, 1,
        -1, 0, 0,
         0,-1, 0;
    std::cout << "cam_R_end_effector: " << std::endl << cam_R_end << std::endl;

    ros::Rate rate(ctrl_freq);

    while (ros::ok()) {
        /**
         * Change setable values in the controller with dynamics configure
         */
        ctl.setKp(Kp);
        ctl.setKd(Kd);
        ctl.setKalmanR(Kf_r0);
        ctl.setKalmanQ(Kf_q0);

        /**
         * finite finite_state machine
         */
        switch (finite_state){
            case fsm::idle:
                if (visual_updated) finite_state = fsm::once;
                else                finite_state = fsm::idle;
                break;
            case fsm::once:
                if (visual_updated) finite_state = fsm::multi;
                else                finite_state = fsm::idle;
                break;
            case fsm::multi:
                if (visual_updated) finite_state = fsm::multi;
                else                finite_state = fsm::idle;
                break;
        }

        if (finite_state == fsm::idle) {
            ctl.finite_state = 0;

            publish_cmd(prev_ctl_val, cmd_pub);
            prev_ctl_val.setZero();
        }
        else if (finite_state == fsm::once) {
            ctl.finite_state = 1;

            ctl.updateFeatures(last_input_image_frame);
            VectorXd ctl_val = ctl.control();

            publish_cmd(cam_R_end * ctl_val, cmd_pub);
            prev_ctl_val = cam_R_end * ctl_val;
        }
        else if (finite_state == fsm::multi) {
            ctl.finite_state = 2;

            ctl.updateFeatures(last_input_image_frame);
            VectorXd ctl_val = ctl.control();

            publish_cmd(cam_R_end * ctl_val, cmd_pub);
            prev_ctl_val = cam_R_end * ctl_val;
        }

        if (finite_state == fsm::multi && gyro_updated) {
            VectorXd kalman_output = ctl.getKalmanOutput();
            VectorXd kalman_input  = ctl.getKalmanInput();
            VectorXd raw_visual_w  = ctl.getRawVisualOmega();
            VectorXd delay_gyro    = ctl.getDelayedGyro();
            publish_angular_velocity(kalman_output, kalman_output_pub);
            publish_angular_velocity(kalman_input, kalman_input_pub);
            publish_angular_velocity(raw_visual_w, omega_visual_pub);
            publish_angular_velocity(delay_gyro, delayed_gyro_pub);
        }

        // remove flag
        visual_updated = false;
        gyro_updated = false;
        rate.sleep();
        ros::spinOnce();
    }
}
