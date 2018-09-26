//
// Created by beck on 24/9/18.
//
#include <stdio.h>
#include <limits>
#include <string.h>
#include <Eigen/Dense>
#include <ros/ros.h>
#include "kalman.h"
#include "geometry_msgs/Point32.h"
#include "sensor_msgs/PointCloud.h"

ros::Subscriber radar_cloud_sub;
ros::Publisher filtered_cloud_pub;

bool filter_initialized = false;

KalmanFilter kf_one;

void
initialize_radar_filter(double distance)
{
    int n = 2;
    int m = 1;

    double dt = 1.0 / 14; // 14Hz from the radar

    Eigen::MatrixXd A(n, n);
    Eigen::MatrixXd H(m, n);
    Eigen::MatrixXd Q(n, n);
    Eigen::MatrixXd R(m, m);
    Eigen::MatrixXd P(n, n);

    // constant velocity model
    A << 1, dt,
         0, 1;
    H << 1, 0;
    Q << 1, 0,
         0, 1;
    R << 1;
    P << 1, 0,
         0, 1;

    // initialize the filter
    KalmanFilter kf(dt, A, H, Q, R, P);
    Eigen::VectorXd x0(n);
    x0 << distance, 0;

    // create a prediction with Kalman filter
    kf.init(ros::Time::now().toSec(), x0);
    kf_one = kf;
	filter_initialized = true;
	ROS_INFO("initialized the radar kalman filter at %f", distance);
}

void
publish_message(const geometry_msgs::Point32& point, const std_msgs::Header& header)
{
	sensor_msgs::PointCloud ptCloud;
	ptCloud.header = header;
	ptCloud.points.push_back(point);
	filtered_cloud_pub.publish(ptCloud);
}

void
radar_callback(const sensor_msgs::PointCloud::ConstPtr pc_ptr)
{
    int n = 2;
    int m = 1;
    int s = pc_ptr->points.size();
    int i;
    geometry_msgs::Point32 point;

    if (!filter_initialized) {
        double distance = 9.0;

        for ( i = 0; i < s; ++i) {
            double x_ = pc_ptr->points[i].x;
            if (x_ > 9 && x_ < 10)
                distance = x_;
        }
        initialize_radar_filter(distance);
    }
    else
    {
        double minChiSquare = std::numeric_limits<double>::max();
        int final_index = s;
		
        // compare the chi-square to select into one
        for ( i = 0; i < s; ++i) {
            Eigen::VectorXd z(m);
            z << pc_ptr->points[i].x;

            double chiSquare = kf_one.chiSquare(z);
            if (chiSquare < minChiSquare) {
                minChiSquare = chiSquare;
                final_index = i;
            }
        }
		ROS_INFO("found the number at %d with X2 %f", final_index, minChiSquare);

        Eigen::VectorXd z_sel(m);
        z_sel << pc_ptr->points[final_index].x;
        kf_one.propagate();
        kf_one.update(z_sel);

	    point.x = kf_one.state()[0]; // estimated state
		point.y = kf_one.state()[1]; // estimated velocity
		point.z = pc_ptr->channels[0].values[final_index]; // point cloud ID

		publish_message(point, pc_ptr->header);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "radar_preprocess");
    ros::NodeHandle n = ros::NodeHandle("~");

    radar_cloud_sub
    = n.subscribe<sensor_msgs::PointCloud>("/inf24g/inf24radar",
                                            10,
                                            radar_callback);
    filtered_cloud_pub
    = n.advertise<sensor_msgs::PointCloud>("/filtered_radar", 10);


    ros::spin();

    return 0;
}
