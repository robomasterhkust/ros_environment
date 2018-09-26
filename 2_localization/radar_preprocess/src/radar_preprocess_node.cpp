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

KalmanFilter kf;

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
    R << 0.01;
    P << 1, 0,
         0, 1;

    // initialize the filter
    KalmanFilter kf_init(dt, A, H, Q, R, P);
    Eigen::VectorXd x0(n);
    x0 << distance, 0;

    // create a prediction with Kalman filter
    kf_init.init(ros::Time::now().toSec(), x0);
    kf = kf_init;
	filter_initialized = true;
	ROS_INFO("initialized the radar kalman filter at %f", distance);
}

// change the ID to the sorted order
void
publish_message(const int id, const std_msgs::Header& header)
{
    geometry_msgs::Point32 point;

	point.x = kf.state()[0]; // estimated state
	point.y = kf.state()[1]; // estimated velocity
	point.z = id; 

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

    if (s > 0) {
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
            int chi_index = s;
            bool found_nearest_point = false;
			int id = 0;

            // compare the chi-square to select into one
            for ( i = 0; i < s; ++i) {
                Eigen::VectorXd z(m);
                z << pc_ptr->points[i].x;

                double chiSquare = kf.chiSquare(z);
                if (chiSquare < minChiSquare) {
                    minChiSquare = chiSquare;
                    chi_index = i;
                }
            }
            double residual = pc_ptr->points[chi_index].x - kf.state()[0];
            if (residual < 1 && residual > -1)
                found_nearest_point = true;

            if (found_nearest_point) {
                ROS_INFO("found the number at %d with X2 %f", chi_index, minChiSquare);

                kf.propagate();

                Eigen::VectorXd z_sel(m);
                z_sel << pc_ptr->points[chi_index].x;
                kf.update(z_sel);

                publish_message(id, pc_ptr->header);
            } 
			else {
				ROS_INFO("radar does not track the obstacle");
				// kf.propagate();

                publish_message(id, pc_ptr->header);
			}
        }
    }
	else {
		ROS_INFO("no point available");
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
