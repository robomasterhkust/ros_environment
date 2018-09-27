//
// Created by beck on 26/9/18.
//

#ifndef ROS_ENVIRONMENT_VISUALSERVOCONTROLLER_H
#define ROS_ENVIRONMENT_VISUALSERVOCONTROLLER_H

#include <Eigen/Dense>
#include <Eigen/SVD>

#pragma once

class VisualServoController {

public:
    /**
     * Initializer for the controller
     * @param Kp
     * @param target_points_in_image_frame
     */
    VisualServoController(
            const double Kp,
            const Eigen::MatrixXd& target_points_in_image_frame
    );

    /*
     * Blank initializer
     */
    VisualServoController();

	void setKp( const double Kp);
	
	void setTarget( const Eigen::MatrixXd& target_points_in_image_frame );

    /**
     * return the control value
     */
    Eigen::VectorXd control(const Eigen::MatrixXd& input_points);

private:
    double Kp;

    // target points
    Eigen::MatrixXd target_points;

    // size of the points taken
    int n, m;

    // state number
    int ss;
};
#endif //ROS_ENVIRONMENT_VISUALSERVOCONTROLLER_H
