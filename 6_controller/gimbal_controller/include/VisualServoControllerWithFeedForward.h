//
// Created by beck on 20181022
//

#ifndef ROS_ENVIRONMENT_VISUALSERVOCONTROLLER_WITH_FEEDFORWARD_H
#define ROS_ENVIRONMENT_VISUALSERVOCONTROLLER_WITH_FEEDFORWARD_H

#include <Eigen/Dense>
#include <Eigen/SVD>

#pragma once

class VisualServoControllerWithFeedForward {

public:
    /**
     * Initializer for the controller
     * @param Kp
     * @param control_frequency
     * @param target_points_in_image_frame
     */
    VisualServoControllerWithFeedForward(
            const double Kp,
            const double control_frequency,
            const Eigen::MatrixXd& target_points_in_image_frame
    );

    /*
     * Blank initializer
     */
    VisualServoControllerWithFeedForward();

	void setKp( const double Kp);

    /**
     * Enable the velocity feedforward
     */
    void setKd( const double Kd);

    void setControlFrequency( const double control_frequency);
	
	void setTarget( const Eigen::MatrixXd& target_points_in_image_frame );

    /**
     * Taking the features input
     * @param input_points
     */
    void updateFeatures(const Eigen::MatrixXd &input_points);

    void updateOmega(const Eigen::MatrixXd &omega);

    /**
     * @return the estimated target movement considering the body movement
     */
    Eigen::MatrixXd estimatePartialError();

    /**
     * @return the control value, type III, half of both pseudo inverse
     */
    Eigen::VectorXd control();

private:
    double Kp;

    double Kd;

    double ctrl_freq;

    // size of the points taken
    int n, m;

    // state number
    int ss;

    // target points
    Eigen::MatrixXd target_points;

    // an estimate of the interaction matrix
    Eigen::MatrixXd Le_hat;

    // an estimate of the pesudo-inverse of the interaction matrix
    Eigen::MatrixXd Le_hat_inverse;

    // current visual error
    Eigen::MatrixXd error;

    // previous visual error
    Eigen::MatrixXd error_prev;

    // time difference between frames
    Eigen::MatrixXd dot_error;

    // estimated feature error considering the angular velocity
    Eigen::MatrixXd estimated_error_partial;

    // body angular velocity
    Eigen::MatrixXd angular_velocity;

    // received one visual feature
    bool error_initialized;

    bool omega_initialized;

};
#endif //ROS_ENVIRONMENT_VISUALSERVOCONTROLLER_WITH_FEEDFORWARD_H
