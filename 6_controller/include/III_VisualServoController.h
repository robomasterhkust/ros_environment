//
// Created by beck on 20181029
//

#ifndef ROS_ENVIRONMENT_VISUALSERVOCONTROLLER_III_H
#define ROS_ENVIRONMENT_VISUALSERVOCONTROLLER_III_H

#include <queue>
#include <Eigen/Dense>
#include <Eigen/SVD>
#include "kalman.h"

#pragma once

class VisualServoController {

public:
    /**
     * Initializer for the controller
     */
    VisualServoController(
            const double Kp,
            const double Kd,
            const double Z,
            const double control_frequency,
            const double target_Z,
            const Eigen::MatrixXd &target_points_in_image_frame
    );

    /**
     * Blank initializer
     */
    VisualServoController();
    
    void resetFlag();

    void setKp( const double Kp);

    void setKd( const double Kd);

    void setZ(  const double Z);

    void setControlFrequency( const double control_frequency);
	
    void setTarget( const Eigen::MatrixXd& target_points_in_image_frame );

    void setTargetZ( const double target_Z );

    /**
     * Taking the features input
     * @param input_points
     */
    void updateFeatures(const Eigen::MatrixXd &input_points);

    void updateOmega(const Eigen::MatrixXd &omega);

    /**
     * Public function to initialize the Kalman filter for visual velocity
     * @param kf_r0
     * @param kf_dt
     */
    void initKalmanFilter(double kf_r0, double kf_q0, double kf_dt);

    void setKalmanR(double kf_r0);

    void setKalmanQ(double kf_q0);

    Eigen::VectorXd getRawVisualOmega(){ return raw_visual_omega; }

    Eigen::VectorXd getDelayedGyro(){ return omega_gyro; }

    Eigen::VectorXd getKalmanInput(){ return omega_hat_target; }

    Eigen::VectorXd getKalmanOutput(){ return estimated_visual_omega; }

    /**
     * @return the control value, type III, half of both pseudo inverse
     */
    Eigen::VectorXd control();

    /**
     * finite state machine controlled by ros node
     * {idle = 0, once = 1, multi = 2};
     */
    int finite_state;

private:
    double Kp;

    double Kd;

    // Estimated distance
    double Z;

    double target_Z;

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
    Eigen::MatrixXd omega_gyro;

    // the estimated feedforward angular velocity, visual - gimbal
    Eigen::VectorXd omega_hat_target;

    // the raw visual angular velocity \in R^3
    Eigen::VectorXd raw_visual_omega;

    // the estimated visual angular velocity
    Eigen::VectorXd estimated_visual_omega;

    bool omega_initialized;

    // the optimal filter for the estimated angular_velocity feedforward
    KalmanFilter kf;

    bool kalman_initialized;

    void printDebugging();

    void runFiniteStateMachine();

    /**
     * @return the estimated target movement considering the body movement
     */
    void runKalman();

    void timestamp_sync(const Eigen::MatrixXd &omega);

    std::queue<Eigen::Matrix<double, 3, 1>> gyro_queue;
};
#endif //ROS_ENVIRONMENT_VISUALSERVOCONTROLLER_III_H
