/*
    Created by beck on 25/9/18.
    Reference: https://github.com/hmartiro/kalman-cpp/blob/master/kalman.cpp
*/

#ifndef ROS_ENVIRONMENT_KALMAN_H
#define ROS_ENVIRONMENT_KALMAN_H

#include <Eigen/Dense>

#pragma once

class KalmanFilter {

public:
    /**
     * Initializer, create a Kalman filter with the specified matrices.
     * @param dt, discrete time step
     * @param A , system dynamics matrix
     * @param C , output matrix
     * @param Q , process noise covariance
     * @param R , measurement noise covariance
     * @param P , estimate error covariance
     */
    KalmanFilter(
            double dt,
            const Eigen::MatrixXd& A,
            const Eigen::MatrixXd& H,
            const Eigen::MatrixXd& Q,
            const Eigen::MatrixXd& R,
            const Eigen::MatrixXd& P
    );

    /**
     * Blank initializer
     */
    KalmanFilter();

    /**
     * Initialize the filter with initial states as zero.
     */
    void init();

    /**
     * Initialize the filter with a guess for initial states.
     */
    void init(double t0, const Eigen::VectorXd& x0);

    /**
     * Update the estimated state based on measured values.
     */
    void update(const Eigen::VectorXd& z);

    /**
     * Propagate the Kalman filter by the space state model,
     * The time step is assumed to remain constant
     */
    void propagate();

    /**
     * Propagate the Kalman filter by the space state model,
     * using the given time step and dynamics matrix.
     */
    void propagate(double dt, const Eigen::MatrixXd A);

    /**
     * Calculate the chiSquare test for outlier rejection
     */
    double chiSquare(const Eigen::VectorXd& z);

    /**
     * Return the current state and time.
     */
    Eigen::VectorXd state() { return x; };
    double time() { return t; };

private:

    // Matrices for computation
    Eigen::MatrixXd A, H, Q, R, P, K, P0, S;

    // System dimensions
    int m, n;

    // Initial and current time
    double t0, t;

    // Discrete time step
    double dt;

    bool initialized;

    // Estimated states and residual
    Eigen::VectorXd x, r;

};


#endif //ROS_ENVIRONMENT_KALMAN_H
