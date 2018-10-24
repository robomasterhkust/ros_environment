/**
 * Beck Pang, 20181022, add the perspective controller with the target velocity feedforward
 * Reference:
 *  Chaumette, François, and Seth Hutchinson. "Visual servo control. I. Basic approaches.
 *  Chaumette, François, and Seth Hutchinson. "Visual servo control. II. Advanced approaches.
 */

#include <iostream>
#include <stdexcept>

#include "VisualServoControllerWithFeedForward.h"

VisualServoControllerWithFeedForward::VisualServoControllerWithFeedForward(
        const double Kp,
        const double control_frequency,
        const Eigen::MatrixXd &target_points_in_image_frame)
        : Kp(Kp), ctrl_freq(control_frequency),
          n(target_points_in_image_frame.rows()), m(target_points_in_image_frame.cols()),
          target_points(target_points_in_image_frame)
{
    Kd = 0;
    ss = 3;
    error_initialized = false;
    omega_initialized = false;
    kalman_initialized= false;

    double kf_r0 = 0.01;
    if (ctrl_freq != 0)
        initKalmanFilter( kf_r0, 1 / ctrl_freq);
}

VisualServoControllerWithFeedForward::VisualServoControllerWithFeedForward()
{
    Kp = 1;
    Kd = 0;
    ss = 3;
    ctrl_freq = 0;
    error_initialized = false;
    omega_initialized = false;
    kalman_initialized= false;
}

void
VisualServoControllerWithFeedForward::setKp(
		const double Kp)
{
    this->Kp = Kp;
}

void
VisualServoControllerWithFeedForward::setKd(
        const double Kd)
{
    this->Kd = Kd;
}

void
VisualServoControllerWithFeedForward::setControlFrequency(
        const double control_frequency)
{
    this->ctrl_freq = control_frequency;
}

void
VisualServoControllerWithFeedForward::setTarget( 
		const Eigen::MatrixXd& target_points_in_image_frame )
{
    this->n = target_points_in_image_frame.rows();
    this->m = target_points_in_image_frame.cols();
    this->target_points = target_points_in_image_frame;
}

/*
Eigen::VectorXd
VisualServoControllerWithFeedForward::getRawVisualOmega()
{
    return raw_visual_omega;
}
 */

/**
 * Initialize the Kalman filter for the omega
 */
void
VisualServoControllerWithFeedForward::initKalmanFilter(double kf_r0, double kf_dt)
{
    const int nn = 2 * this->n;
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(nn, nn);
    Eigen::MatrixXd O = Eigen::MatrixXd::Zero(nn, nn);
    Eigen::MatrixXd A(2 * nn, 2 * nn);
    Eigen::MatrixXd H(1 * nn, 2 * nn);
    Eigen::MatrixXd Q(2 * nn, 2 * nn);
    Eigen::MatrixXd R(1 * nn, 1 * nn);
    Eigen::MatrixXd P0(2 *nn, 2 * nn);

    A << I, I * kf_dt,
         O, I;
    H << I, O;
    Q << I, O,
         O, I;
    R << I * kf_r0;
    P0<< I, O,
         O, I;
    kf.setMatrices(kf_dt, A, H, Q, R, P0);
    kf.init();
    kalman_initialized = true;
    std::cout << "Kalman filter initialized with A size " << 2 * nn << std::endl;
}

void
VisualServoControllerWithFeedForward::setKalmanR(double kf_r0)
{
    if (!kalman_initialized)
        throw std::runtime_error("Kalman filter not initialized.");

    const int nn = 2 * this->n;
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(nn, nn);

    kf.setR(I * kf_r0);
}

/**
 * Update the interaction matrix, its inverse, and error
 * @param input_points \in R^8
 */
void
VisualServoControllerWithFeedForward::updateFeatures(const Eigen::MatrixXd &input_points)
{
    if (input_points.rows() != n)
        throw std::runtime_error("points not the same as the target.");

    int i;
    /**
     * Form the feature Jacobian Le =
     * Le_i =
     * [xy,  -(1 + x^2),   y]
     * [1 + y^2,    -xy,  -x]
     * Le   = [Le1, Le2, Le3, Le4]'
     */
    Eigen::MatrixXd Le(n *m, ss);
    Eigen::MatrixXd Le_star(n *m, ss);

    for ( i = 0; i < n; ++i) {
        int x = input_points(i, 0);
        int y = input_points(i, 1);
        Le(m * i, 0) = x * y;
        Le(m * i, 1) = -(1 + x * x);
        Le(m * i, 2) = y;
        Le(m * i + 1, 0) = 1 + y * y;
        Le(m * i + 1, 1) = -x * y;
        Le(m * i + 1, 2) = -x;
    }
    for ( i = 0; i < n; ++i) {
        int x = target_points(i, 0);
        int y = target_points(i, 1);
        Le_star(m * i, 0) = x * y;
        Le_star(m * i, 1) = -(1 + x * x);
        Le_star(m * i, 2) = y;
        Le_star(m * i + 1, 0) = 1 + y * y;
        Le_star(m * i + 1, 1) = -x * y;
        Le_star(m * i + 1, 2) = -x;
    }
    Le_hat = 0.5 * (Le + Le_star);


    /**
     * Calculate the Le_inverse
     * K is the form: 8 x 1 matrix
     * take Moore-Penrose pseudo-inverse, A = U Z V', A+ = V Z' U'
     */
    Eigen::JacobiSVD <Eigen::MatrixXd> Le_svd(Le_hat, Eigen::ComputeThinU | Eigen::ComputeThinV);

    int rank = Le_svd.singularValues().size();
    double pinvtoler = 1.e-6;
    Eigen::VectorXd singularValueInv(rank);

    for (int i = 0; i < rank; ++i) {
        double svd_val = Le_svd.singularValues()(i);
        if (svd_val > pinvtoler)
            singularValueInv(i) = 1 / svd_val;
        else
            singularValueInv(i) = 0;
    }

    Le_hat_inverse = Le_svd.matrixV() * singularValueInv.asDiagonal() * Le_svd.matrixU().transpose();

    /**
     * Update the visual error
     */
    error = Eigen::MatrixXd::Zero(n * m, 1);
    for ( i = 0; i < n; ++i) {
        error(m * i, 0) = input_points(i, 0) - target_points(i, 0);
        error(m * i + 1, 0) = input_points(i, 1) - target_points(i, 1);
    }

    if (!error_initialized) {
        error_prev = Eigen::MatrixXd::Zero(n * m, 1);
        dot_error = Eigen::MatrixXd::Zero(n * m, 1);
        error_initialized = true;
    }
    else {
        dot_error = (error - error_prev) * ctrl_freq;
        error_prev = error;
    }
}

/**
 * Update the body frame angular velocity w in camera frame
 * @param omega angular velocity in camera frame \in R^3
 */
void
VisualServoControllerWithFeedForward::updateOmega(const Eigen::MatrixXd &omega)
{
    if (omega.rows() != ss)
        throw std::runtime_error("angular velocity dimension error.");

    angular_velocity = omega;
    if (!omega_initialized) {
        omega_initialized = true;
    }
}


/**
 * Kalman filter update to estimate the target angular velocity \in R^3
 */
void
VisualServoControllerWithFeedForward::estimatePartialError()
{
    if (error_initialized && omega_initialized) {
        estimated_error_partial = dot_error - Le_hat * angular_velocity;

        std::cout << "enter kf update with visual" << std::endl << estimated_error_partial << std::endl;
        kf.update(estimated_error_partial);
        std::cout << "end kf update" << std::endl;

        raw_visual_omega = Le_hat_inverse * dot_error;

        // estimated_angular_velocity_ff = Le_hat_inverse * estimated_error_partial;
        estimated_visual_omega = Le_hat_inverse * kf.state().topRows(2 * n);
        /**
         * print the debugging message
         */
        std::cout << "Kalman filter state " << std::endl << kf.state().transpose() << std::endl;
        std::cout << "Kalman filter covariance " << std::endl << kf.covariance().transpose() << std::endl;
//        std::cout << "e(t) is " << std::endl << error.transpose() << std::endl;
//        std::cout << "e(t - dt) is " << std::endl << error_prev.transpose()  << std::endl;
//        std::cout << "e.^ is " << std::endl << dot_error.transpose() << std::endl;
//        std::cout << "^de/dt is " << std::endl << estimated_error_partial.transpose() << std::endl;
//        std::cout << "hat_Le is " << std::endl << Le_hat << std::endl;
//        std::cout << "hat_Le_+ is " << std::endl << Le_hat_inverse << std::endl;
    }
    else {
        estimated_angular_velocity_ff = Eigen::MatrixXd::Zero(ss, 1);
        raw_visual_omega = Eigen::MatrixXd::Zero(ss, 1);
        estimated_visual_omega = Eigen::MatrixXd::Zero(ss, 1);
    }

//    return estimated_visual_omega;
}

/**
 * Core controller
 * @return controller w_target in camera frame
 */
Eigen::VectorXd
VisualServoControllerWithFeedForward::control()
{
    Eigen::VectorXd control_val(ss);

    if (!error_initialized && !omega_initialized) {
        control_val << Eigen::MatrixXd::Zero(ss, 1);
    }
    else if (!error_initialized || !omega_initialized) {
        control_val << -Kp * Le_hat_inverse * error;
    }
    else {
        estimatePartialError();
        control_val << -Kp * Le_hat_inverse * error - Kd * getEstimatedVisualOmega();
    }

//    std::cout << "control_val is " << std::endl << control_val << std::endl;

    return control_val;
}
