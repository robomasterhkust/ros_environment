/**
 * Beck Pang, 20180926, trying to use the four points directly to build a robuster controller
 * Reference: Chaumette, François, and Seth Hutchinson. "Visual servo control. I. Basic approaches.
 */

#include <iostream>
#include <stdexcept>

#include "VisualServoController.h"

VisualServoController::VisualServoController(
        const double Kp,
        const Eigen::MatrixXd &target_points_in_image_frame)
        : Kp(Kp), target_points(target_points_in_image_frame),
          n(target_points_in_image_frame.rows()), m(target_points_in_image_frame.cols())
{
    ss = 3;
}

VisualServoController::VisualServoController()
{
    ss = 3;
}

void
VisualServoController::setKp(
		const double Kp)
		
{
    this->Kp = Kp;
} 

void
VisualServoController::setTarget( 
		const Eigen::MatrixXd& target_points_in_image_frame )
{
    this->n = target_points_in_image_frame.rows();
    this->m = target_points_in_image_frame.cols();
    this->target_points = target_points_in_image_frame;
}

/**
 * Core controller, (wz) = - Kp * 0.5 * (Le + Le*)^-1 e
 * 		for pinhole camera only
 * @param input_points
 * @return controller output
 */
Eigen::VectorXd
VisualServoController::control(const Eigen::MatrixXd &input_points)
{
    if (input_points.rows() != n)
        throw std::runtime_error("points not the same as the target");

    int i;
    /**
     * Form the feature Jacobian Le =
     * Le_i = 
     * [xy,  -(1 + x^2),   y]
     * [1 + y^2,    -xy,  -x]
     * Le   = [Le1, Le2, Le3, Le4]'
     *
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
    Eigen::MatrixXd Le_sum = Le + Le_star;

    /**
     * Calculate the gain
     * K is the form: 8 x 1 matrix
     * take Moore-Penrose pseudo-inverse, A = U Z V', A+ = V Z' U'
     */
    Eigen::JacobiSVD <Eigen::MatrixXd> Le_svd(Le_sum, Eigen::ComputeThinU | Eigen::ComputeThinV);

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

    Eigen::MatrixXd K = -Kp * 0.5 * Le_svd.matrixV() * singularValueInv.asDiagonal() * Le_svd.matrixU().transpose();

    /**
     * Calculate the error
     * where error is the feature image frame value - target image frame value
     * s:   a set of features that are available in the image data
     * s*:  constant
     * e = s - s*;
     */
    Eigen::MatrixXd error(n * m, 1);
    for ( i = 0; i < n; ++i) {
        error(m * i, 0) = input_points(i, 0) - target_points(i, 0);
        error(m * i + 1, 0) = input_points(i, 1) - target_points(i, 1);
    }

    Eigen::VectorXd control_val(ss);
    control_val << K * error;
    std::cout << "control_val is " << std::endl << control_val << std::endl;

    return control_val;
}

Eigen::VectorXd
VisualServoController::control_type_current(const Eigen::MatrixXd& input_points)
{
    if (input_points.rows() != n)
        throw std::runtime_error("points not the same as the target");

    int i;
    /**
     * Form the feature Jacobian Le =
     * Le_i = 
     * [xy,  -(1 + x^2),   y]
     * [1 + y^2,    -xy,  -x]
     * Le   = [Le1, Le2, Le3, Le4]'
     *
     */
    Eigen::MatrixXd Le(n *m, ss);

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


    /**
     * Calculate the gain
     * K is the form: 8 x 1 matrix
     * take Moore-Penrose pseudo-inverse, A = U Z V', A+ = V Z' U'
     */
    Eigen::JacobiSVD <Eigen::MatrixXd> Le_svd(Le, Eigen::ComputeThinU | Eigen::ComputeThinV);

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

    Eigen::MatrixXd K = -Kp * 0.5 * Le_svd.matrixV() * singularValueInv.asDiagonal() * Le_svd.matrixU().transpose();

    /**
     * Calculate the error
     * where error is the feature image frame value - target image frame value
     * s:   a set of features that are available in the image data
     * s*:  constant
     * e = s - s*;
     */
    Eigen::MatrixXd error(n * m, 1);
    for ( i = 0; i < n; ++i) {
        error(m * i, 0) = input_points(i, 0) - target_points(i, 0);
        error(m * i + 1, 0) = input_points(i, 1) - target_points(i, 1);
    }

    Eigen::VectorXd control_val(ss);
    control_val << K * error;
    std::cout << "control_val in type current is " << std::endl << control_val << std::endl;

    return control_val;
}

Eigen::VectorXd 
VisualServoController::control_type_target(const Eigen::MatrixXd& input_points)
{
    if (input_points.rows() != n)
        throw std::runtime_error("points not the same as the target");

    int i;
    /**
     * Form the feature Jacobian Le =
     * Le_i = 
     * [xy,  -(1 + x^2),   y]
     * [1 + y^2,    -xy,  -x]
     * Le   = [Le1, Le2, Le3, Le4]'
     *
     */
    Eigen::MatrixXd Le_star(n *m, ss);

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

    /**
     * Calculate the gain
     * K is the form: 8 x 1 matrix
     * take Moore-Penrose pseudo-inverse, A = U Z V', A+ = V Z' U'
     */
    Eigen::JacobiSVD <Eigen::MatrixXd> Le_svd(Le_star, Eigen::ComputeThinU | Eigen::ComputeThinV);

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

    Eigen::MatrixXd K = -Kp * 0.5 * Le_svd.matrixV() * singularValueInv.asDiagonal() * Le_svd.matrixU().transpose();

    /**
     * Calculate the error
     * where error is the feature image frame value - target image frame value
     * s:   a set of features that are available in the image data
     * s*:  constant
     * e = s - s*;
     */
    Eigen::MatrixXd error(n * m, 1);
    for ( i = 0; i < n; ++i) {
        error(m * i, 0) = input_points(i, 0) - target_points(i, 0);
        error(m * i + 1, 0) = input_points(i, 1) - target_points(i, 1);
    }

    Eigen::VectorXd control_val(ss);
    control_val << K * error;
    std::cout << "control_val in type target is " << std::endl << control_val << std::endl;

    return control_val;
}

