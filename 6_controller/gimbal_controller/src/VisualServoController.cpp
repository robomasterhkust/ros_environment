/**
 * Beck Pang, 20180926, trying to use the four points directly to build a robuster controller
 * Reference: Chaumette, François, and Seth Hutchinson. "Visual servo control. I. Basic approaches."
 *
 * core controller,
 * (wz) = - Kp * 0.5 * (Le + Le*)^-1 (x, y)
 * Le_i = [y, -x]'
 * Le   = [Le1, Le2, Le3, Le4]'
 */

#include <iostream>
#include <stdexcept>

#include "VisualServoController.h"

VisualServoController::VisualServoController(
        const double Kp,
        const Eigen::MatrixXd& target_points_in_image_frame)
        : Kp(Kp), target_points(target_points_in_image_frame),
          n(target_points_in_image_frame.rows()), m(target_points_in_image_frame.cols())
{
    ss = 1;
}

VisualServoController::VisualServoController()
{
    ss = 1;
}

Eigen::VectorXd 
VisualServoController::control(const Eigen::MatrixXd& input_points)
{
    if (input_points.rows() != n)
        throw std::runtime_error("points not the same as the target");

    Eigen::MatrixXd Le(n * m, ss);
    Eigen::MatrixXd Le_star(n * m, ss);

    for (int i = 0; i < n; ++i) {
        Le(m * i, 0)     = input_points(i, 0);
        Le(m * i + 1, 0) = input_points(i, 1);
    }
    for (int i = 0; i < n; ++i) {
        Le_star(m * i, 0)     = target_points(i, 0);
        Le_star(m * i + 1, 0) = target_points(i, 1);
    }
    Eigen::MatrixXd Le_sum = Le + Le_star;

 

    /*
     * TODO: what is the error?
     * s:   a set of features that are available in the image data
     * s*:  constant
     * e = s - s*;
     * K is the form: 8 x 1 matrix
     */
    // take Moore-Penrose pseudo-inverse, A = U Z V', A+ = V Z' U'
    Eigen::JacobiSVD<Eigen::MatrixXd> Le_svd(Le_sum, Eigen::ComputeThinU | Eigen::ComputeThinV);
    
    int rank = Le_svd.singularValues().size();
    double pinvtoler=1.e-6;
    Eigen::VectorXd singularValueInv(rank);
    
    for (int i=0; i < rank; ++i) {
        double svd_val = Le_svd.singularValues()(i);
        if (svd_val > pinvtoler)
            singularValueInv(i) = 1 / svd_val;
        else
            singularValueInv(i) = 0;
    }
    
    Eigen::MatrixXd K = - Kp * 0.5 * Le_svd.matrixV() * singularValueInv.asDiagonal() * Le_svd.matrixU().transpose();
    std::cout << "K is the form of " << K << std::endl;
    Eigen::Vector3d empty(3);

    return empty;
}
