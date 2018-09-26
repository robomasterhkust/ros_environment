//
// Created by beck on 25/9/18.
//

#include <iostream>
#include <stdexcept>

#include "kalman.h"

KalmanFilter::KalmanFilter(
        double dt,
        const Eigen::MatrixXd& A,
        const Eigen::MatrixXd& H,
        const Eigen::MatrixXd& Q,
        const Eigen::MatrixXd& R,
        const Eigen::MatrixXd& P)
    : A(A), H(H), Q(Q), R(R), P0(P),
      m(H.rows()), n(A.rows()), dt(dt), initialized(false),
      x(n)
{

}

KalmanFilter::KalmanFilter() {}

void
KalmanFilter::init(double t0, const Eigen::VectorXd& x0)
{
    x = x0;
    P = P0;
    this->t0 = t0;
    t = t0;
    initialized = true;
}

void
KalmanFilter::init()
{
    x.setZero();
    P = P0;
    t0 = 0;
    t = t0;
    initialized = true;
}

void
KalmanFilter::propagate()
{
    if (!initialized)
        throw std::runtime_error("Filter is not initialized");

    x = A * x;
    P = A * P * A.transpose() + Q;
}

void
KalmanFilter::propagate(double dt, const Eigen::MatrixXd A)
{
    this->A = A;
    this->dt = dt;
    propagate();
}

void
KalmanFilter::update(const Eigen::VectorXd& z)
{
    if (!initialized)
        throw std::runtime_error("Filter is not initialized");

    r = z - H * x;
    S = H * P * H.transpose() + R;
    K = P * H.transpose() * S.inverse();

    x = x + K * r;
    P = P - K * H * P;
}

double
KalmanFilter::chiSquare(const Eigen::VectorXd& z)
{
    if (!initialized)
        throw std::runtime_error("Filter is not initialized");
    r = z - H * x;
    S = H * P * H.transpose() + R;
    return r.transpose() * S * r;
}