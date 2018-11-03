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
        const Eigen::MatrixXd& P0)
    : A(A), H(H), Q(Q), R(R), P0(P0),
      m(H.rows()), n(A.rows()), dt(dt), initialized(false),
      x(n)
{
    init();
}

KalmanFilter::KalmanFilter() {}

void
KalmanFilter::setMatrices(
        double dt,
        const Eigen::MatrixXd& A,
        const Eigen::MatrixXd& H,
        const Eigen::MatrixXd& Q,
        const Eigen::MatrixXd& R,
        const Eigen::MatrixXd& P0
)
{
    this->dt= dt;
    this->A = A;
    this->H = H;
    this->Q = Q;
    this->R = R;
    this->P0= P0;
    m = H.rows();
    n = A.rows();
}

void
KalmanFilter::setR(const Eigen::MatrixXd& R)
{
    if (R.rows() != m || R.cols() != m)
        throw std::runtime_error("R has different size");

    this->R = R;
}

void
KalmanFilter::setQ(const Eigen::MatrixXd& Q)
{
    if (Q.rows() != n || Q.cols() != n)
        throw std::runtime_error("Q has different size");

    this->Q = Q;
}


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
    x.resize(n, 1);
    x.setZero();
    P = P0;
    t0 = 0;
    t = 0;
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

    if (z.rows() != m)
        throw std::runtime_error("input size does not match.");
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
