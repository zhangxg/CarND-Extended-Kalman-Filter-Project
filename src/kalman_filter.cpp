#include "kalman_filter.h"
#include "tools.h"

#include <iostream>
using namespace std;

using Eigen::MatrixXd;
using Eigen::VectorXd;

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

//void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
//                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
//  x_ = x_in;
//  P_ = P_in;
//  F_ = F_in;
//  H_ = H_in;
//  R_ = R_in;
//  Q_ = Q_in;
//
//// I_ = MatrixXd::Identity(2, 2);
//}

/**
* normalize the PI to -Pi and PI
* */
double normalizePi(double x) {
    x = fmod(x + M_PI, 2*M_PI);
    if (x < 0)
    x += 2*M_PI;
    return x - M_PI;
}

void KalmanFilter::Predict() {
    /**
    * predict the state
    * F: (4x4), is the transition matrix, which is set to fixed value, right now, tuning??
    * x: (4x1), is the state, (px, py, vx, vy)
    * P: (4x4), the covariance matrix, indicates the confidence of the state, the initial value
    *   is given an arbitrary value, needs more tuning???
    * Q: (4x4), the process noise, corresponds to the uncertainty that
    *   you expect in your state equations. calculated from dt and noise_ax/y
    *
    */
    // set u to zero
    x_ = F_ * x_;  // (4x4)*(4x1)=(4x1)
    P_ = F_ * P_ * F_.transpose() + Q_; //(4x4)*(4x4)*(4x4)=(4x4)
}

void KalmanFilter::Update(const VectorXd &z) {
    /**
    * update the state by using Kalman Filter equations
    * H: (2x4), for kalman filter, the laser measurement has no velocity
    *   value, the matrix erases the velocity from the predicted state;
    *
    */
    VectorXd y = z - H_ * x_;             // (2x1)-(2x4)*(4x1) = (2x1)
    MatrixXd Ht = H_.transpose();         // (2x4) -> (4x2)
    MatrixXd S = H_ * P_ * Ht + R_;       // (2x4)*(4x4)*(4x2)+(2x2) = (2x2)
    MatrixXd K = P_ * Ht * S.inverse();   // (4x4)*(4x2)*(2x2) = (4x2)
    x_ += K * y;                          // (4x2)*(2x1) = (4x1)
    P_ = (MatrixXd::Identity(4, 4) - K * H_) * P_;  //(4x4)-(4x2)*(2x4)*(4x4)=(4x4)
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
    /**
    * update the state by using Extended Kalman Filter equations
    */

//    VectorXd y = z - H_ * x_;          // 3x1

    double rho = sqrt(x_(0)*x_(0) + x_(1)*x_(1));
    double theta = atan(x_(1) / x_(0));
    double rho_dot = (x_(0)*x_(2) + x_(1)*x_(3)) / rho;
    VectorXd h = VectorXd(3); // h(x_)
    h << rho, theta, rho_dot;
    VectorXd y = z - h;
    MatrixXd Ht = H_.transpose();      // 4x3
    MatrixXd S = H_ * P_ * Ht + R_;    //(3x4)*(4x4)*(4x3)+(3x3)=(3x3)
    MatrixXd K = P_ * Ht * S.inverse();//(4x4)*(4x3)*(3x3)=(4x3)
    x_ += K * y;                       //(4x3)*(3x1) = (4x1)
    P_ = (MatrixXd::Identity(4, 4) - K * H_) * P_; //((4x4) - (4x3)*(3x4)) = (4x4)
}
