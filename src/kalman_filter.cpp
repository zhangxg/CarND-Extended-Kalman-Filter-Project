#include "kalman_filter.h"
#include "tools.h"

#include <iostream>
using namespace std;

using Eigen::MatrixXd;
using Eigen::VectorXd;

// Please note that the Eigen library does not initialize 
// VectorXd or MatrixXd objects with zeros upon creation.

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Predict() {
    /**
    * predict the state
    * F: (4x4), is the transition matrix, is determined by the motion model, 
    * x: (4x1), is the state, (px, py, vx, vy)
    * P: (4x4), the covariance matrix, indicates the confidence of the state, 
    *           is given an arbitrary value during initialization.
    * Q: (4x4), the process noise, corresponds to the uncertainty that
    *           you expect in your state equations. 
    *           calculated from dt and noise_ax/y
    */
    // set u to zero
    x_ = F_ * x_;  // (4x4)*(4x1)=(4x1)
    P_ = F_ * P_ * F_.transpose() + Q_; //(4x4)*(4x4)*(4x4)=(4x4)
}

void KalmanFilter::Update(const VectorXd &z) {
    /**
    * state update, for Laser
    * y: measurement residual; 
    * z: the observation, vector containing the states;
    * H: projection matrix, converting from the state space to observaton space;
    * x: the state;
    * P: the covariance of the process noise;
    * R: measurement noise covirance matrix;
    * K: kalman filter gain;
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
    * state update, for Radar
    */
    // first: convert the previous state from state space to observation space
    // 
    double rho = sqrt(x_(0)*x_(0) + x_(1)*x_(1));
    float theta = atan2(x_(1), x_(0));  
    double rho_dot = (x_(0)*x_(2) + x_(1)*x_(3)) / rho;
    VectorXd h = VectorXd(3); 
    h << rho, theta, rho_dot;

    VectorXd y = z - h;
    // for angle normalization, https://discussions.udacity.com/t/ekf-gets-off-track/276122/25
    y[1] = atan2(sin(y[1]), cos(y[1]));  
    MatrixXd Ht = H_.transpose();      // 4x3
    MatrixXd S = H_ * P_ * Ht + R_;    //(3x4)*(4x4)*(4x3)+(3x3)=(3x3)
    MatrixXd K = P_ * Ht * S.inverse();//(4x4)*(4x3)*(3x3)=(4x3)
    x_ += K * y;                       //(4x3)*(3x1) = (4x1)
    P_ = (MatrixXd::Identity(4, 4) - K * H_) * P_; //((4x4) - (4x3)*(3x4)) = (4x4)
}
