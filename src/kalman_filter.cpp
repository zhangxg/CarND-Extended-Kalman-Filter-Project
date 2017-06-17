#include "kalman_filter.h"
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;

// I_ = MatrixXd::Identity(2, 2);
}

void KalmanFilter::Predict() {
  /**
    * predict the state
  */

  x_ = F_ * x_;  //fixme, needs the u
  P_ = F_ * P_ * F_.transpose() + Q_;

}

void KalmanFilter::Update(const VectorXd &z) {
  /**
    * update the state by using Kalman Filter equations
  */
  VectorXd y = z - H_ * x_;             //
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd K = P_ * Ht * S.inverse();
  x_ += K * y;
//  long dim = z.size();
  P_ = (MatrixXd::Identity(2, 2) - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
    * update the state by using Extended Kalman Filter equations
  */
  // 1. calculate the Jacobian, a (3x4) matrix
  Tools tools;
//  MatrixXd J = tools.CalculateJacobian(x_); //x is vector of 4, the state
  H_ = tools.CalculateJacobian(x_);  // 3x4
  VectorXd y = z - H_ * x_;          // 3x1
  MatrixXd Ht = H_.transpose();      // 4x3
  MatrixXd S = H_ * P_ * Ht + R_;    //(3x4)*(4x4)*(4x3)=(3x3)
  MatrixXd K = P_ * Ht * S.inverse();//(4x4)*(4x3)*(3x3)=(4x3)
  x_ += K * y;                       //(4x3)*(3x1) = (4x1)
//  unsigned  int dim = z.size();
  P_ = (MatrixXd::Identity(4, 4) - K * H_) * P_; //((4x4) - (4x3)*(3x4)) = (4x4)
}
