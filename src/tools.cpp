#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
    * Calculate the RMSE here.
  */
    VectorXd rmse(4);
//    rmse << (-1, -1, -1, -1);  // incorrect, no bracket
    rmse << 0, 0, 0, 0;

    //check the inputs validation
    if (estimations.size() <= 0 && estimations.size() != ground_truth.size()) {
        cout << "(warning) the input for rmse calculation is incorrect " << endl;
        return rmse;
    }

    // calculate sum
//    VectorXd residual(4);
//    residual << (0, 0, 0, 0);
    for (int i=0; i < estimations.size(); ++i) {
        VectorXd residual = estimations[i] - ground_truth[i];
        residual = residual.array() * residual.array();
        rmse += residual;
    }

    // calculate the suqare root
    rmse = rmse/estimations.size();
    rmse = rmse.array().sqrt();
    return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
  TODO:
    * Calculate a Jacobian here.
  */
}
