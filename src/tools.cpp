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
    rmse << 0, 0, 0, 0;

    //check the inputs validation
    if (estimations.size() <= 0 && estimations.size() != ground_truth.size()) {
        cout << "(warning) the input for rmse calculation is incorrect " << endl;
        return rmse;
    }

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
    * Calculate a Jacobian here.
  */
    MatrixXd Hj(3, 4);
    float px = x_state(0);
    float py = x_state(1);
    float vx = x_state(2);
    float vy = x_state(3);

    float p2 = px*px + py*py;
    //check division by zero
    if(fabs(p2) < 0.0001){
        cout << "Calculate Jacobian () - Error - Division by Zero" << endl;
        return Hj;
    }

    float p1 = sqrt(p2);
    float p3 = p2 * p1;

    float v1 = vx*py - vy*px;
    float v2 = -v1;
    Hj << px/p1, py/p1, 0, 0,
          -py/p2, px/p2, 0, 0,
          py*v1/p3, px*v2/p3, px/p1, py/p1;
    return Hj;
}
