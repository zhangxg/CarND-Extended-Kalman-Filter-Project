//
// Created by zhangxg on 17/6/17.
//

#include "tools.h"
#include "Eigen/Dense"
#include "iostream"
#include "kalman_filter.h"
#include "FileLoader.h"
#include "FusionEKF.h"

using namespace std;

int main() {
    vector<MeasurementPackage> measurement_pack_list;
    FileLoader fileLoader = FileLoader("../data/obj_pose-laser-radar-synthetic-input.txt");
    measurement_pack_list = fileLoader.loadData();

    size_t N = measurement_pack_list.size();
    cout << N << endl;

    FusionEKF fusionEKF;

    for (size_t k = 0; k < N; ++k) {	//start filtering from the second frame (the speed is unknown in the first frame)
        cout << k << "====" << endl;
//        cout << measurement_pack_list[k].ground_truth_ << endl;
//        fusionEKF.ProcessMeasurement(measurement_pack_list[k]);
    }
    return 0;
}

////    vector<VectorXd> estimations, ground_truths;
////
////    VectorXd e(4), g(4);
////    e << (0, 0, 0, 0);
////    g << (1, 1, 1, 1);
////    estimations.push_back(e);
////    estimations.push_back(e);
////    estimations.push_back(e);
////
////    ground_truths.push_back(g);
////    ground_truths.push_back(g);
////    ground_truths.push_back(g);
//
//vector<VectorXd> estimations;
//vector<VectorXd> ground_truth;
//
////the input list of estimations
//VectorXd e(4);
//e << 1, 1, 0.2, 0.1;
//estimations.push_back(e);
//e << 2, 2, 0.3, 0.2;
//estimations.push_back(e);
//e << 3, 3, 0.4, 0.3;
//estimations.push_back(e);
//
////the corresponding list of ground truth values
//VectorXd g(4);
//g << 1.1, 1.1, 0.3, 0.2;
//ground_truth.push_back(g);
//g << 2.1, 2.1, 0.4, 0.3;
//ground_truth.push_back(g);
//g << 3.1, 3.1, 0.5, 0.4;
//ground_truth.push_back(g);
//
//Tools tools;
//VectorXd rmse = tools.CalculateRMSE(estimations, ground_truth);
//cout << rmse << endl;
//
//VectorXd x_predicted(4);
//x_predicted << 1, 2, 0.2, 0.4;
//MatrixXd Hj = tools.CalculateJacobian(x_predicted);
//cout << Hj << endl;
//
//
////kalman filter
//cout << "== Kalman filter== " << endl;
//KalmanFilter filter;
////design the KF with 1D motion
//VectorXd x = VectorXd(2);
//x << 0, 0;
//filter.x_ = x;
//
//MatrixXd P = MatrixXd(2, 2);
//P << 1000, 0, 0, 1000;
//filter.P_ = P;
//
//VectorXd u = VectorXd(2);
//u << 0, 0;
////    filter.u
//
//MatrixXd F = MatrixXd(2, 2);
//F << 1, 1, 0, 1;
//filter.F_ = F;
//
//MatrixXd H = MatrixXd(1, 2);
//H << 1, 0;
//filter.H_ = H;
//
//MatrixXd R = MatrixXd(1, 1);
//R << 1;
//filter.R_ = R;
//
////    I = MatrixXd::Identity(2, 2);
//
//MatrixXd Q = MatrixXd(2, 2);
//Q << 0, 0, 0, 0;
//filter.Q_ = Q;
//
////create a list of measurements
//vector<VectorXd> measurements;
//VectorXd single_meas(1);
//single_meas << 1;
//measurements.push_back(single_meas);
//single_meas << 2;
//measurements.push_back(single_meas);
//single_meas << 3;
//measurements.push_back(single_meas);
//
//for (unsigned int i=0; i < measurements.size(); ++i) {
//filter.Update(measurements[i]);
//filter.Predict();
//std::cout << "x=" << std::endl <<  filter.x_ << std::endl;
//std::cout << "P=" << std::endl <<  filter.P_ << std::endl;
//}
