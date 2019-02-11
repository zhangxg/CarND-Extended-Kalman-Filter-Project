
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "../../Eigen/Dense"

#include "../../FusionEKF.h"
#include "../../tools.h"
#include "../../measurement_package.h"

using Eigen::VectorXd;

std::vector<MeasurementPackage> readMeasurements() {
  std::ifstream in("/Users/zhangxg/work/gitrepo/uda-sd/CarND-Extended-Kalman-Filter-Project/data/obj_pose-laser-radar-synthetic-input.txt");
  std::string line;
  std::string type;
  std::vector<MeasurementPackage> measurements;
  long long timestamp;
  float x_groundtruth, y_groundtruth, vx_groundtruth, vy_groundtruth;
  float yaw_groundtruth, yawrate_groundtruth;
  float rho_measured, phi_measured, rhodot_measured;
  float x_measured, y_measured;

  while (std::getline(in, line)) {
    std::istringstream iss(line);
    MeasurementPackage meas_package;
    iss >> type;
    if (type == "R") {
      iss >> rho_measured;
      iss >> phi_measured;
      iss >> rhodot_measured;
      meas_package.sensor_type_ = MeasurementPackage::RADAR;
      meas_package.raw_measurements_ = VectorXd(3);
      meas_package.raw_measurements_ << rho_measured, phi_measured, rhodot_measured;
    } else if (type == "L") {
      iss >> x_measured;
      iss >> y_measured;
      meas_package.sensor_type_ = MeasurementPackage::LASER;
      meas_package.raw_measurements_ = VectorXd(2);
      meas_package.raw_measurements_ << x_measured, y_measured;
    } else {
      throw "Found un-recognized sensor type";
    }
    iss >> timestamp;
    iss >> x_groundtruth;
    iss >> y_groundtruth;
    iss >> vx_groundtruth;
    iss >> vy_groundtruth;
    iss >> yaw_groundtruth;
    iss >> yawrate_groundtruth;
    meas_package.timestamp_ = timestamp;
    meas_package.ground_truth_ = VectorXd(6);
    meas_package.ground_truth_ << x_groundtruth, y_groundtruth,
                                  vx_groundtruth, vy_groundtruth,
                                  yaw_groundtruth, yawrate_groundtruth;
    measurements.push_back(meas_package);
  }
  return measurements;
}


int main() {
  std::vector<MeasurementPackage> measurements;
  measurements = readMeasurements();
  FusionEKF fusionEKF;
  Tools tools;
  vector<VectorXd> estimations;
  vector<VectorXd> ground_truths;

  std::cout << measurements.size() << std::endl;
  for (MeasurementPackage m : measurements) {
    fusionEKF.ProcessMeasurement(m);

    VectorXd estimate(4);
    estimate << fusionEKF.ekf_.x_(0), 
                fusionEKF.ekf_.x_(1),
                fusionEKF.ekf_.x_(2),
                fusionEKF.ekf_.x_(3);

    estimations.push_back(estimate);

    VectorXd ground_truth(4);
    ground_truth << m.ground_truth_(0),
                    m.ground_truth_(1), 
                    m.ground_truth_(2), 
                    m.ground_truth_(3);
    ground_truths.push_back(ground_truth);

    VectorXd RMSE = tools.CalculateRMSE(estimations, ground_truths);

    std::cout << RMSE << std::endl;
  }
  std::cout << "Done Calculation." << std::endl;

  return 0;
}