
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "../../Eigen/Dense"
#include "../../measurement_package.h"

//using namespace std;

using Eigen::VectorXd;

int main() {

  std::ifstream in("/Users/zhangxg/work/gitrepo/uda-sd/CarND-Extended-Kalman-Filter-Project/data/obj_pose-laser-radar-synthetic-input.txt");
  std::string line;
  std::string type;
  std::vector<MeasurementPackage> measurements;
  float timestamp;
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
  std::cout << measurements.size() << std::endl;
  for (MeasurementPackage m : measurements) {
    std::cout << m.ground_truth_ << "--";
    std::cout << std::endl;
  }
  return 0;
}