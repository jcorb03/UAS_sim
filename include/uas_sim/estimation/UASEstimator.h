#pragma once
#include "uas_sim/UAS_structs.h"
#include <Eigen/Dense>
#include <cmath>
#include <numbers>
#include <iostream>

struct KalmanFilterState {
  // State estimate: [x, y, velocity, heading]
  Eigen::Vector4d state_estimate;

  // State covariance matrix
  Eigen::Matrix4d P;

  // State transition matrix
  Eigen::Matrix4d A;

  // Process noise covariance matrix
  Eigen::Matrix4d Q;

  // Measurement matrix
  Eigen::Matrix<double, 2, 4> H;

  // Measurement noise covariance matrix
  Eigen::Matrix2d R;

  // Kalman gain
  Eigen::Matrix<double, 4, 2> K;
};


class Estimator {
public:
  Estimator();

  void initialiseKalmanProperties(KalmanFilterState kalman);
  void update(UAS_measurement& measurement, double sim_time, bool gps_update,
    double update_period, UAS_operating_constraints operating_constraints_, 
    UAS_command uas_command);

  UAS_state get_state_estimate() const;

private:
  double time_ = 0.0;
  int measurement_count_;
  UAS_state estimated_state_;
  UAS_measurement measurement_;
  double measurement_noise_stddev_;
  KalmanFilterState kalman_;

};