#pragma once
#include "uas_sim/UAS_structs.h"
#include <vector>

struct KalmanFilterState {
  // State estimate
  // x_hat
  std::vector<double> state_estimate;

  // State covariance
  // P
  std::vector<std::vector<double>> covariance;

  // State transition matrix
  // F
  std::vector<std::vector<double>> state_transition;

  // Process noise covariance
  // Q
  std::vector<std::vector<double>> process_noise;

  // Measurement matrix
  // H
  std::vector<std::vector<double>> measurement_matrix;

  // Measurement noise covariance
  // R
  std::vector<std::vector<double>> measurement_noise;

  // Kalman gain
  // K
  std::vector<std::vector<double>> kalman_gain;
};


class Estimator {
public:
  Estimator();

  void initialiseKalmanProperties(KalmanFilterState kalman);
  void update(UAS_measurement& measurement, double sim_time);

  UAS_state get_state_estimate() const;

private:
  int time_ = 0.0;
  int measurement_count_;
  UAS_state estimated_state_;
  UAS_measurement measurement_;
  double measurement_noise_stddev_;
  KalmanFilterState kalman_;

};