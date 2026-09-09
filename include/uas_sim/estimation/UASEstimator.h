#pragma once
#include "uas_sim/UAS_structs.h"

class UASEstimator {
public:
  UASEstimator(
    const UAS_state& initial_estimate,
    double initial_uncertainty,
    double measurement_noise_stddev
  );

  void update(UAS_measurement& measurement);

  UAS_state get_state_estimate() const;

private:
  UAS_state estimated_state_;
  UAS_measurement measurement_;
  double measurement_noise_stddev_;
  std::vector<std::vector<double>> state_uncertainty_;

};