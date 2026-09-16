#include "uas_sim/UAS/UAS.h"
#include "uas_sim/UAS_structs.h"

void UAS::initialiseKalman(KalmanFilterState kalman) {
  estimator_.initialiseKalmanProperties(kalman);
}