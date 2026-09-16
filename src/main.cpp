#include <iostream>
#include "uas_sim/UAS_structs.h"
#include "uas_sim/Simulation.h"
#include "uas_sim/data/make_csv.h"

int main() {
  
  UAS_operating_constraints constraints{
      2.0,    // min_speed [m/s]
      5.0,   // max_speed [m/s]
      2.0,    // max_accel [m/s^2]
      0.5     // max_turn_rate [rad/s]
  };


  std::vector<Waypoint> waypoints{
      {100.0, 0.0},
      {100.0, 100.0},
      {0.0, 100.0},
      {0.0, 0.0}
  };

  UAS_state initial_state{
      0.0,    // x [m]
      0.0,    // y [m]
      5.0,   // velocity [m/s]
      0.0     // heading [rad]
  };

  GPSSensor gps_sensor(
    1,    // noise standard deviation [m]
    0.1,     // update rate [Hz]
    5.0
  );

  SimConfig sim_config{
      120.0,    // sim length [s]
      0.1,   // timestep[s]
      10.0     // waypoint tolerance [m]
  };

  UAS uas(
    constraints,
    waypoints,
    initial_state,
    gps_sensor
  );

  std::vector<UAS> Uas_vec = { uas };

  Simulation simulation(
    Uas_vec,
    sim_config
  );

  KalmanFilterState kalman;

  kalman.state_estimate.setZero();

  kalman.P = Eigen::Matrix4d::Identity();

  kalman.A = Eigen::Matrix4d::Identity();

  kalman.Q = Eigen::Matrix4d::Zero();

  kalman.H.setZero();

  kalman.R = Eigen::Matrix2d::Identity() * 4.0;

  kalman.K.setZero();

  uas.initialiseKalman(kalman);

  bool done = simulation.run();

  std::vector<double> time_history = simulation.getTimeHistory();


  std::vector<UAS_state> estimation_history = simulation.getStateEstimateHistory();

  
    bool made = results::makeCsv(time_history, estimation_history);
    std::cout << "CSV status: " << made;
  

  return 0;
}