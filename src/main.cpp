#include <iostream>
#include "uas_sim/UAS_structs.h"
#include "uas_sim/Simulation.h"
#include "uas_sim/data/make_csv.h"
#include "uas_sim/map/ObstacleMap.h"
#include "uas_sim/routeplanner/routeplanner.h"
#include "uas_sim/courses/course1.h"

void SimulationTestRun() {

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

  UAS_state second_initial_state{
      -20.0,  // x [m]
      0.0,    // y [m]
      5.0,    // velocity [m/s]
      0.0     // heading [rad]
  };

  GPSSensor gps_sensor(
    1,    // noise standard deviation [m]
    0.1,     // update rate [Hz]
    5.0
  );

  SimConfig sim_config{
      210.0,    // sim length [s]
      0.1,   // timestep[s]
      10.0     // waypoint tolerance [m]
  };

  UAS uas(
    constraints,
    waypoints,
    initial_state,
    gps_sensor
  );

  UAS second_uas(
    constraints,
    waypoints,
    second_initial_state,
    gps_sensor
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
  second_uas.initialiseKalman(kalman);

  std::vector<UAS> Uas_vec = { uas, second_uas };

  Simulation simulation(
    Uas_vec,
    sim_config
  );

  bool done = simulation.run();

  std::vector<double> time_history = simulation.getTimeHistory();


  std::vector<std::vector<UAS_state>> estimation_history = simulation.getStateEstimateHistory();


  bool made = results::makeCsv(time_history, estimation_history);
  std::cout << "Simulation complete: " << done << "\n"
    << "CSV status: " << made << '\n';
}


int main() {
  
  UAS_operating_constraints constraints{
     2.0,    // min_speed [m/s]
     5.0,   // max_speed [m/s]
     2.0,    // max_accel [m/s^2]
     0.5     // max_turn_rate [rad/s]
  };

  RoutePlanner planner = Courses::Course1();
  std::vector<Waypoint> path = planner.Plan(Waypoint(10.0, 90.0));

  PathFollower guidance(constraints,
    path);

  UAS_state state(10.0, 90.0, 5.0, 3.14);

  guidance.SetMethod(FollowerMethod::PURE_PURSUIT);

  //Simple loop
  double time = 0.0;
  double timestep = 0.1;
  std::vector<std::vector<UAS_state>> state_history = {};
  state_history.resize(1);
  state_history.front().push_back(state);
  std::vector<double> time_history = {time};

  double dist = std::numeric_limits<double>::infinity();
  Waypoint state_coords = { state.x, state.y };

  while (dist > 1.0) {
    UAS_command command = guidance.GetCommand(state, path);

    // Step forward
    state.x += state.v * std::sin(state.heading) * timestep;
    state.y += state.v * std::cos(state.heading) * timestep;

    double heading_error =
      command.heading - state.heading;

    heading_error =
      std::atan2(
        std::sin(heading_error),
        std::cos(heading_error));

    double turn_rate = std::clamp((heading_error) / timestep,
      -constraints.max_turn_rate, (constraints.max_turn_rate));

    state.heading += turn_rate * timestep;

    time += timestep;
    state_history.front().push_back(state);
    time_history.push_back(time);

    state_coords.x = state.x; 
    state_coords.y = state.y;
    dist = helpers::distance(state_coords, path.back());
  };
    
  
  
  bool made = results::makeCsv(time_history, state_history);


  return 0;
}
