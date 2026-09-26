#include <iostream>
#include "uas_sim/UAS_structs.h"
#include "uas_sim/Simulation.h"
#include "uas_sim/data/make_csv.h"
#include "uas_sim/map/ObstacleMap.h"
#include "uas_sim/routeplanner/routeplanner.h"

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
  
  WorldBounds bounds{ 0.0,0.0,100.0,100.0 };

  CircleObstacle circle1{ 10.0, 15.0, 3.0 };
  CircleObstacle circle2{ 20.0, 75.0, 4.0 };
  CircleObstacle circle3{ 50.0, 50.0, 5.0 };
  std::vector<CircleObstacle> circles = { circle1, circle2, circle3 };


  RectangleObstacle rectangle1{ 20.0, 20.0, 22.0, 23.0 };
  RectangleObstacle rectangle2{ 65.0, 40.0, 80.0, 48.0 };
  std::vector<RectangleObstacle> rectangles = { rectangle1, rectangle2};

  ObstacleMap map;
  map.bounds = bounds;
  map.circles = circles;
  map.rectangles = rectangles;

  map.writeToCSV();

  std::vector<Waypoint> checkpoints{
    Waypoint(10.0, 10.0),
    Waypoint(60.0, 35.0),
    Waypoint(80.0, 65.0),
    Waypoint(20.0, 90.0),
  };

  RoutePlanner planner(checkpoints, map);
  planner.SetPlanningAlgorithm(RoutePlanningAlgos::RRT_STAR);
  planner.Plan(Waypoint(10.0, 90.0));
  
  return 0;
}
