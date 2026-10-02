#include <iostream>
#include "uas_sim/UAS_structs.h"
#include "uas_sim/Simulation.h"
#include "uas_sim/data/make_csv.h"
#include "uas_sim/map/ObstacleMap.h"
#include "uas_sim/routeplanner/routeplanner.h"
#include "uas_sim/courses/course1.h"
#include "uas_sim/SampleSimulations.h"

int main() {
  
  //Choose Simulation from SimulationCatalogue()

  UAS_operating_constraints constraints{
     1.0,    // min_speed [m/s]
     3.0,    // max_speed [m/s]
     2.0,    // max_accel [m/s^2]
     1.0     // max_turn_rate [rad/s]
  };

  RoutePlanner planner = Courses::Course1();
  UAS_state state(10.0, 90.0, 3.0, 3.14);

  GPSSensor gps_sensor(
    0.25,  // position noise standard deviation [m]
    0.1,   // update period [s]
    10.0   // time until sensor failure [s]
  );

  RangeSensorConfig rsensorconfig{
    15.0, // max range [m]
    1.57, // half angle from direction of movement
    0.02, // angular resolution [rad]
    1.0,  // range resolution [m]
    0.5   // update period [s]
  };

  SimConfig sim_config{
      300.0,    // sim length [s]
      0.1,   // timestep[s]
      5.0      // waypoint tolerance [m]
  };

  std::vector<Waypoint> checkpoints{
      Waypoint(5.0, 5.0),
      Waypoint(25.0,5.0),
      Waypoint(60.0, 35.0),
      Waypoint(80.0, 65.0),
      Waypoint(40.0, 90.0),
  };

  KalmanFilterState kalman;

  kalman.state_estimate.setZero();

  kalman.P = Eigen::Matrix4d::Identity();

  kalman.A = Eigen::Matrix4d::Identity();

  kalman.Q = Eigen::Matrix4d::Identity() * 0.01;

  kalman.H.setZero();

  kalman.R = Eigen::Matrix2d::Identity() * 0.0625;

  kalman.K.setZero();

  WorldBounds bounds{ 0.0, 0.0, 100.0, 100.0 };

  SimulationConfig config{
    constraints,
    state,
    gps_sensor,
    rsensorconfig,
    sim_config,
    checkpoints,
    FollowerMethod::PURE_PURSUIT,
    planner,
    kalman,
    NavigationMode::UnknownMap,
    1.0
  };

  Simulation simulation(config);

  bool done = simulation.run();

  std::vector<double> time_history = simulation.getTimeHistory();


  std::vector<std::vector<UAS_state>> estimation_history = simulation.getStateEstimateHistory();


  bool made = results::makeCsv(time_history, estimation_history);
  std::cout << "Simulation complete: " << done << "\n"
    << "CSV status: " << made << '\n';
  
  OccupancyGrid occupancyGrid = simulation.getOccupancyGrid();

  return 0;
}
