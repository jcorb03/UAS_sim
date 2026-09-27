#include <iostream>
#include "uas_sim/UAS_structs.h"
#include "uas_sim/Simulation.h"
#include "uas_sim/data/make_csv.h"
#include "uas_sim/map/ObstacleMap.h"
#include "uas_sim/routeplanner/routeplanner.h"
#include "uas_sim/courses/course1.h"

int main() {
  
  UAS_operating_constraints constraints{
     2.0,    // min_speed [m/s]
     5.0,   // max_speed [m/s]
     2.0,    // max_accel [m/s^2]
     0.5     // max_turn_rate [rad/s]
  };

  RoutePlanner planner = Courses::Course1();


  UAS_state state(10.0, 90.0, 5.0, 3.14);

  GPSSensor gps_sensor(
    1,    // noise standard deviation [m]
    0.1,     // update rate [Hz]
    5.0
  );

  SimConfig sim_config{
      190.0,    // sim length [s]
      0.1,   // timestep[s]
      10.0     // waypoint tolerance [m]
  };

  std::vector<Waypoint> checkpoints{
      Waypoint(5.0, 5.0),
      Waypoint(60.0, 35.0),
      Waypoint(80.0, 65.0),
      Waypoint(40.0, 90.0),
  };

  UAS uas(
    constraints,
    checkpoints,
    state,
    gps_sensor,
    FollowerMethod::PURE_PURSUIT,
    planner
  );

  KalmanFilterState kalman;

  uas.initialiseKalman(kalman);

  std::vector<UAS> Uas_vec = { uas };

  Simulation simulation(
    Uas_vec,
    sim_config
  );

  bool done = simulation.run();

  bool made = results::makeCsv(
    simulation.getTimeHistory(), 
    simulation.getStateEstimateHistory());
  std::cout << "Simulation complete: " << done << "\n"
    << "CSV status: " << made << '\n';

  return 0;
}
