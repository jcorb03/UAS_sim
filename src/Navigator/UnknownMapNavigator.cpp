#include "uas_sim/navigator/UnknownMapNavigator.h"

UnknownMapNavigator::UnknownMapNavigator() {

}

void UnknownMapNavigator::setMissionGoals(std::vector<Waypoint> goals) {
  goals_ = goals;
}

bool UnknownMapNavigator::update(const UAS_state& estimated_state,
  const OccupancyGrid& occupancy_grid,
  bool new_scan) {

  if (!new_scan) {
    return false;
  }


}
