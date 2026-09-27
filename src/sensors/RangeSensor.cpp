#include "uas_sim/sensors/RangeSensor.h"
#include <cmath>
#include "uas_sim/map/ObstacleMap.h"
RangeSensor::RangeSensor(RangeSensorConfig config) : config_(config) {

}

void RangeSensor::getSurroundings(
  const UAS_state& state,
  const ObstacleMap& map) {

  measurement_.clear();
  Waypoint point;

  for (double bearing = state.heading - config_.field_of_view;
    bearing <= state.heading + config_.field_of_view;
    bearing += config_.angular_resolution) {

    const double wrapped_bearing =
      std::atan2(std::sin(bearing), std::cos(bearing));

    double measured_range =
      std::numeric_limits<double>::infinity();

    for (double range = 0.0;
      range < config_.max_range;
      range += config_.range_resolution) {

      point.x = state.x + range * std::sin(wrapped_bearing);
      point.y = state.y + range * std::cos(wrapped_bearing);

      if (!map.isPointFree(point)) {
        measured_range = range;
        break;
      }
    }

    measurement_.push_back(
      RangeMeasurement{
          measured_range,
          wrapped_bearing
      });
  }
}


std::vector<RangeMeasurement> RangeSensor::getMeasurement() const{
  return measurement_;
}