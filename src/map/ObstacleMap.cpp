#include "uas_sim/map/ObstacleMap.h"

bool ObstacleMap::isPointFree(const Waypoint& to) const{

  for (const CircleObstacle& circle : circles) {
    bool hit_circle = std::sqrt((to.x - circle.x) * (to.x - circle.x)
      + (to.y - circle.y) * (to.y - circle.y)) < circle.radius;
    if (hit_circle) {
      return false;
    }
  }

  for (const RectangleObstacle& obstacle : rectangles) {
    bool hit_rec = (to.x > obstacle.min_x && to.y > obstacle.min_y &&
      to.x < obstacle.max_x && to.y < obstacle.max_y);
    if (hit_rec) {
      return false;
    }
  }
  return true;
}

bool ObstacleMap::isSegmentFree(const Waypoint& from, const Waypoint& to) const {

  int N = 100;
  Waypoint point = from;
  double dx = (to.x - from.x);
  double dy = (to.y - from.y);

  for (int i = 1; i < N; ++i) {

    point.x = from.x + (i / N) * dx;
    point.y = from.y + (i / N) * dy;

    if (!isPointFree(point)) {
      return false;
    }
  }
  return true;
}