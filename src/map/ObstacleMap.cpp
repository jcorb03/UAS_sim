#include "uas_sim/map/ObstacleMap.h"
#include <filesystem>
#include <fstream>
#include <iostream>

bool ObstacleMap::isPointFree(const Waypoint& to) const{

  for (const CircleObstacle& circle : circles) {
    bool hit_circle = std::sqrt((to.x - circle.x) * (to.x - circle.x)
      + (to.y - circle.y) * (to.y - circle.y)) < (circle.radius+safety_margin);
    if (hit_circle) {
      return false;
    }
  }

  for (const RectangleObstacle& obstacle : rectangles) {
    bool hit_rec = (to.x > obstacle.min_x && to.y > obstacle.min_y &&
      to.x < obstacle.max_x && to.y < obstacle.max_y);
    double dist = distance(to, obstacle);
    if (hit_rec || (dist < safety_margin) ){
      return false;
    }
  }
  return true;
}


double ObstacleMap::distance(
  const Waypoint& to,
  const RectangleObstacle& obstacle) const
{
  const double closest_x =
    std::clamp(
      to.x,
      obstacle.min_x,
      obstacle.max_x);

  const double closest_y =
    std::clamp(
      to.y,
      obstacle.min_y,
      obstacle.max_y);

  const double dx = to.x - closest_x;
  const double dy = to.y - closest_y;

  return std::sqrt(dx * dx + dy * dy);
}

bool ObstacleMap::isSegmentFree(const Waypoint& from, const Waypoint& to) const {

  int N = 100;
  Waypoint point = from;
  double dx = (to.x - from.x);
  double dy = (to.y - from.y);

  for (int i = 1; i < N; ++i) {

    const double fraction = static_cast<double>(i) / static_cast<double>(N);
    point.x = from.x + fraction * dx;
    point.y = from.y + fraction * dy;

    if (!isPointFree(point)) {
      return false;
    }
  }
  return true;
}

void ObstacleMap::writeToCSV() const
{
  const std::filesystem::path circles_path =
    std::filesystem::path{ UAS_SIM_PROJECT_DIR } / "results" "/circles.csv";
  std::ofstream circles_file(circles_path);
  circles_file << "X,Y,Radius\n";

  for (const CircleObstacle& circle : circles)
  {
    circles_file << circle.x << ","
      << circle.y << ","
      << circle.radius << "\n";
  }

  const std::filesystem::path rectangles_path =
    std::filesystem::path{ UAS_SIM_PROJECT_DIR } / "results" "/rectangles.csv";
  std::ofstream rectangles_file(rectangles_path);

  rectangles_file << "MinX,MinY,MaxX,MaxY\n";

  for (const RectangleObstacle& rectangle : rectangles)
  {
    rectangles_file << rectangle.min_x << ","
      << rectangle.min_y << ","
      << rectangle.max_x << ","
      << rectangle.max_y << "\n";
  }

  const std::filesystem::path bounds_path =
    std::filesystem::path{ UAS_SIM_PROJECT_DIR } / "results" "/bounds.csv";
  std::ofstream bounds_file(bounds_path);

  bounds_file << "MinX,MinY,MaxX,MaxY\n";
  bounds_file << bounds.min_x << ","
    << bounds.min_y << ","
    << bounds.max_x << ","
    << bounds.max_y << "\n";
}