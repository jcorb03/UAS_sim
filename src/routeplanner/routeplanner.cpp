#include "uas_sim\routeplanner\routeplanner.h"
#include <filesystem>
#include <fstream>
#include <iostream>

RoutePlanner::RoutePlanner(std::vector<Waypoint> checkpoints, ObstacleMap obstacle_map): checkpoints_(checkpoints),
obstacle_map_(obstacle_map){

}

void RoutePlanner::SetPlanningAlgorithm(RoutePlanningAlgos algorithm){
  algorithm_ = algorithm;
}

std::vector<Waypoint> RoutePlanner::Plan(const Waypoint& start) {
   
  if (algorithm_ == RoutePlanningAlgos::SIMPLE) {
    return checkpoints_;
  }
  else {
    return plan_RRT_route(start);
  }
}

std::vector<Waypoint> RoutePlanner::plan_RRT_route(Waypoint start)
{
  const WorldBounds& bounds = obstacle_map_.bounds;

  const double dx = bounds.max_x - bounds.min_x;
  const double dy = bounds.max_y - bounds.min_y;

  constexpr int N = 20000;
  const double max_extension = dx / 3.0;
  const double neighbour_radius = dx / 10.0;
  constexpr double goal_tolerance = 5.0;

  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<double> dis(0.0, 1.0);

  std::vector<Waypoint> overall_path;

  for (std::size_t checkpoint_index = 0;
    checkpoint_index < checkpoints_.size();
    ++checkpoint_index)
  {
    const Waypoint goal = checkpoints_.at(checkpoint_index);

    if (checkpoint_index > 0)
    {
      start = checkpoints_.at(checkpoint_index - 1);
    }

    std::vector<RRT_Node> nodes;
    nodes.reserve(N + 1);

    nodes.push_back(RRT_Node{
        start,
        0,
        -1,
        0.0
      });

    int best_goal_node = -1;
    double best_goal_cost =
      std::numeric_limits<double>::infinity();

    for (int i = 0; i < N; ++i)
    {
      // Randomly sample a point
      RRT_Node x_new{
          Waypoint{
              bounds.min_x + dis(gen) * dx,
              bounds.min_y + dis(gen) * dy
          },
          -1,
          0.0
      };

      // Find nearest node
      int nearest_index = 0;
      double nearest_dist_sq =
        std::numeric_limits<double>::infinity();

      for (std::size_t j = 0; j < nodes.size(); ++j)
      {
        const double ddx =
          x_new.coords.x - nodes.at(j).coords.x;

        const double ddy =
          x_new.coords.y - nodes.at(j).coords.y;

        const double dist_sq =
          ddx * ddx + ddy * ddy;

        if (dist_sq < nearest_dist_sq)
        {
          nearest_dist_sq = dist_sq;
          nearest_index = static_cast<int>(j);
        }
      }

      const double nearest_dist =
        std::sqrt(nearest_dist_sq);

      // Limit the extension distance
      if (nearest_dist > max_extension)
      {
        x_new.coords.x =
          nodes.at(nearest_index).coords.x +
          (x_new.coords.x -
            nodes.at(nearest_index).coords.x)
          / nearest_dist * max_extension;

        x_new.coords.y =
          nodes.at(nearest_index).coords.y +
          (x_new.coords.y -
            nodes.at(nearest_index).coords.y)
          / nearest_dist * max_extension;
      }

      // Check whether the new node and connecting segment are free
      if (!obstacle_map_.isPointFree(x_new.coords) ||
        !obstacle_map_.isSegmentFree(
          nodes.at(nearest_index).coords,
          x_new.coords))
      {
        continue;
      }

      // Find nearby nodes
      std::vector<int> nearby_nodes;
      nearby_nodes.reserve(nodes.size());

      const double neighbour_radius_sq =
        neighbour_radius * neighbour_radius;

      for (std::size_t j = 0; j < nodes.size(); ++j)
      {
        const double ddx =
          x_new.coords.x - nodes.at(j).coords.x;

        const double ddy =
          x_new.coords.y - nodes.at(j).coords.y;

        const double dist_sq =
          ddx * ddx + ddy * ddy;

        if (dist_sq < neighbour_radius_sq)
        {
          nearby_nodes.push_back(static_cast<int>(j));
        }
      }

      // Choose the lowest-cost parent
      int best_parent_index = nearest_index;

      double best_cost =
        nodes.at(nearest_index).cost + nearest_dist;

      for (const int node_index : nearby_nodes)
      {
        const double ddx =
          x_new.coords.x -
          nodes.at(node_index).coords.x;

        const double ddy =
          x_new.coords.y -
          nodes.at(node_index).coords.y;

        const double distance =
          std::sqrt(ddx * ddx + ddy * ddy);

        const double candidate_cost =
          nodes.at(node_index).cost + distance;

        if (candidate_cost < best_cost &&
          obstacle_map_.isSegmentFree(
            nodes.at(node_index).coords,
            x_new.coords))
        {
          best_cost = candidate_cost;
          best_parent_index = node_index;
        }
      }

      x_new.parent = best_parent_index;
      x_new.cost = best_cost;

      // Add the new node to the tree
      nodes.push_back(x_new);

      const int new_node_index =
        static_cast<int>(nodes.size() - 1);

      // Rewire nearby nodes if the new node provides a cheaper path
      for (const int node_index : nearby_nodes)
      {
        const double ddx =
          x_new.coords.x -
          nodes.at(node_index).coords.x;

        const double ddy =
          x_new.coords.y -
          nodes.at(node_index).coords.y;

        const double distance =
          std::sqrt(ddx * ddx + ddy * ddy);

        const double candidate_cost =
          nodes.at(new_node_index).cost + distance;

        if (candidate_cost < nodes.at(node_index).cost &&
          obstacle_map_.isSegmentFree(
            nodes.at(new_node_index).coords,
            nodes.at(node_index).coords))
        {
          nodes.at(node_index).parent =
            new_node_index;

          nodes.at(node_index).cost =
            candidate_cost;
        }
      }

      // Check whether the new node can reach the goal
      const double goal_dx =
        x_new.coords.x - goal.x;

      const double goal_dy =
        x_new.coords.y - goal.y;

      const double goal_distance =
        std::sqrt(goal_dx * goal_dx +
          goal_dy * goal_dy);

      if (goal_distance < goal_tolerance &&
        obstacle_map_.isSegmentFree(
          x_new.coords,
          goal))
      {
        const double goal_cost =
          x_new.cost + goal_distance;

        if (goal_cost < best_goal_cost)
        {
          best_goal_cost = goal_cost;
          best_goal_node = new_node_index;
        }
      }
    }

    // Extract the best path found to the goal
    if (best_goal_node == -1)
    {
      continue;
    }

    std::vector<Waypoint> path;

    int current_node = best_goal_node;

    while (current_node != -1)
    {
      path.push_back(
        nodes.at(current_node).coords);

      current_node =
        nodes.at(current_node).parent;
    }

    std::reverse(path.begin(), path.end());

    path.push_back(goal);

    overall_path.insert(
      overall_path.end(),
      path.begin(),
      path.end());
  }

  writePathCSV(overall_path);
  return overall_path;
}

void RoutePlanner::writePathCSV(const std::vector<Waypoint>& overall_path) const {
  const std::filesystem::path path_filepath =
    std::filesystem::path{ UAS_SIM_PROJECT_DIR } / "results" "/path.csv";
  std::ofstream path_file(path_filepath);
  path_file << "X,Y\n";

  for (const Waypoint& waypoint : overall_path)
  {
    path_file << waypoint.x << ","
      << waypoint.y <<  "\n";
  
    const std::filesystem::path checkpoints_path =
      std::filesystem::path{ UAS_SIM_PROJECT_DIR } / "results" "/checkpoints.csv";
    std::ofstream path_file(checkpoints_path);
    path_file << "X,Y\n";

    for (const Waypoint& waypoint : checkpoints_)
    {
      path_file << waypoint.x << ","
        << waypoint.y << "\n";
    }
  }

}