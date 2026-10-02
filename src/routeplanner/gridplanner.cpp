#include "uas_sim/routeplanner/gridplanner.h"

GridPlanner::GridPlanner() {}

bool GridPlanner::Plan(const Costmap& costmap, GridIndex start, GridIndex goal) {
  route_ = {};

  if (costmap.costs.empty() || costmap.costs.front().empty()) {
    std::cerr << "Costmap is empty.\n";
    return false;
  }

  if (start.i < 0 || start.i>= costmap.costs.size() || start.j < 0 || start.j >= costmap.costs[0].size()) {
    std::cerr << "Start index is out of bounds." << std::endl;
    return false;
  }
  else if (goal.i < 0 || goal.i>= costmap.costs.size() || goal.j < 0 || goal.j >= costmap.costs[0].size()) {
    std::cerr << "Goal index is out of bounds." << std::endl;
    return false;
  }
  else if (!costmap.isTraversable(start)) {
    std::cerr << "Start index is not traversable." << std::endl;
    return false;
  }
  else if (!costmap.isTraversable(goal)) {
    std::cerr << "Goal index is not traversable." << std::endl;
    return false;
  }
  else if(start.i == goal.i && start.j == goal.j) {
    route_.found_path = true;
    route_.path.push_back(start);
    return true;
  }

  //Initialize the nodes, open set, and closed set

  std::vector<std::vector<Node>> nodes(costmap.costs.size(), std::vector<Node>(costmap.costs[0].size()));
  std::vector<Node*> open_set;
  open_set.reserve(costmap.costs.size() * costmap.costs[0].size());
  std::vector<std::vector<bool>> closed_set(costmap.costs.size(), std::vector<bool>(costmap.costs[0].size(), false));

  Node& start_node = nodes.at(start.i).at(start.j);
  start_node.index = start;
  start_node.g_cost = 0.0;
  start_node.h_cost = Heuristic(start, goal, costmap.resolution_);
  start_node.parent = nullptr;
  start_node.open = true;

  open_set.push_back(&start_node);

  while (!open_set.empty()) {

    //Sort open set
    std::sort(open_set.begin(), open_set.end(),
      [](const Node* left, const Node* right)
      { return (left->f_cost() < right->f_cost()); });

    //Get lowest total cost and remove from set
    Node* node = open_set.front();
    open_set.erase(open_set.begin(), open_set.begin()+1);

    //Goal check
    if (node->index.i == goal.i &&
      node->index.j == goal.j) {
      const Node* path_node = node;

      while (path_node != nullptr) {
        route_.path.insert(route_.path.begin(), path_node->index);
        path_node = path_node->parent;
      }
      route_.found_path = true;
      return true;
    }

    //Check node open
    if (!node->open) {
      continue;
    }

    //Mark closed
    node->open = false;
    closed_set.at(node->index.i).at(node->index.j) = true;

    // Examine neighbours
    for (std::pair<int,int>& neighbour_diff : neighbour_diffs_) {
      GridIndex candidate_neighbour = { node->index.i + neighbour_diff.first,
      node->index.j + neighbour_diff.second };
      
      if (!costmap.isTraversable(candidate_neighbour)) {
        continue;
      }

      

        double neighbour_dist = std::sqrt(neighbour_diff.first * neighbour_diff.first +
          neighbour_diff.second * neighbour_diff.second) *costmap.resolution_;

        double possible_g = node->g_cost+neighbour_dist *
          costmap.costs.at(candidate_neighbour.i).at(candidate_neighbour.j);
        Node& neighbour =
          nodes.at(candidate_neighbour.i).at(candidate_neighbour.j);

        //Check if route from current node is faster
        if (possible_g <
          neighbour.g_cost) {

          neighbour.index = candidate_neighbour;
          neighbour.g_cost = possible_g;
          neighbour.h_cost = Heuristic(
            candidate_neighbour,
            goal,
            costmap.resolution_
          );
          neighbour.parent = node;
          neighbour.open = true;


          open_set.push_back(&nodes.at(candidate_neighbour.i).at(candidate_neighbour.j));
        
      }
    }
  }

  route_.found_path = false;

  return false;
}


double GridPlanner::Heuristic(GridIndex from, GridIndex goal, double resolution) {
  return resolution * (std::max(std::abs(from.i - goal.i), std::abs(from.j - goal.j))
    + (std::sqrt(2.0) - 1.0) * std::min(std::abs(from.i - goal.i), std::abs(from.j - goal.j)));
}

PlanResult GridPlanner::GetRoute() const {
  return route_;
}
