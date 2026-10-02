#include "uas_sim/routeplanner/gridplanner.h"

GridPlanner::GridPlanner() {}

std::vector<GridIndex> GridPlanner::Plan(const OccupancyGrid& occupancy_grid, 
  const Costmap& costmap, GridIndex start, GridIndex goal) {
  bool can_reach_target = PlanAStar(costmap, start, goal);

  if (can_reach_target) {
    return route_.path;
  }

  FindFrontiers(occupancy_grid, costmap, start);
  
  GridIndex target = NavigateTo(costmap, start, goal);

  return route_.path;
}

bool GridPlanner::PlanAStar(const Costmap& costmap, GridIndex start, GridIndex goal) {
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
      route_.total_cost = node->g_cost;

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

bool GridPlanner::FindFrontiers(const OccupancyGrid& occupancy_grid,
  const Costmap& costmap, GridIndex from) {

  frontiers_.clear();
  std::vector < std::pair<int, int>> cardinal_ne = 
  { {1,0},{0,-1}, {-1,0}, {0,1} };
  std::vector<GridIndex> frontier_cells{};

  for (int i = 0; i < occupancy_grid.grid.size(); ++i) {
    for (int j = 0; j < occupancy_grid.grid[0].size(); ++j) {

      GridIndex cell{ i,j };

      if (occupancy_grid.getState(i, j) != OccupancyState::Free){
        continue;
      }
      if (!costmap.isTraversable(cell)) {
        continue;
      }
      GridIndex cell = { i,j };
      for (std::pair<int, int>& ne : cardinal_ne) {

        GridIndex neighbour{
          i + ne.first,
          j + ne.second
        };

        // Do not read outside the occupancy grid.
        if (neighbour.i < 0 || neighbour.j < 0 ||
          neighbour.i >= static_cast<int>(occupancy_grid.grid.size()) ||
          neighbour.j >= static_cast<int>(
            occupancy_grid.grid.front().size())) {
          continue;
        }

        //Frontier Cell Check
        if (occupancy_grid.getState(neighbour.i, neighbour.j) ==
          OccupancyState::Unknown) {
          //needs to be added to a frontier
          if (std::find(frontier_cells.begin(), frontier_cells.end(), cell)
            == frontier_cells.end()) {
            frontier_cells.push_back(cell);
            break;
          }
        }
      }
    }
  }
  
  bool created = ClusterFrontiers(frontier_cells);


  return created;
}

bool GridPlanner::ClusterFrontiers(std::vector<GridIndex> frontier_cells) {
  frontiers_.clear();
  std::vector<GridIndex> visited_cells{};
  std::vector < std::pair<int, int>> cardinal_ne =
  { {1,0},{0,-1}, {-1,0}, {0,1} };

  //for every frontier cell
  for (int i = 0; i < frontier_cells.size(); ++i) {
    
    // check cell hasn't already been visited
    if (std::find(visited_cells.begin(), visited_cells.end(), frontier_cells.at(i))
      != visited_cells.end()) {
      continue;
    }

    Frontier new_frontier{};
    
    std::vector<GridIndex> queue = { frontier_cells.at(i) };
    visited_cells.push_back(frontier_cells.at(i));

    while (!queue.empty()) {
      GridIndex cell = queue.back();
      queue.pop_back();

      new_frontier.cells.push_back(cell);
      
      for (std::pair<int, int>& ne : cardinal_ne) {
        GridIndex neighbour{
          cell.i + ne.first,
          cell.j + ne.second
        };

        if (std::find(frontier_cells.begin(), frontier_cells.end(), neighbour) != frontier_cells.end() &&
          std::find(visited_cells.begin(), visited_cells.end(), neighbour) == visited_cells.end()) {
          visited_cells.push_back(neighbour);
          queue.push_back(neighbour);
        }
      }
    }

    frontiers_.push_back(new_frontier);
  }
  return true;
}

double GridPlanner::Heuristic(GridIndex from, GridIndex goal,
  double resolution) {
  return resolution * (std::max(std::abs(from.i - goal.i), std::abs(from.j - goal.j))
    + (std::sqrt(2.0) - 1.0) * 
    std::min(std::abs(from.i - goal.i), std::abs(from.j - goal.j)));
}

PlanResult GridPlanner::GetRoute() const {
  return route_;
}

GridIndex GridPlanner::NavigateTo(const Costmap& costmap, GridIndex start, GridIndex to) {

  double best_score = -std::numeric_limits<double>::infinity();
  PlanResult best_route{};
  GridIndex navigate_to = to;

  for (Frontier& frontier : frontiers_) {
    
    if (frontier.cells.empty()) {
      continue;
    }

    // Get Centroid of Cluster
      int centroid_i = std::ceil(std::accumulate(
        frontier.cells.begin(), frontier.cells.end(), 0.0,
        [](double sum, const GridIndex& cell) {
          return sum + cell.i;
        }
      ) / frontier.cells.size());

      int centroid_j = std::ceil(std::accumulate(
        frontier.cells.begin(), frontier.cells.end(), 0.0,
        [](double sum, const GridIndex& cell) {
          return sum + cell.j;
        }
      ) / frontier.cells.size());

      double closest_distance_sq =
        std::numeric_limits<double>::infinity();

      GridIndex closest_to_centroid{};

      for (const GridIndex& cell : frontier.cells) {
        const double di = cell.i - centroid_i;
        const double dj = cell.j - centroid_j;
        const double distance_sq = di * di + dj * dj;

        if (distance_sq < closest_distance_sq) {
          closest_distance_sq = distance_sq;
          closest_to_centroid = cell;
        }
      }

      bool pass = PlanAStar(costmap, start, closest_to_centroid);

      if (!pass) {
        continue;
      }

      int information_gain = frontier.cells.size();
      double cost = route_.total_cost;

      double score = information_gain / (cost + 1e-3) -
        Heuristic(closest_to_centroid, to, costmap.resolution_);
      
      if (score > best_score) {
        best_score = score;
        best_route = route_;
        navigate_to = closest_to_centroid;
      }
  };
  route_ = best_route;
  return navigate_to;
}

std::vector<Frontier> GridPlanner::GetFrontiers() const {
  return frontiers_;
}
