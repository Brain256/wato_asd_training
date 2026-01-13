#include "planner_core.hpp"

namespace robot
{

PlannerCore::PlannerCore(const rclcpp::Logger& logger) 
: logger_(logger) {}

nav_msgs::msg::Path PlannerCore::planPath(
  const nav_msgs::msg::OccupancyGrid& map,
  const geometry_msgs::msg::Pose& start_pose,
  const geometry_msgs::msg::PointStamped& goal)
{
  CellIndex start_cell = worldToGrid(start_pose.position.x, start_pose.position.y, map);
  CellIndex goal_cell = worldToGrid(goal.point.x, goal.point.y, map);
  
  RCLCPP_INFO(logger_, "Planning from cell (%d, %d) to cell (%d, %d)", 
    start_cell.x, start_cell.y, goal_cell.x, goal_cell.y);
  
  if(!isValidCell(start_cell, map)){
    RCLCPP_ERROR(logger_, "Start position is not valid (obstacle or out of bounds)!");
    return nav_msgs::msg::Path(); 
  }
  
  if(!isValidCell(goal_cell, map)){
    RCLCPP_ERROR(logger_, "Goal position is not valid (obstacle or out of bounds)!");
    return nav_msgs::msg::Path();  
  }
  
  std::priority_queue<AStarNode, std::vector<AStarNode>, CompareF> open_set;
  
  std::unordered_map<CellIndex, bool, CellIndexHash> in_open_set;
  
  std::unordered_map<CellIndex, bool, CellIndexHash> closed_set;
  
  std::unordered_map<CellIndex, CellIndex, CellIndexHash> came_from;
  
  std::unordered_map<CellIndex, double, CellIndexHash> g_score;
  

  g_score[start_cell] = 0.0;
  double f_start = heuristic(start_cell, goal_cell); 
  open_set.push(AStarNode(start_cell, f_start));
  in_open_set[start_cell] = true;
  
  while(!open_set.empty()){
    AStarNode current_node = open_set.top();
    open_set.pop();
    CellIndex current = current_node.index;
    
    in_open_set[current] = false;
    
    if(current == goal_cell){
      RCLCPP_INFO(logger_, "Path found!");
      return reconstructPath(came_from, current, map, map.header.frame_id);
    }

    closed_set[current] = true;
    
    std::vector<CellIndex> neighbors = getNeighbors(current);
    
    for (const auto& neighbor : neighbors) {
      if(!isValidCell(neighbor, map)){
        continue;
      }
      
      if(closed_set[neighbor]){
        continue;
      }
      
      double step_cost = moveCost(current, neighbor, map);
      double tentative_g_score = g_score[current] + step_cost;
      
      bool is_better_path = false;
      
      if(g_score.find(neighbor) == g_score.end()){
        is_better_path = true;
      } 
      else if(tentative_g_score < g_score[neighbor]){
        is_better_path = true;
      }
      
      if (is_better_path) {
        came_from[neighbor] = current;
        g_score[neighbor] = tentative_g_score;
        
        double h_score = heuristic(neighbor, goal_cell);
        double f_score = tentative_g_score + h_score;
        
        if (!in_open_set[neighbor]) {
          open_set.push(AStarNode(neighbor, f_score));
          in_open_set[neighbor] = true;
        }
      }
    }
  }

  RCLCPP_WARN(logger_, "No path found to goal!");
  return nav_msgs::msg::Path();
}

CellIndex PlannerCore::worldToGrid(double x, double y, 
                                    const nav_msgs::msg::OccupancyGrid& map) const
{
  int grid_x = static_cast<int>((x - map.info.origin.position.x) / map.info.resolution);
  int grid_y = static_cast<int>((y - map.info.origin.position.y) / map.info.resolution);
  
  return CellIndex(grid_x, grid_y);
}

void PlannerCore::gridToWorld(const CellIndex& cell, 
                               const nav_msgs::msg::OccupancyGrid& map, 
                               double& x, double& y) const
{
  x = (cell.x + 0.5) * map.info.resolution + map.info.origin.position.x;
  y = (cell.y + 0.5) * map.info.resolution + map.info.origin.position.y;
}

bool PlannerCore::isValidCell(const CellIndex& cell, 
                               const nav_msgs::msg::OccupancyGrid& map) const
{
  if (cell.x < 0 || cell.x >= static_cast<int>(map.info.width) ||
      cell.y < 0 || cell.y >= static_cast<int>(map.info.height)) {
    return false;
  }
  
  int value = getCellValue(cell, map);

  return value >= 0 && value < 50;
}

int PlannerCore::getCellValue(const CellIndex& cell, 
                               const nav_msgs::msg::OccupancyGrid& map) const
{
  //Map data is stored in a 1D array in row-major order
  //Index formula: index = y * width + x
  
  int index = cell.y * map.info.width + cell.x;
  
  if (index < 0 || index >= static_cast<int>(map.data.size())) {
    return -1;
  }
  
  return map.data[index];
}

double PlannerCore::heuristic(const CellIndex& current, const CellIndex& goal) const
{
  
  double dx = goal.x - current.x;
  double dy = goal.y - current.y;
  return std::sqrt(dx * dx + dy * dy);
}

std::vector<CellIndex> PlannerCore::getNeighbors(const CellIndex& cell) const
{
  std::vector<CellIndex> neighbors;
  neighbors.reserve(8);
  
  neighbors.emplace_back(cell.x + 1, cell.y);      
  neighbors.emplace_back(cell.x - 1, cell.y);      
  neighbors.emplace_back(cell.x, cell.y + 1);      
  neighbors.emplace_back(cell.x, cell.y - 1);     
  
  neighbors.emplace_back(cell.x + 1, cell.y + 1); 
  neighbors.emplace_back(cell.x - 1, cell.y + 1); 
  neighbors.emplace_back(cell.x + 1, cell.y - 1);  
  neighbors.emplace_back(cell.x - 1, cell.y - 1); 
  
  return neighbors;
}

double PlannerCore::moveCost(const CellIndex& current,
                             const CellIndex& neighbor,
                             const nav_msgs::msg::OccupancyGrid& map) const
{
  double base = (neighbor.x != current.x && neighbor.y != current.y) ? std::sqrt(2.0) : 1.0;
  int occ = getCellValue(neighbor, map);
  if (occ < 0) {
    return base;
  }
  int capped = std::min(29, occ);
  double frac = static_cast<double>(capped) / 30.0;
  const double weight = 3.5;
  double penalty = weight * frac * frac;
  return base + penalty;
}

nav_msgs::msg::Path PlannerCore::reconstructPath(
  const std::unordered_map<CellIndex, CellIndex, CellIndexHash>& came_from,
  CellIndex current,
  const nav_msgs::msg::OccupancyGrid& map,
  const std::string& frame_id) const
{
  std::vector<CellIndex> cell_path;
  cell_path.push_back(current);
  
  while (came_from.find(current) != came_from.end()) {
    current = came_from.at(current);
    cell_path.push_back(current);
  }
  
  std::reverse(cell_path.begin(), cell_path.end());
  
  nav_msgs::msg::Path path;
  path.header.frame_id = frame_id;
  path.header.stamp = rclcpp::Clock().now();
  
  for (const auto& cell : cell_path) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    
    gridToWorld(cell, map, pose.pose.position.x, pose.pose.position.y);
    pose.pose.position.z = 0.0;
    
    pose.pose.orientation.w = 1.0;
    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = 0.0;
    
    path.poses.push_back(pose);
  }
  
  return path;
}

}  // namespace robot
