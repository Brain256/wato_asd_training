#ifndef PLANNER_CORE_HPP_
#define PLANNER_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include <queue>
#include <unordered_map>
#include <vector>
#include <cmath>

namespace robot
{

// ------------------- Supporting Structures -------------------
 
// 2D grid index
struct CellIndex
{
  int x;
  int y;
 
  CellIndex(int xx, int yy) : x(xx), y(yy) {}
  CellIndex() : x(0), y(0) {}
 
  bool operator==(const CellIndex &other) const
  {
    return (x == other.x && y == other.y);
  }
 
  bool operator!=(const CellIndex &other) const
  {
    return (x != other.x || y != other.y);
  }
};
 
// Hash function for CellIndex so it can be used in std::unordered_map
struct CellIndexHash
{
  std::size_t operator()(const CellIndex &idx) const
  {
    // A simple hash combining x and y
    return std::hash<int>()(idx.x) ^ (std::hash<int>()(idx.y) << 1);
  }
};
 
// Structure representing a node in the A* open set
struct AStarNode
{
  CellIndex index;
  double f_score;  // f = g + h
 
  AStarNode(CellIndex idx, double f) : index(idx), f_score(f) {}
};
 
// Comparator for the priority queue (min-heap by f_score)
struct CompareF
{
  bool operator()(const AStarNode &a, const AStarNode &b)
  {
    // We want the node with the smallest f_score on top
    return a.f_score > b.f_score;
  }
};

class PlannerCore {
  public:
    explicit PlannerCore(const rclcpp::Logger& logger);
    
    nav_msgs::msg::Path planPath(
      const nav_msgs::msg::OccupancyGrid& map,
      const geometry_msgs::msg::Pose& start_pose,
      const geometry_msgs::msg::PointStamped& goal);

  private:
    rclcpp::Logger logger_;
    
    CellIndex worldToGrid(double x, double y, 
                          const nav_msgs::msg::OccupancyGrid& map) const;
    
    void gridToWorld(const CellIndex& cell, 
                     const nav_msgs::msg::OccupancyGrid& map, 
                     double& x, double& y) const;
    
    bool isValidCell(const CellIndex& cell, 
                     const nav_msgs::msg::OccupancyGrid& map) const;
    
    int getCellValue(const CellIndex& cell, 
                     const nav_msgs::msg::OccupancyGrid& map) const;
    
    
    double heuristic(const CellIndex& current, const CellIndex& goal) const;
    
    std::vector<CellIndex> getNeighbors(const CellIndex& cell) const;
    
    nav_msgs::msg::Path reconstructPath(
      const std::unordered_map<CellIndex, CellIndex, CellIndexHash>& came_from,
      CellIndex current,
      const nav_msgs::msg::OccupancyGrid& map,
      const std::string& frame_id) const;
};
}  

#endif
