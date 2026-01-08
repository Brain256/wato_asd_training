#ifndef MAP_MEMORY_CORE_HPP_
#define MAP_MEMORY_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp" 

namespace robot
{

class MapMemoryCore {
  public:
    explicit MapMemoryCore(const rclcpp::Logger& logger);
    
    nav_msgs::msg::OccupancyGrid integrateCostmap(
        const nav_msgs::msg::OccupancyGrid& costmap,
        const nav_msgs::msg::Odometry& odom);

  private:
    rclcpp::Logger logger_;
    
    nav_msgs::msg::OccupancyGrid global_map_;
    bool map_initialized_;
    
    void initializeGlobalMap(const nav_msgs::msg::OccupancyGrid& first_costmap);
    void mergeCostmapIntoGlobalMap(const nav_msgs::msg::OccupancyGrid& costmap,
                                  const nav_msgs::msg::Odometry& odom);
};

}

#endif
