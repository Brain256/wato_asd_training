#ifndef COSTMAP_CORE_HPP_
#define COSTMAP_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"   
#include "nav_msgs/msg/occupancy_grid.hpp" 
#include <vector>                            
#include <cmath>             

namespace robot
{

class CostmapCore {
  public:
    // Constructor, we pass in the node's RCLCPP logger to enable logging to terminal
    explicit CostmapCore(const rclcpp::Logger& logger);
    
    nav_msgs::msg::OccupancyGrid processScan(
        const sensor_msgs::msg::LaserScan::SharedPtr scan);
    

    void setParameters(double width, double height, double resolution,
                      double inflation_radius, int max_cost);

  private:
    rclcpp::Logger logger_;

    double width_;
    double height_;
    double resolution_;
    double inflation_radius_;
    int max_cost_;

    int grid_width_;
    int grid_height_;

    std::vector<int8_t> costmap_data_;

    void initializeCostmap();
    void convertToGrid(double range, double angle, int& x_grid, int& y_grid);
    void markObstacle(int x_grid, int y_grid);
    void inflateObstacles();
    double getDistance(int x1, int y1, int x2, int y2);
};
}  

#endif