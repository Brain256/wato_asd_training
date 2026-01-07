#include "costmap_core.hpp"

namespace robot
{

CostmapCore::CostmapCore(const rclcpp::Logger& logger) 
    : logger_(logger),
        width_(40.0),
        height_(40.0),
        resolution_(0.1),
        inflation_radius_(1.0),
        max_cost_(100){
  
    grid_width_ = static_cast<int>(width_ / resolution_);
    grid_height_ = static_cast<int>(height_ / resolution_);

    RCLCPP_INFO(logger_, "CostmapCore initialized: %d x %d cells", grid_width_, grid_height_);
}

void CostmapCore::initializeCostmap(){
    costmap_data_.clear();
    costmap_data_.resize(grid_width_ * grid_height_, 0);
}

void CostmapCore::convertToGrid(double range, double angle, int& x_grid, int& y_grid){

    double x = range * std::cos(angle);
    double y = range * std::sin(angle);


    double x_centered = x + (width_ / 2.0);
    double y_centered = y + (height_ / 2.0);

    x_grid = static_cast<int>(x_centered / resolution_);
    y_grid = static_cast<int>(y_centered / resolution_);
}

void CostmapCore::markObstacle(int x_grid, int y_grid){
    if(x_grid < 0 || x_grid >= grid_width_ || y_grid < 0 || y_grid >= grid_height_){
        return;
    }

    //Convert to 1D index and mark as obstacle
    int index = y_grid * grid_width_ + x_grid;
    costmap_data_[index] = max_cost_;
}

double CostmapCore::getDistance(int x1, int y1, int x2, int y2){
    int dx = x2 - x1;
    int dy = y2 - y1;
    return std::sqrt(dx * dx + dy * dy) * resolution_;
}

void CostmapCore::inflateObstacles(){
    std::vector<int8_t> inflated_costmap = costmap_data_;

    int inflation_cells = static_cast<int>(inflation_radius_ / resolution_);
  
    for(int y = 0; y < grid_height_; ++y){
        for(int x = 0; x < grid_width_; ++x){
            int index = y * grid_width_ + x;
            
            if(costmap_data_[index] == max_cost_){
                for(int dy = -inflation_cells; dy <= inflation_cells; ++dy){
                    for(int dx = -inflation_cells; dx <= inflation_cells; ++dx){
                    
                        int nx = x + dx;
                        int ny = y + dy;
                        
                        if(nx < 0 || nx >= grid_width_ || ny < 0 || ny >= grid_height_){
                            continue;
                        }
                        
                        double distance = getDistance(x, y, nx, ny);
                        
                        if(distance <= inflation_radius_){
                            int cost = static_cast<int>(max_cost_ * (1.0 - distance / inflation_radius_));
                            int neighbor_index = ny * grid_width_ + nx;
                            if (cost > inflated_costmap[neighbor_index]) {
                                inflated_costmap[neighbor_index] = cost;
                            }
                        }
                    }
                }
            }
        }
    }
    costmap_data_ = inflated_costmap;
}

nav_msgs::msg::OccupancyGrid CostmapCore::processScan(const sensor_msgs::msg::LaserScan::SharedPtr scan){
  
    initializeCostmap();

    for(size_t i = 0; i < scan->ranges.size(); ++i){
        double angle = scan->angle_min + i * scan->angle_increment;
        double range = scan->ranges[i];

        if(range >= scan->range_min && range <= scan->range_max){
            int x_grid, y_grid;
            convertToGrid(range, angle, x_grid, y_grid);
            
            markObstacle(x_grid, y_grid);
        }
    }

    inflateObstacles();

    nav_msgs::msg::OccupancyGrid costmap_msg;

    costmap_msg.header.stamp = scan->header.stamp;  
    costmap_msg.header.frame_id = scan->header.frame_id;  

    costmap_msg.info.resolution = resolution_;
    costmap_msg.info.width = grid_width_;
    costmap_msg.info.height = grid_height_;

    //Set origin (bottom-left corner of map in world coordinates)
    costmap_msg.info.origin.position.x = -width_ / 2.0;
    costmap_msg.info.origin.position.y = -height_ / 2.0;
    costmap_msg.info.origin.position.z = 0.0;

    //Orientation (no rotation, just identity quaternion)
    costmap_msg.info.origin.orientation.x = 0.0;
    costmap_msg.info.origin.orientation.y = 0.0;
    costmap_msg.info.origin.orientation.z = 0.0;
    costmap_msg.info.origin.orientation.w = 1.0;

    //Copy the costmap data (already a 1D array!)
    costmap_msg.data = costmap_data_;

    RCLCPP_DEBUG(logger_, "Costmap created with %zu cells", costmap_data_.size());

    return costmap_msg;
}

void CostmapCore::setParameters(double width, double height, double resolution,
                                double inflation_radius, int max_cost) {
  width_ = width;
  height_ = height;
  resolution_ = resolution;
  inflation_radius_ = inflation_radius;
  max_cost_ = max_cost;
  
  grid_width_ = static_cast<int>(width_ / resolution_);
  grid_height_ = static_cast<int>(height_ / resolution_);
  
  RCLCPP_INFO(logger_, "Parameters updated: %dx%d cells (%.1fm x %.1fm)", 
              grid_width_, grid_height_, width_, height_);
}

}