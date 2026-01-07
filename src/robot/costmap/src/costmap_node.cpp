#include <memory>
#include "costmap_node.hpp"

CostmapNode::CostmapNode() : Node("costmap_node"), costmap_(robot::CostmapCore(this->get_logger())){
  this->declare_parameter("width", 40.0);
  this->declare_parameter("height", 40.0);
  this->declare_parameter("resolution", 0.1);
  this->declare_parameter("inflation_radius", 1.0);
  this->declare_parameter("max_cost", 100);

  double width = this->get_parameter("width").as_double();
  double height = this->get_parameter("height").as_double();
  double resolution = this->get_parameter("resolution").as_double();
  double inflation_radius = this->get_parameter("inflation_radius").as_double();
  int max_cost = this->get_parameter("max_cost").as_int();

  costmap_.setParameters(width, height, resolution, inflation_radius, max_cost);

  laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "/lidar",
    10,
    std::bind(&CostmapNode::laserCallback, this, std::placeholders::_1)
  );

  costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
    "/costmap", 
    10
  );

  RCLCPP_INFO(this->get_logger(), "Costmap node initialized!");
  RCLCPP_INFO(this->get_logger(), "  Subscribed to: /lidar");
  RCLCPP_INFO(this->get_logger(), "  Publishing to: /costmap");
}

void CostmapNode::laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
  auto costmap_msg = costmap_.processScan(scan);
  
  costmap_pub_->publish(costmap_msg);
  
  RCLCPP_DEBUG(this->get_logger(), "Published costmap");
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}