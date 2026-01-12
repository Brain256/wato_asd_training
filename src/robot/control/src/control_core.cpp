#include "control_core.hpp"

namespace robot
{

ControlCore::ControlCore(const rclcpp::Logger& logger) 
  : pure_pursuit_(logger),
    logger_(logger),
    node_(nullptr)
{
}

void ControlCore::initialize(rclcpp::Node* node)
{
  node_ = node;

  // Create subscribers
  path_sub_ = node_->create_subscription<nav_msgs::msg::Path>(
      "/path", 10, 
      [this](const nav_msgs::msg::Path::SharedPtr msg) { onPathReceived(msg); });

  odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
      "/odom/filtered", 10,
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) { onOdomReceived(msg); });

  // Create publisher
  cmd_vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

  // Create timer for control loop (10 Hz = 100 ms)
  control_timer_ = node_->create_wall_timer(
      std::chrono::milliseconds(100),
      [this]() { controlLoop(); });

  RCLCPP_INFO(logger_, "Control node initialized with Pure Pursuit Controller");
}

void ControlCore::onPathReceived(const nav_msgs::msg::Path::SharedPtr msg)
{
  pure_pursuit_.onPathReceived(msg);
}

void ControlCore::onOdomReceived(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  pure_pursuit_.onOdomReceived(msg);
}

void ControlCore::controlLoop()
{
  // Compute velocity command from Pure Pursuit Controller
  geometry_msgs::msg::Twist cmd_vel = pure_pursuit_.computeCommand();

  // Publish the velocity command
  cmd_vel_pub_->publish(cmd_vel);
}

}  
