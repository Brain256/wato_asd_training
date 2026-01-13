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
  geometry_msgs::msg::Twist cmd_vel = pure_pursuit_.computeCommand();

  const bool is_zero =
      cmd_vel.linear.x == 0.0 && cmd_vel.linear.y == 0.0 && cmd_vel.linear.z == 0.0 &&
      cmd_vel.angular.x == 0.0 && cmd_vel.angular.y == 0.0 && cmd_vel.angular.z == 0.0;

  if (!is_zero) {
    cmd_vel_pub_->publish(cmd_vel);
    last_cmd_was_zero_ = false;
  } else if (!last_cmd_was_zero_) {
    cmd_vel_pub_->publish(cmd_vel);
    last_cmd_was_zero_ = true;
  }
}

}  
