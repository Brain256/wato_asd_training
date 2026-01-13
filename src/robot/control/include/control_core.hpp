#ifndef CONTROL_CORE_HPP_
#define CONTROL_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "pure_pursuit.controller.hpp"

namespace robot
{

class ControlCore {
  public:
    // Constructor, we pass in the node's RCLCPP logger to enable logging to terminal
    ControlCore(const rclcpp::Logger& logger);

    // Initialize subscribers, publishers, and timer
    void initialize(rclcpp::Node* node);
  
  private:
    // Control loop callback
    void controlLoop();

    // Callbacks for subscriptions
    void onPathReceived(const nav_msgs::msg::Path::SharedPtr msg);
    void onOdomReceived(const nav_msgs::msg::Odometry::SharedPtr msg);

    // ROS2 components
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::TimerBase::SharedPtr control_timer_;

    // Pure Pursuit Controller instance
    PurePursuitController pure_pursuit_;

    // Logger
    rclcpp::Logger logger_;

    // Raw node pointer for creating subscriptions/publishers
    rclcpp::Node* node_;

    // Track whether last published command was zero to avoid spamming zeros
    bool last_cmd_was_zero_ = true;
};

} 

#endif 
