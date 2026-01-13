#ifndef PLANNER_NODE_HPP_
#define PLANNER_NODE_HPP_

#include "rclcpp/rclcpp.hpp"

#include "planner_core.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "nav_msgs/msg/path.hpp"

class PlannerNode : public rclcpp::Node {
  public:
    PlannerNode();

  private:
  
    enum class State {
      WAITING_FOR_GOAL,                // Initial state - waiting for user to send a goal
      WAITING_FOR_ROBOT_TO_REACH_GOAL  // Active navigation - moving toward goal
    };
    
    State state_;  
    
    
    robot::PlannerCore planner_;
    
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr goal_point_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    
    rclcpp::TimerBase::SharedPtr timer_;

    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void goalPointCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg);

    void timerCallback();
    
    bool goalReached() const;
    
    void planPath();
    
    nav_msgs::msg::OccupancyGrid current_map_;
    geometry_msgs::msg::PoseStamped goal_pose_;
    geometry_msgs::msg::Pose robot_pose_;

    bool map_received_ = false;
    bool goal_received_ = false;
    bool odom_received_ = false;

    // Timing for replanning and goal tracking
    rclcpp::Time goal_start_time_;
    rclcpp::Time last_plan_time_;
    double plan_timeout_sec_ = 0.2; // seconds

    // Progress-based timeout: replan if robot hasn't moved enough
    geometry_msgs::msg::Pose last_progress_pose_;
    double progress_min_dist_ = 0.1;   // meters
    double progress_timeout_sec_ = 0.5; // seconds
};

#endif
