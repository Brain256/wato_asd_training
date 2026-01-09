#include "planner_node.hpp"

PlannerNode::PlannerNode() 
  : Node("planner"), 
    planner_(robot::PlannerCore(this->get_logger())),
    state_(State::WAITING_FOR_GOAL) 
{
  map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/map", 10, 
    std::bind(&PlannerNode::mapCallback, this, std::placeholders::_1));
  
  goal_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
    "/goal_point", 10,
    std::bind(&PlannerNode::goalCallback, this, std::placeholders::_1));
  
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", 10,
    std::bind(&PlannerNode::odomCallback, this, std::placeholders::_1));
  
  path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10);
  
  timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&PlannerNode::timerCallback, this));
  
  RCLCPP_INFO(this->get_logger(), "Planner node initialized. State: WAITING_FOR_GOAL");
}

void PlannerNode::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  current_map_ = *msg;  
  map_received_ = true;  
  
  RCLCPP_INFO(this->get_logger(), "Received map: %dx%d cells", 
    msg->info.width, msg->info.height);
  
  if (state_ == State::WAITING_FOR_ROBOT_TO_REACH_GOAL) {
    RCLCPP_INFO(this->get_logger(), "Map updated. Replanning...");
    planPath();
  }
}

void PlannerNode::goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
{
  goal_ = *msg;  
  goal_received_ = true;
  
  RCLCPP_INFO(this->get_logger(), "Received goal: (%.2f, %.2f)", 
    msg->point.x, msg->point.y);
  
  state_ = State::WAITING_FOR_ROBOT_TO_REACH_GOAL;
  RCLCPP_INFO(this->get_logger(), "State: WAITING_FOR_ROBOT_TO_REACH_GOAL");
  
  planPath();
}

void PlannerNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  robot_pose_ = msg->pose.pose;  
  odom_received_ = true;
}

void PlannerNode::timerCallback()
{
  if (state_ == State::WAITING_FOR_ROBOT_TO_REACH_GOAL) {
    
    if (goalReached()) {
      RCLCPP_INFO(this->get_logger(), "Goal reached!");
      state_ = State::WAITING_FOR_GOAL;
      RCLCPP_INFO(this->get_logger(), "State: WAITING_FOR_GOAL");
    }
  }
}

bool PlannerNode::goalReached() const
{
  if (!goal_received_ || !odom_received_) {
    return false;
  }
  
  double dx = goal_.point.x - robot_pose_.position.x;
  double dy = goal_.point.y - robot_pose_.position.y;
  double distance = std::sqrt(dx * dx + dy * dy);
  
  const double GOAL_THRESHOLD = 0.5;
  return distance < GOAL_THRESHOLD;
}

void PlannerNode::planPath()
{
  if (!map_received_) {
    RCLCPP_WARN(this->get_logger(), "Cannot plan path: Map not received yet!");
    return;
  }
  
  if (!goal_received_) {
    RCLCPP_WARN(this->get_logger(), "Cannot plan path: Goal not received yet!");
    return;
  }
  
  if (!odom_received_) {
    RCLCPP_WARN(this->get_logger(), "Cannot plan path: Odometry not received yet!");
    return;
  }
  
  if (current_map_.data.empty()) {
    RCLCPP_WARN(this->get_logger(), "Cannot plan path: Map data is empty!");
    return;
  }
  
  RCLCPP_INFO(this->get_logger(), "Running A* pathfinding...");
  nav_msgs::msg::Path path = planner_.planPath(current_map_, robot_pose_, goal_);
  
  if (path.poses.empty()) {
    RCLCPP_WARN(this->get_logger(), "No valid path found to goal!");
  } else {
    RCLCPP_INFO(this->get_logger(), "Path found with %zu waypoints", path.poses.size());
    path_pub_->publish(path);
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
  return 0;
}
