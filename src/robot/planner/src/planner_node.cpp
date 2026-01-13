#include "planner_node.hpp"

PlannerNode::PlannerNode() 
  : Node("planner"), 
    planner_(robot::PlannerCore(this->get_logger())),
    state_(State::WAITING_FOR_GOAL) 
{
  map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/map", 10, 
    std::bind(&PlannerNode::mapCallback, this, std::placeholders::_1));
  
  goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/goal_pose", 10,
    std::bind(&PlannerNode::goalCallback, this, std::placeholders::_1));
  goal_point_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
    "/goal_point", 10,
    std::bind(&PlannerNode::goalPointCallback, this, std::placeholders::_1));
  
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", 10,
    std::bind(&PlannerNode::odomCallback, this, std::placeholders::_1));
  
  path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10);
  
  timer_ = this->create_wall_timer(std::chrono::milliseconds(200), std::bind(&PlannerNode::timerCallback, this));
  
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

void PlannerNode::goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  goal_pose_ = *msg;  
  goal_received_ = true;
  goal_start_time_ = this->now();
  
  RCLCPP_INFO(this->get_logger(), "Received goal: (%.2f, %.2f)", 
    msg->pose.position.x, msg->pose.position.y);
  
  state_ = State::WAITING_FOR_ROBOT_TO_REACH_GOAL;
  RCLCPP_INFO(this->get_logger(), "State: WAITING_FOR_ROBOT_TO_REACH_GOAL");
  
  planPath();
}

void PlannerNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  robot_pose_ = msg->pose.pose;  
  odom_received_ = true;
}

void PlannerNode::goalPointCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
{
  // Convert PointStamped to PoseStamped (zero yaw)
  geometry_msgs::msg::PoseStamped pose;
  pose.header = msg->header;
  pose.pose.position = msg->point;
  pose.pose.orientation.w = 1.0;
  pose.pose.orientation.x = 0.0;
  pose.pose.orientation.y = 0.0;
  pose.pose.orientation.z = 0.0;
  goalCallback(std::make_shared<geometry_msgs::msg::PoseStamped>(pose));
}

void PlannerNode::timerCallback()
{
  if (state_ == State::WAITING_FOR_ROBOT_TO_REACH_GOAL) {
    
    if (goalReached()) {
      RCLCPP_INFO(this->get_logger(), "Goal reached!");
      // Publish empty path to signal stop
      nav_msgs::msg::Path empty_path;
      empty_path.header.stamp = this->now();
      empty_path.header.frame_id = current_map_.header.frame_id;
      path_pub_->publish(empty_path);

      // Reset state for new goals
      state_ = State::WAITING_FOR_GOAL;
      RCLCPP_INFO(this->get_logger(), "State: WAITING_FOR_GOAL");
      goal_received_ = false;
    } else {
      // Timeout-based replanning to handle lack of progress
      if (last_plan_time_.nanoseconds() == 0 ||
          (this->now() - last_plan_time_).seconds() >= plan_timeout_sec_) {
        RCLCPP_INFO(this->get_logger(), "Replanning due to timeout...");
        planPath();
      } else {
        // Progress-based timeout: replan if robot hasn't moved enough
        double dx = robot_pose_.position.x - last_progress_pose_.position.x;
        double dy = robot_pose_.position.y - last_progress_pose_.position.y;
        double dist = std::sqrt(dx * dx + dy * dy);
        if ((this->now() - last_plan_time_).seconds() >= progress_timeout_sec_ && dist < progress_min_dist_) {
          RCLCPP_INFO(this->get_logger(), "Replanning due to lack of progress...");
          planPath();
        }
      }
    }
  }
}

bool PlannerNode::goalReached() const
{
  if (!goal_received_ || !odom_received_) {
    return false;
  }
  
  double dx = goal_pose_.pose.position.x - robot_pose_.position.x;
  double dy = goal_pose_.pose.position.y - robot_pose_.position.y;
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
  
  // Convert PoseStamped to PointStamped for planner
  geometry_msgs::msg::PointStamped goal_point;
  goal_point.header = goal_pose_.header;
  goal_point.point = goal_pose_.pose.position;
  
  nav_msgs::msg::Path path = planner_.planPath(current_map_, robot_pose_, goal_point);
  
  if (path.poses.empty()) {
    RCLCPP_WARN(this->get_logger(), "No valid path found to goal!");
  } else {
    RCLCPP_INFO(this->get_logger(), "Path found with %zu waypoints", path.poses.size());
    path_pub_->publish(path);
    last_plan_time_ = this->now();
    last_progress_pose_ = robot_pose_;
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
  return 0;
}
