#include "pure_pursuit.controller.hpp"
#include <algorithm>

namespace robot
{

PurePursuitController::PurePursuitController(const rclcpp::Logger& logger)
    : logger_(logger),
      current_path_(nullptr),
      robot_odom_(nullptr),
      lookahead_distance_(1.0),
      goal_tolerance_(0.1),
      linear_speed_(0.5)
{
}

void PurePursuitController::onPathReceived(const nav_msgs::msg::Path::SharedPtr msg)
{
    current_path_ = msg;
}

void PurePursuitController::onOdomReceived(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    robot_odom_ = msg;
}

geometry_msgs::msg::Twist PurePursuitController::computeCommand()
{
    geometry_msgs::msg::Twist cmd_vel;

    // Check if we have necessary data
    if (!current_path_ || !robot_odom_) {
        return cmd_vel;  // Return zero velocity if no data
    }

    // Check if path is empty
    if (current_path_->poses.empty()) {
        return cmd_vel;
    }

    // Check if goal is reached
    if (isGoalReached()) {
        return cmd_vel;  // Stop the robot
    }

    // Find the lookahead point
    auto lookahead_point = findLookaheadPoint();
    if (!lookahead_point) {
        return cmd_vel;  // No valid lookahead point found
    }

    // Compute velocity command
    cmd_vel = computeVelocity(*lookahead_point);

    return cmd_vel;
}

std::optional<geometry_msgs::msg::PoseStamped> PurePursuitController::findLookaheadPoint()
{
    const auto& robot_pos = robot_odom_->pose.pose.position;
    const auto& poses = current_path_->poses;

    // Find the first point that is beyond the lookahead distance
    for (size_t i = 0; i < poses.size(); ++i) {
        double distance = computeDistance(robot_pos, poses[i].pose.position);
        if (distance >= lookahead_distance_) {
            return poses[i];
        }
    }

    // If no point is beyond lookahead distance, return the last point
    if (!poses.empty()) {
        return poses.back();
    }

    return std::nullopt;
}

geometry_msgs::msg::Twist PurePursuitController::computeVelocity(
    const geometry_msgs::msg::PoseStamped& target)
{
    geometry_msgs::msg::Twist cmd_vel;

    const auto& robot_pos = robot_odom_->pose.pose.position;
    const auto& robot_quat = robot_odom_->pose.pose.orientation;

    // Extract robot's current yaw
    double robot_yaw = extractYaw(robot_quat);

    // Calculate vector from robot to target
    double dx = target.pose.position.x - robot_pos.x;
    double dy = target.pose.position.y - robot_pos.y;

    // Calculate angle to target
    double target_angle = std::atan2(dy, dx);

    // Calculate steering angle (difference between target angle and robot's heading)
    double steering_angle = target_angle - robot_yaw;

    // Normalize steering angle to [-pi, pi]
    while (steering_angle > M_PI) steering_angle -= 2 * M_PI;
    while (steering_angle < -M_PI) steering_angle += 2 * M_PI;

    // Compute curvature for the circular arc
    // Using Pure Pursuit formula: curvature = (2 * sin(steering_angle)) / lookahead_distance
    double curvature = (2.0 * std::sin(steering_angle)) / lookahead_distance_;

    // Set linear velocity (constant forward speed)
    cmd_vel.linear.x = linear_speed_;

    // Set angular velocity based on curvature
    // angular_velocity = linear_velocity * curvature
    cmd_vel.angular.z = linear_speed_ * curvature;

    return cmd_vel;
}

double PurePursuitController::computeDistance(
    const geometry_msgs::msg::Point& a, 
    const geometry_msgs::msg::Point& b) const
{
    double dx = a.x - b.x;
    double dy = a.y - b.y;
    return std::sqrt(dx * dx + dy * dy);
}

double PurePursuitController::extractYaw(const geometry_msgs::msg::Quaternion& quat) const
{
    // Convert quaternion to yaw using the formula from map_memory
    const double siny_cosp = 2.0 * (quat.w * quat.z + quat.x * quat.y);
    const double cosy_cosp = 1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z);
    return std::atan2(siny_cosp, cosy_cosp);
}

bool PurePursuitController::isGoalReached() const
{
    if (!current_path_ || current_path_->poses.empty()) {
        return false;
    }

    const auto& robot_pos = robot_odom_->pose.pose.position;
    const auto& goal_pos = current_path_->poses.back().pose.position;

    double distance_to_goal = computeDistance(robot_pos, goal_pos);
    return distance_to_goal <= goal_tolerance_;
}

}  // namespace robot
