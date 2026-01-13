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

    if (!current_path_ || !robot_odom_) {
        return cmd_vel;  
    }

    if (current_path_->poses.empty()) {
        return cmd_vel;
    }

    if (isGoalReached()) {
        return cmd_vel;  
    }

    auto lookahead_point = findLookaheadPoint();
    if (!lookahead_point) {
        return cmd_vel; 
    }

    cmd_vel = computeVelocity(*lookahead_point);

    return cmd_vel;
}

std::optional<geometry_msgs::msg::PoseStamped> PurePursuitController::findLookaheadPoint()
{
    const auto& robot_pos = robot_odom_->pose.pose.position;
    const auto& poses = current_path_->poses;

    if (poses.empty()) {
        return std::nullopt;
    }

    size_t closest_idx = 0;
    double closest_dist = std::numeric_limits<double>::infinity();
    for (size_t i = 0; i < poses.size(); ++i) {
        double d = computeDistance(robot_pos, poses[i].pose.position);
        if (d < closest_dist) {
            closest_dist = d;
            closest_idx = i;
        }
    }

    for (size_t i = closest_idx; i < poses.size(); ++i) {
        double d = computeDistance(robot_pos, poses[i].pose.position);
        if (d >= lookahead_distance_) {
            return poses[i];
        }
    }

    return poses.back();
}

geometry_msgs::msg::Twist PurePursuitController::computeVelocity(
    const geometry_msgs::msg::PoseStamped& target)
{
    geometry_msgs::msg::Twist cmd_vel;

    const auto& robot_pos = robot_odom_->pose.pose.position;
    const auto& robot_quat = robot_odom_->pose.pose.orientation;

    double robot_yaw = extractYaw(robot_quat);

    double dx = target.pose.position.x - robot_pos.x;
    double dy = target.pose.position.y - robot_pos.y;

    double target_angle = std::atan2(dy, dx);

    double steering_angle = target_angle - robot_yaw;

    while (steering_angle > M_PI) steering_angle -= 2 * M_PI;
    while (steering_angle < -M_PI) steering_angle += 2 * M_PI;

    if (std::abs(steering_angle) > M_PI / 2.0) {
        cmd_vel.linear.x = 0.0;

        double turn_rate = turn_gain_ * steering_angle;
        if (turn_rate > max_angular_speed_) turn_rate = max_angular_speed_;
        if (turn_rate < -max_angular_speed_) turn_rate = -max_angular_speed_;
        cmd_vel.angular.z = turn_rate;
        return cmd_vel;
    }

    double curvature = (2.0 * std::sin(steering_angle)) / lookahead_distance_;
    cmd_vel.linear.x = linear_speed_;
    double w = cmd_vel.linear.x * curvature;

    if (w > max_angular_speed_) w = max_angular_speed_;
    if (w < -max_angular_speed_) w = -max_angular_speed_;
    cmd_vel.angular.z = w;

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
