#ifndef PURE_PURSUIT_CONTROLLER_HPP_
#define PURE_PURSUIT_CONTROLLER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <cmath>
#include <optional>

namespace robot
{

class PurePursuitController {
public:
    PurePursuitController(const rclcpp::Logger& logger);

    // Callback for path subscription
    void onPathReceived(const nav_msgs::msg::Path::SharedPtr msg);

    // Callback for odometry subscription
    void onOdomReceived(const nav_msgs::msg::Odometry::SharedPtr msg);

    // Main control loop - computes and returns velocity command
    geometry_msgs::msg::Twist computeCommand();

    // Setter for lookahead distance
    void setLookaheadDistance(double distance) { lookahead_distance_ = distance; }

    // Setter for linear speed
    void setLinearSpeed(double speed) { linear_speed_ = speed; }

    // Setter for goal tolerance
    void setGoalTolerance(double tolerance) { goal_tolerance_ = tolerance; }

private:
    // Helper methods
    std::optional<geometry_msgs::msg::PoseStamped> findLookaheadPoint();
    geometry_msgs::msg::Twist computeVelocity(const geometry_msgs::msg::PoseStamped& target);
    double computeDistance(const geometry_msgs::msg::Point& a, const geometry_msgs::msg::Point& b) const;
    double extractYaw(const geometry_msgs::msg::Quaternion& quat) const;
    bool isGoalReached() const;

    // Logger
    rclcpp::Logger logger_;

    // Data members
    nav_msgs::msg::Path::SharedPtr current_path_;
    nav_msgs::msg::Odometry::SharedPtr robot_odom_;

    // Parameters
    double lookahead_distance_;
    double goal_tolerance_;
    double linear_speed_;
};

}  // namespace robot

#endif  // PURE_PURSUIT_CONTROLLER_HPP_
