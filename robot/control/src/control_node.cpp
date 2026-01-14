#include <chrono>
#include <memory>
#include <cmath>
#include <algorithm>

#include "control_node.hpp"

ControlNode::ControlNode()
    : Node("control"),
      control_(robot::ControlCore(this->get_logger())),
      lookahead_distance_(1),   // 1.5 meters lookahead
      goal_tolerance_(0.3),       // 0.3 meters to goal
      linear_speed_(1.25),         // 0.5 m/s forward speed
      max_angular_speed_(2.5),    // 1.0 rad/s max turn rate
      path_received_(false),
      odom_received_(false)
{
    // Subscribers
    path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
        "/path", 10,
        std::bind(&ControlNode::pathCallback, this, std::placeholders::_1));
    
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom/filtered", 10,
        std::bind(&ControlNode::odomCallback, this, std::placeholders::_1));
    
    // Publisher
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    
    // Control timer (10 Hz = 100ms)
    control_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&ControlNode::controlLoop, this));
    
    RCLCPP_INFO(this->get_logger(), "Control node initialized - Pure Pursuit controller active");
}

void ControlNode::pathCallback(const nav_msgs::msg::Path::SharedPtr msg) {
    current_path_ = msg;
    path_received_ = true;
    
    if (!msg->poses.empty()) {
        RCLCPP_INFO(this->get_logger(), "Received new path with %zu waypoints", 
                    msg->poses.size());
    }
}

void ControlNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    robot_odom_ = msg;
    odom_received_ = true;
}

void ControlNode::controlLoop() {
    // Check if we have necessary data
    if (!path_received_ || !odom_received_) {
        return;
    }
    
    if (!current_path_ || current_path_->poses.empty()) {
        // No path - stop the robot
        geometry_msgs::msg::Twist stop_cmd;
        stop_cmd.linear.x = 0.0;
        stop_cmd.angular.z = 0.0;
        cmd_vel_pub_->publish(stop_cmd);
        return;
    }
    
    // Check if goal is reached
    if (isGoalReached()) {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                            "Goal reached! Stopping robot.");
        geometry_msgs::msg::Twist stop_cmd;
        stop_cmd.linear.x = 0.0;
        stop_cmd.angular.z = 0.0;
        cmd_vel_pub_->publish(stop_cmd);
        return;
    }
    
    // Find lookahead point
    auto lookahead_point = findLookaheadPoint();
    if (!lookahead_point) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                            "No valid lookahead point found");
        return;
    }
    
    // Compute and publish velocity command
    auto cmd_vel = computeVelocity(*lookahead_point);
    cmd_vel_pub_->publish(cmd_vel);
    
    RCLCPP_DEBUG(this->get_logger(), "Publishing cmd_vel: linear=%.2f, angular=%.2f",
                cmd_vel.linear.x, cmd_vel.angular.z);
}

std::optional<geometry_msgs::msg::PoseStamped> ControlNode::findLookaheadPoint() {
    if (!current_path_ || current_path_->poses.empty() || !robot_odom_) {
        return std::nullopt;
    }
    
    // Get robot's current position
    geometry_msgs::msg::Point robot_pos = robot_odom_->pose.pose.position;
    
    // Find closest point on path
    int closest_idx = findClosestPointOnPath();
    
    // Search forward from closest point for lookahead point
    for (size_t i = closest_idx; i < current_path_->poses.size(); ++i) {
        const auto& pose = current_path_->poses[i];
        double distance = computeDistance(robot_pos, pose.pose.position);
        
        // If this point is at or beyond lookahead distance, use it
        if (distance >= lookahead_distance_) {
            return pose;
        }
    }
    
    // If no point found at lookahead distance, return the last point (goal)
    return current_path_->poses.back();
}

int ControlNode::findClosestPointOnPath() {
    if (!current_path_ || current_path_->poses.empty() || !robot_odom_) {
        return 0;
    }
    
    geometry_msgs::msg::Point robot_pos = robot_odom_->pose.pose.position;
    
    int closest_idx = 0;
    double min_distance = std::numeric_limits<double>::max();
    
    for (size_t i = 0; i < current_path_->poses.size(); ++i) {
        double distance = computeDistance(robot_pos, current_path_->poses[i].pose.position);
        if (distance < min_distance) {
            min_distance = distance;
            closest_idx = i;
        }
    }
    
    return closest_idx;
}

geometry_msgs::msg::Twist ControlNode::computeVelocity(
    const geometry_msgs::msg::PoseStamped& target) {
    
    geometry_msgs::msg::Twist cmd_vel;
    
    // Get robot's current position and orientation
    double robot_x = robot_odom_->pose.pose.position.x;
    double robot_y = robot_odom_->pose.pose.position.y;
    double robot_yaw = extractYaw(robot_odom_->pose.pose.orientation);
    
    // Get target position
    double target_x = target.pose.position.x;
    double target_y = target.pose.position.y;
    
    // Calculate angle to target
    double dx = target_x - robot_x;
    double dy = target_y - robot_y;
    double angle_to_target = std::atan2(dy, dx);
    
    // Calculate heading error
    double heading_error = angle_to_target - robot_yaw;
    
    // Normalize heading error to [-pi, pi]
    while (heading_error > M_PI) heading_error -= 2.0 * M_PI;
    while (heading_error < -M_PI) heading_error += 2.0 * M_PI;
    
    // Pure Pursuit: Calculate curvature
    // L is the distance to the lookahead point
    double L = std::sqrt(dx * dx + dy * dy);
    
    // Avoid division by zero
    if (L < 0.01) {
        cmd_vel.linear.x = 0.0;
        cmd_vel.angular.z = 0.0;
        return cmd_vel;
    }
    
    // Curvature formula: k = 2 * sin(alpha) / L
    // where alpha is the angle between robot heading and lookahead point
    double curvature = (2.0 * std::sin(heading_error)) / L;
    
    // Set linear velocity (constant)
    cmd_vel.linear.x = linear_speed_;
    
    // Set angular velocity based on curvature
    // omega = v * k
    cmd_vel.angular.z = linear_speed_ * curvature;
    
    // Clamp angular velocity
    cmd_vel.angular.z = std::max(-max_angular_speed_, 
                                 std::min(max_angular_speed_, cmd_vel.angular.z));
    
    // If heading error is large, reduce linear speed and increase turning
    if (std::abs(heading_error) > M_PI / 4) {  // More than 45 degrees
        cmd_vel.linear.x *= 0.5;  // Slow down when turning sharply
    }
    
    return cmd_vel;
}

bool ControlNode::isGoalReached() {
    if (!current_path_ || current_path_->poses.empty() || !robot_odom_) {
        return false;
    }
    
    // Get goal position (last point in path)
    const auto& goal = current_path_->poses.back();
    geometry_msgs::msg::Point robot_pos = robot_odom_->pose.pose.position;
    
    double distance = computeDistance(robot_pos, goal.pose.position);
    return distance < goal_tolerance_;
}

double ControlNode::computeDistance(const geometry_msgs::msg::Point& a,
                                   const geometry_msgs::msg::Point& b) {
    double dx = b.x - a.x;
    double dy = b.y - a.y;
    return std::sqrt(dx * dx + dy * dy);
}

double ControlNode::extractYaw(const geometry_msgs::msg::Quaternion& quat) {
    // Convert quaternion to yaw (rotation around z-axis)
    // Using the formula: yaw = atan2(2*(w*z + x*y), 1 - 2*(y^2 + z^2))
    double siny_cosp = 2.0 * (quat.w * quat.z + quat.x * quat.y);
    double cosy_cosp = 1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z);
    return std::atan2(siny_cosp, cosy_cosp);
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ControlNode>());
    rclcpp::shutdown();
    return 0;
}