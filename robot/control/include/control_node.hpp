#ifndef CONTROL_NODE_HPP_
#define CONTROL_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include <cmath>
#include <optional>
#include <vector>

#include "control_core.hpp"

class ControlNode : public rclcpp::Node {
  public:
    ControlNode();
    
  private:
    // Callbacks
    void pathCallback(const nav_msgs::msg::Path::SharedPtr msg);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void controlLoop();
    
    // Pure Pursuit functions
    std::optional<geometry_msgs::msg::PoseStamped> findLookaheadPoint();
    geometry_msgs::msg::Twist computeVelocity(const geometry_msgs::msg::PoseStamped& target);
    
    // Helper functions
    double computeDistance(const geometry_msgs::msg::Point& a, 
                          const geometry_msgs::msg::Point& b);
    double extractYaw(const geometry_msgs::msg::Quaternion& quat);
    bool isGoalReached();
    int findClosestPointOnPath();
    
    robot::ControlCore control_;
    
    // ROS constructs
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::TimerBase::SharedPtr control_timer_;
    
    // Data storage
    nav_msgs::msg::Path::SharedPtr current_path_;
    nav_msgs::msg::Odometry::SharedPtr robot_odom_;
    
    // Parameters
    double lookahead_distance_;  // Distance to lookahead point
    double goal_tolerance_;      // Distance to consider goal reached
    double linear_speed_;        // Constant forward speed
    double max_angular_speed_;   // Maximum angular velocity
    
    // State
    bool path_received_;
    bool odom_received_;
};

#endif