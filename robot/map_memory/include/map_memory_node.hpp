#ifndef MAP_MEMORY_NODE_HPP_
#define MAP_MEMORY_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include <cmath>

#include "map_memory_core.hpp"

class MapMemoryNode : public rclcpp::Node {
  public:
    MapMemoryNode();
    
  private:
    // Callback functions
    void costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void updateMap();
    
    // Helper functions
    void integrateCostmap();
    void initializeGlobalMap();
    double calculateDistance(double x1, double y1, double x2, double y2);
    void transformAndMergeCostmap();
    
    robot::MapMemoryCore map_memory_;
    
    // ROS constructs
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    // Map and state variables
    nav_msgs::msg::OccupancyGrid global_map_;
    nav_msgs::msg::OccupancyGrid latest_costmap_;
    geometry_msgs::msg::Pose latest_pose_;
    
    // Position tracking
    double last_x_;
    double last_y_;
    double current_x_;
    double current_y_;
    
    // Parameters
    double distance_threshold_;  // 1.5 meters
    double global_map_resolution_;
    int global_map_width_;
    int global_map_height_;
    
    // Flags
    bool costmap_updated_;
    bool should_update_map_;
    bool global_map_initialized_;
    bool first_odom_received_;
};

#endif