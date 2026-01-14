#ifndef COSTMAP_NODE_HPP_
#define COSTMAP_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "costmap_core.hpp"

class CostmapNode : public rclcpp::Node {
public:
  CostmapNode();

private:
  // Callbacks
  void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan);
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

  // Internal helpers
  void initializeCostmap();
  void convertToGrid(double range, double angle, int &x_grid, int &y_grid);
  void markObstacle(int x_grid, int y_grid);
  void inflateObstacles();
  void publishCostmap();

  // Core object (given by the assignment)
  robot::CostmapCore costmap_;

  // ROS interfaces
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_pub_;

  // 2D costmap data (row-major: [y][x])
  std::vector<std::vector<int8_t>> costmap_data_;

  // Costmap parameters
  double resolution_;    // meters per cell
  int width_;            // number of cells in x
  int height_;           // number of cells in y
  double inflation_radius_;
  int max_cost_;

  // Robot pose (from odometry)
  double robot_x_;
  double robot_y_;
  bool   have_odom_;
};

#endif  // COSTMAP_NODE_HPP_