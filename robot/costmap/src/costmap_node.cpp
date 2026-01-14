#include <chrono>
#include <memory>
#include <cmath>
#include <algorithm>

#include "costmap_node.hpp"

CostmapNode::CostmapNode()
    : Node("costmap"),
      costmap_(robot::CostmapCore(this->get_logger())),
      resolution_(0.1),        // 0.1 m per cell
      width_(300),             // 30 m wide
      height_(300),            // 30 m tall
      inflation_radius_(2),  // increased from 0.7 to 1.0
      max_cost_(100),
      robot_x_(0.0),
      robot_y_(0.0),
      have_odom_(false)
{
  // Laser subscriber
  laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/lidar", 10,
      std::bind(&CostmapNode::laserCallback, this, std::placeholders::_1));

  // Odometry subscriber
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom/filtered", 10,
      std::bind(&CostmapNode::odomCallback, this, std::placeholders::_1));

  // Costmap publisher
  costmap_pub_ =
      this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 10);

  // Allocate 2D costmap
  costmap_data_.resize(height_);
  for (int y = 0; y < height_; ++y) {
    costmap_data_[y].assign(width_, 0);
  }

  RCLCPP_INFO(this->get_logger(),
              "Costmap node initialized with 2D array [%d x %d]",
              height_, width_);
}

void CostmapNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  robot_x_ = msg->pose.pose.position.x;
  robot_y_ = msg->pose.pose.position.y;
  have_odom_ = true;
}

void CostmapNode::laserCallback(
    const sensor_msgs::msg::LaserScan::SharedPtr scan) {
  if (!have_odom_) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                         "No odometry yet; skipping costmap update");
    return;
  }

  RCLCPP_INFO(this->get_logger(),
              "Received laser scan with %zu points", scan->ranges.size());

  initializeCostmap();

  for (size_t i = 0; i < scan->ranges.size(); ++i) {
    double angle = scan->angle_min + i * scan->angle_increment;
    double range = scan->ranges[i];

    if (range < scan->range_max && range > scan->range_min) {
      int x_grid, y_grid;
      convertToGrid(range, angle, x_grid, y_grid);

      if (x_grid >= 0 && x_grid < width_ &&
          y_grid >= 0 && y_grid < height_) {
        markObstacle(x_grid, y_grid);
      }
    }
  }

  inflateObstacles();
  publishCostmap();

  RCLCPP_INFO(this->get_logger(), "Published costmap");
}

void CostmapNode::initializeCostmap() {
  for (int y = 0; y < height_; ++y) {
    for (int x = 0; x < width_; ++x) {
      costmap_data_[y][x] = 0;
    }
  }
}

void CostmapNode::convertToGrid(double range,
                                double angle,
                                int &x_grid,
                                int &y_grid) {
  double x_r = range * std::cos(angle);
  double y_r = range * std::sin(angle);

  double x_cell = x_r / resolution_ + static_cast<double>(width_) / 2.0;
  double y_cell = y_r / resolution_ + static_cast<double>(height_) / 2.0;

  x_grid = static_cast<int>(std::floor(x_cell));
  y_grid = static_cast<int>(std::floor(y_cell));
}

void CostmapNode::markObstacle(int x_grid, int y_grid) {
  costmap_data_[y_grid][x_grid] = max_cost_;
}

void CostmapNode::inflateObstacles() {
  std::vector<std::vector<int8_t>> original = costmap_data_;

  int inflation_cells =
      static_cast<int>(inflation_radius_ / resolution_);

  for (int y = 0; y < height_; ++y) {
    for (int x = 0; x < width_; ++x) {
      if (original[y][x] != max_cost_) {
        continue;
      }

      for (int dy = -inflation_cells; dy <= inflation_cells; ++dy) {
        for (int dx = -inflation_cells; dx <= inflation_cells; ++dx) {
          int nx = x + dx;
          int ny = y + dy;

          if (nx < 0 || nx >= width_ ||
              ny < 0 || ny >= height_) {
            continue;
          }

          double distance =
              std::sqrt(static_cast<double>(dx * dx + dy * dy)) *
              resolution_;

          if (distance > inflation_radius_) {
            continue;
          }

          int cost = static_cast<int>(
              max_cost_ * (1.0 - distance / inflation_radius_));

          costmap_data_[ny][nx] = std::max(costmap_data_[ny][nx],
                                           static_cast<int8_t>(cost));
        }
      }
    }
  }
}

void CostmapNode::publishCostmap() {
  nav_msgs::msg::OccupancyGrid grid_msg;

  grid_msg.header.stamp = this->now();
  grid_msg.header.frame_id = "sim_world";

  grid_msg.info.resolution = resolution_;
  grid_msg.info.width = width_;
  grid_msg.info.height = height_;

  double origin_x = robot_x_ - (width_ * resolution_) / 2.0;
  double origin_y = robot_y_ - (height_ * resolution_) / 2.0;

  grid_msg.info.origin.position.x = origin_x;
  grid_msg.info.origin.position.y = origin_y;
  grid_msg.info.origin.position.z = 0.0;
  grid_msg.info.origin.orientation.w = 1.0;

  grid_msg.data.resize(width_ * height_);
  for (int y = 0; y < height_; ++y) {
    for (int x = 0; x < width_; ++x) {
      int idx = y * width_ + x;
      grid_msg.data[idx] = costmap_data_[y][x];
    }
  }

  costmap_pub_->publish(grid_msg);
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}