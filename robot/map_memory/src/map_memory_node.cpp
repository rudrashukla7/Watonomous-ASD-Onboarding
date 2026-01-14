#include <chrono>
#include <memory>
#include <cmath>

#include "map_memory_node.hpp"

MapMemoryNode::MapMemoryNode() 
    : Node("map_memory"),
      map_memory_(robot::MapMemoryCore(this->get_logger())),
      last_x_(0.0),
      last_y_(0.0),
      current_x_(0.0),
      current_y_(0.0),
      distance_threshold_(0.1),  // 1.5 meters
      global_map_resolution_(0.1),
      global_map_width_(3000),   // 300 meters
      global_map_height_(3000),  // 300 meters
      costmap_updated_(false),
      should_update_map_(false),
      global_map_initialized_(false),
      first_odom_received_(false)
{
    // Subscriber to /costmap
    costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/costmap", 10,
        std::bind(&MapMemoryNode::costmapCallback, this, std::placeholders::_1));

    // Subscriber to /odom/filtered
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom/filtered", 10,
        std::bind(&MapMemoryNode::odomCallback, this, std::placeholders::_1));

    // Publisher to /map
    map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);

    // Timer (1 Hz)
    timer_ = this->create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&MapMemoryNode::updateMap, this));

    // Initialize global map
    initializeGlobalMap();

    RCLCPP_INFO(this->get_logger(), "Map Memory node initialized");
}

void MapMemoryNode::initializeGlobalMap() {
    global_map_.header.frame_id = "sim_world";
    global_map_.info.resolution = global_map_resolution_;
    global_map_.info.width = global_map_width_;
    global_map_.info.height = global_map_height_;

    // Origin at center of world
    global_map_.info.origin.position.x =
        -(global_map_width_ * global_map_resolution_) / 2.0;
    global_map_.info.origin.position.y =
        -(global_map_height_ * global_map_resolution_) / 2.0;
    global_map_.info.origin.position.z = 0.0;
    global_map_.info.origin.orientation.w = 1.0;

    // *** CHANGED: initialize as FREE (0) instead of UNKNOWN (-1) ***
    global_map_.data.resize(global_map_width_ * global_map_height_, 0);

    global_map_initialized_ = true;
    RCLCPP_INFO(
        this->get_logger(),
        "Global map initialized: %dx%d cells",
        global_map_width_, global_map_height_);
}

void MapMemoryNode::costmapCallback(
    const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    latest_costmap_ = *msg;
    costmap_updated_ = true;
    RCLCPP_INFO(this->get_logger(), "Received costmap (%u x %u)",
                msg->info.width, msg->info.height);
}

void MapMemoryNode::odomCallback(
    const nav_msgs::msg::Odometry::SharedPtr msg) {
    current_x_ = msg->pose.pose.position.x;
    current_y_ = msg->pose.pose.position.y;
    latest_pose_ = msg->pose.pose;

    // First odom initializes last_x_/last_y_
    if (!first_odom_received_) {
        last_x_ = current_x_;
        last_y_ = current_y_;
        first_odom_received_ = true;
        RCLCPP_INFO(this->get_logger(),
                    "First odometry received at (%.2f, %.2f)",
                    current_x_, current_y_);
        return;
    }

    // Distance since last fusion
    double distance = calculateDistance(current_x_, current_y_,
                                        last_x_,   last_y_);

    if (distance >= distance_threshold_) {
        RCLCPP_INFO(this->get_logger(),
                    "Robot moved %.2f m since last map update, "
                    "flagging for fusion",
                    distance);
        last_x_ = current_x_;
        last_y_ = current_y_;
        should_update_map_ = true;
    }
}

double MapMemoryNode::calculateDistance(double x1, double y1,
                                        double x2, double y2) {
    return std::sqrt(std::pow(x2 - x1, 2) + std::pow(y2 - y1, 2));
}

void MapMemoryNode::updateMap() {
    static bool initial_map_published = false;

    // Initial map: as soon as we have BOTH odom and costmap
    if (!initial_map_published && costmap_updated_ && first_odom_received_) {
        RCLCPP_INFO(this->get_logger(),
                    "Publishing initial global map (first costmap + odom)...");
        integrateCostmap();
        global_map_.header.stamp = this->now();
        map_pub_->publish(global_map_);
        initial_map_published = true;
        costmap_updated_ = false;
        RCLCPP_INFO(this->get_logger(), "Initial global map published!");
        return;
    }

    // Subsequent updates: only if robot moved enough AND new costmap is ready
    if (should_update_map_ && costmap_updated_) {
        RCLCPP_INFO(this->get_logger(), "Updating global map with new costmap...");
        integrateCostmap();

        global_map_.header.stamp = this->now();
        map_pub_->publish(global_map_);

        should_update_map_ = false;
        costmap_updated_ = false;

        RCLCPP_INFO(this->get_logger(), "Global map updated and published");
    }
}

void MapMemoryNode::integrateCostmap() {
    if (!global_map_initialized_) {
        RCLCPP_WARN(this->get_logger(),
                    "Global map not initialized; cannot integrate costmap");
        return;
    }

    // Robot pose in world
    double robot_x = latest_pose_.position.x;
    double robot_y = latest_pose_.position.y;

    // Quaternion -> yaw (assuming planar motion)
    double qw = latest_pose_.orientation.w;
    double qz = latest_pose_.orientation.z;
    double robot_yaw = 2.0 * std::atan2(qz, qw);

    double cos_yaw = std::cos(robot_yaw);
    double sin_yaw = std::sin(robot_yaw);

    // Costmap metadata
    int costmap_width   = static_cast<int>(latest_costmap_.info.width);
    int costmap_height  = static_cast<int>(latest_costmap_.info.height);
    double costmap_res  = latest_costmap_.info.resolution;

    // Iterate through costmap cells
    for (int cy = 0; cy < costmap_height; ++cy) {
        for (int cx = 0; cx < costmap_width; ++cx) {
            int costmap_index = cy * costmap_width + cx;
            int8_t costmap_value = latest_costmap_.data[costmap_index];

            // Skip unknown cells
            if (costmap_value == -1) {
                continue;
            }

            // Local coordinates (centered)
            double local_x = (cx - costmap_width  / 2.0) * costmap_res;
            double local_y = (cy - costmap_height / 2.0) * costmap_res;

            // Rotate + translate to world frame
            double world_x = robot_x + (local_x * cos_yaw - local_y * sin_yaw);
            double world_y = robot_y + (local_x * sin_yaw + local_y * cos_yaw);

            // World -> global map cell
            int gx = static_cast<int>(
                (world_x - global_map_.info.origin.position.x) / global_map_resolution_);
            int gy = static_cast<int>(
                (world_y - global_map_.info.origin.position.y) / global_map_resolution_);

            if (gx < 0 || gx >= global_map_width_ ||
                gy < 0 || gy >= global_map_height_) {
                continue;
            }

            int global_index = gy * global_map_width_ + gx;

            // Linear fusion: new info overwrites old
            global_map_.data[global_index] = costmap_value;
        }
    }

    RCLCPP_INFO(this->get_logger(),
                "Costmap integrated at (%.2f, %.2f, %.2f rad)",
                robot_x, robot_y, robot_yaw);
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MapMemoryNode>());
    rclcpp::shutdown();
    return 0;
}