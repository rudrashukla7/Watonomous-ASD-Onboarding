#ifndef PLANNER_NODE_HPP_
#define PLANNER_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <queue>
#include <unordered_map>
#include <vector>
#include <cmath>

#include "planner_core.hpp"

// ------------------- Supporting Structures -------------------

// 2D grid index
struct CellIndex {
  int x;
  int y;

  CellIndex(int xx, int yy) : x(xx), y(yy) {}
  CellIndex() : x(0), y(0) {}

  bool operator==(const CellIndex &other) const {
    return (x == other.x && y == other.y);
  }

  bool operator!=(const CellIndex &other) const {
    return (x != other.x || y != other.y);
  }
};

// Hash function for CellIndex
struct CellIndexHash {
  std::size_t operator()(const CellIndex &idx) const {
    return std::hash<int>()(idx.x) ^ (std::hash<int>()(idx.y) << 1);
  }
};

// A* Node structure
struct AStarNode {
  CellIndex index;
  double f_score;  // f = g + h

  AStarNode(CellIndex idx, double f) : index(idx), f_score(f) {}
};

// Comparator for priority queue (min-heap)
struct CompareF {
  bool operator()(const AStarNode &a, const AStarNode &b) {
    return a.f_score > b.f_score;
  }
};

// ------------------- Planner Node Class -------------------

class PlannerNode : public rclcpp::Node {
  public:
    PlannerNode();
    
  private:
    // State machine
    enum class State {
      WAITING_FOR_GOAL,
      WAITING_FOR_ROBOT_TO_REACH_GOAL
    };
    State state_;
    
    // Callbacks
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void timerCallback();
    
    // Planning functions
    void planPath();
    bool runAStar(CellIndex start, CellIndex goal, std::vector<CellIndex>& path);
    double heuristic(const CellIndex& a, const CellIndex& b);
    std::vector<CellIndex> getNeighbors(const CellIndex& cell);
    bool isValidCell(const CellIndex& cell);
    CellIndex worldToGrid(double x, double y);
    void gridToWorld(const CellIndex& cell, double& x, double& y);
    
    // Helper functions
    bool goalReached();
    void reconstructPath(
        const std::unordered_map<CellIndex, CellIndex, CellIndexHash>& came_from,
        CellIndex current,
        std::vector<CellIndex>& path);
    
    robot::PlannerCore planner_;
    
    // ROS constructs
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr goal_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    // Data storage
    nav_msgs::msg::OccupancyGrid current_map_;
    geometry_msgs::msg::PointStamped goal_;
    geometry_msgs::msg::Pose robot_pose_;
    
    // Flags
    bool goal_received_;
    bool map_received_;
    bool odom_received_;
    
    // Parameters
    double goal_tolerance_;  // Distance threshold to consider goal reached
    int obstacle_threshold_; // Occupancy value above which a cell is an obstacle
};

#endif