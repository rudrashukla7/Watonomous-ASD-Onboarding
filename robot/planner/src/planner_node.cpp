#include <chrono>
#include <memory>
#include <cmath>
#include <limits>
#include <algorithm>
#include <queue>
#include <unordered_map>
#include <vector>

#include "planner_node.hpp"

PlannerNode::PlannerNode()
    : Node("planner"),
      state_(State::WAITING_FOR_GOAL),
      planner_(robot::PlannerCore(this->get_logger())),
      goal_received_(false),
      map_received_(false),
      odom_received_(false),
      goal_tolerance_(0.5),   // 0.5 meters
      obstacle_threshold_(20)  // only cost == 0 is free
{
    // /map
    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/map", 10,
        std::bind(&PlannerNode::mapCallback, this, std::placeholders::_1));

    // /goal_point
    goal_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
        "/goal_point", 10,
        std::bind(&PlannerNode::goalCallback, this, std::placeholders::_1));

    // /odom/filtered
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom/filtered", 10,
        std::bind(&PlannerNode::odomCallback, this, std::placeholders::_1));

    // /path
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10);

    // Timer to check goal reached
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(500),
        std::bind(&PlannerNode::timerCallback, this));

    RCLCPP_INFO(this->get_logger(),
                "Planner node initialized - waiting for goal");
}

void PlannerNode::mapCallback(
    const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    current_map_ = *msg;
    map_received_ = true;

    if (state_ == State::WAITING_FOR_ROBOT_TO_REACH_GOAL) {
        RCLCPP_DEBUG(this->get_logger(),
                     "Map updated while navigating - replanning");
        planPath();
    }
}

void PlannerNode::goalCallback(
    const geometry_msgs::msg::PointStamped::SharedPtr msg) {
    goal_ = *msg;
    goal_received_ = true;
    state_ = State::WAITING_FOR_ROBOT_TO_REACH_GOAL;

    RCLCPP_INFO(this->get_logger(),
                "New goal received at (%.2f, %.2f) frame='%s'",
                goal_.point.x, goal_.point.y,
                goal_.header.frame_id.c_str());

    planPath();
}

void PlannerNode::odomCallback(
    const nav_msgs::msg::Odometry::SharedPtr msg) {
    robot_pose_ = msg->pose.pose;
    odom_received_ = true;
}

void PlannerNode::timerCallback() {
    if (state_ == State::WAITING_FOR_ROBOT_TO_REACH_GOAL) {
        if (goalReached()) {
            RCLCPP_INFO(this->get_logger(), "Goal reached!");
            state_ = State::WAITING_FOR_GOAL;
            goal_received_ = false;
        }
    }
}

bool PlannerNode::goalReached() {
    if (!odom_received_ || !goal_received_) {
        return false;
    }

    double dx = goal_.point.x - robot_pose_.position.x;
    double dy = goal_.point.y - robot_pose_.position.y;
    double distance = std::sqrt(dx * dx + dy * dy);

    return distance < goal_tolerance_;
}

void PlannerNode::planPath() {
    if (!goal_received_ || !map_received_ || !odom_received_) {
        RCLCPP_WARN(this->get_logger(),
                    "Cannot plan: missing goal (%d), map (%d), or odom (%d)",
                    goal_received_, map_received_, odom_received_);
        return;
    }

    if (current_map_.data.empty()) {
        RCLCPP_WARN(this->get_logger(), "Cannot plan: map is empty");
        return;
    }

    CellIndex start = worldToGrid(robot_pose_.position.x,
                                  robot_pose_.position.y);
    CellIndex goal  = worldToGrid(goal_.point.x,
                                  goal_.point.y);

    RCLCPP_INFO(this->get_logger(),
                "Planning from grid (%d, %d) to (%d, %d)",
                start.x, start.y, goal.x, goal.y);

    auto within_bounds = [this](const CellIndex& c) {
        return c.x >= 0 &&
               c.y >= 0 &&
               c.x < static_cast<int>(current_map_.info.width) &&
               c.y < static_cast<int>(current_map_.info.height);
    };

    if (!within_bounds(start) || !within_bounds(goal)) {
        RCLCPP_ERROR(this->get_logger(),
                     "Start or goal index out of map bounds");
        return;
    }

    int start_index = start.y * current_map_.info.width + start.x;
    int goal_index  = goal.y  * current_map_.info.width + goal.x;

    int8_t start_cost = current_map_.data[start_index];
    int8_t goal_cost  = current_map_.data[goal_index];

    RCLCPP_INFO(this->get_logger(),
                "Start cell cost: %d | Goal cell cost: %d",
                start_cost, goal_cost);

    if (!isValidCell(start)) {
        RCLCPP_ERROR(this->get_logger(),
                     "Start cell (%d, %d) invalid or not free (cost=%d)",
                     start.x, start.y, start_cost);
        return;
    }

    if (!isValidCell(goal)) {
        RCLCPP_ERROR(this->get_logger(),
                     "Goal cell (%d, %d) invalid or not free (cost=%d)",
                     goal.x, goal.y, goal_cost);
        return;
    }

    std::vector<CellIndex> path_cells;
    bool success = runAStar(start, goal, path_cells);

    if (!success) {
        RCLCPP_WARN(this->get_logger(),
                    "A* failed to find a path from (%d,%d) to (%d,%d)",
                    start.x, start.y, goal.x, goal.y);
        return;
    }

    nav_msgs::msg::Path path_msg;
    path_msg.header.stamp = this->now();
    path_msg.header.frame_id = "sim_world";

    for (const auto& cell : path_cells) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header = path_msg.header;

        double wx, wy;
        gridToWorld(cell, wx, wy);

        pose.pose.position.x = wx;
        pose.pose.position.y = wy;
        pose.pose.position.z = 0.0;
        pose.pose.orientation.w = 1.0;

        path_msg.poses.push_back(pose);
    }

    path_pub_->publish(path_msg);
    RCLCPP_INFO(this->get_logger(),
                "Published path with %zu waypoints", path_msg.poses.size());
}

bool PlannerNode::runAStar(
    CellIndex start,
    CellIndex goal,
    std::vector<CellIndex>& path) {

    std::priority_queue<AStarNode,
                        std::vector<AStarNode>,
                        CompareF> open_set;

    std::unordered_map<CellIndex, double, CellIndexHash> g_score;
    std::unordered_map<CellIndex, CellIndex, CellIndexHash> came_from;
    std::unordered_map<CellIndex, bool,   CellIndexHash> in_closed_set;

    g_score[start] = 0.0;
    double f_start = heuristic(start, goal);
    open_set.push(AStarNode(start, f_start));

    while (!open_set.empty()) {
        AStarNode current_node = open_set.top();
        open_set.pop();
        CellIndex current = current_node.index;

        if (current == goal) {
            reconstructPath(came_from, current, path);
            return true;
        }

        if (in_closed_set[current]) {
            continue;
        }
        in_closed_set[current] = true;

        std::vector<CellIndex> neighbors = getNeighbors(current);
        for (const auto& neighbor : neighbors) {
            if (in_closed_set[neighbor] || !isValidCell(neighbor)) {
                continue;
            }

            double tentative_g = g_score[current] + 1.0;

            if (g_score.find(neighbor) == g_score.end() ||
                tentative_g < g_score[neighbor]) {

                came_from[neighbor] = current;
                g_score[neighbor]   = tentative_g;

                double f_score = tentative_g + heuristic(neighbor, goal);
                open_set.push(AStarNode(neighbor, f_score));
            }
        }
    }

    return false;
}

double PlannerNode::heuristic(
    const CellIndex& a, const CellIndex& b) {
    int dx = b.x - a.x;
    int dy = b.y - a.y;
    return std::sqrt(dx * dx + dy * dy);
}

std::vector<CellIndex> PlannerNode::getNeighbors(
    const CellIndex& cell) {
    std::vector<CellIndex> neighbors;

    int dx[8] = {-1, 0, 1, -1, 1, -1, 0, 1};
    int dy[8] = {-1,-1,-1,  0, 0,  1, 1, 1};

    for (int i = 0; i < 8; ++i) {
        neighbors.emplace_back(cell.x + dx[i], cell.y + dy[i]);
    }

    return neighbors;
}

bool PlannerNode::isValidCell(const CellIndex& cell) {
    if (cell.x < 0 ||
        cell.y < 0 ||
        cell.x >= static_cast<int>(current_map_.info.width) ||
        cell.y >= static_cast<int>(current_map_.info.height)) {
        return false;
    }

    int index = cell.y * current_map_.info.width + cell.x;
    int8_t cost = current_map_.data[index];

    // Only cost == 0 is free; unknown or any positive cost is blocked
    if (cost == -1 || cost > obstacle_threshold_) {
        return false;
    }

    return true;
}

CellIndex PlannerNode::worldToGrid(double x, double y) {
    int gx = static_cast<int>(
        (x - current_map_.info.origin.position.x) /
        current_map_.info.resolution);
    int gy = static_cast<int>(
        (y - current_map_.info.origin.position.y) /
        current_map_.info.resolution);

    return CellIndex(gx, gy);
}

void PlannerNode::gridToWorld(
    const CellIndex& cell,
    double& x, double& y) {
    x = cell.x * current_map_.info.resolution +
        current_map_.info.origin.position.x;
    y = cell.y * current_map_.info.resolution +
        current_map_.info.origin.position.y;
}

void PlannerNode::reconstructPath(
    const std::unordered_map<CellIndex, CellIndex, CellIndexHash>& came_from,
    CellIndex current,
    std::vector<CellIndex>& path) {

    path.clear();
    path.push_back(current);

    while (came_from.find(current) != came_from.end()) {
        current = came_from.at(current);
        path.push_back(current);
    }

    std::reverse(path.begin(), path.end());
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PlannerNode>());
    rclcpp::shutdown();
    return 0;
}