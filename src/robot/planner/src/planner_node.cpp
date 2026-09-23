#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <queue>
#include <unordered_map>
#include <vector>

#include "planner_node.hpp"

namespace {

struct QueueEntry {
  double priority;
  std::size_t index;

  bool operator>(const QueueEntry &other) const {
    return priority > other.priority;
  }
};

std::size_t worldToIndex(
    double x, double y, const nav_msgs::msg::OccupancyGrid &map) {
  const int grid_x = static_cast<int>(std::floor(
      (x - map.info.origin.position.x) / map.info.resolution));
  const int grid_y = static_cast<int>(std::floor(
      (y - map.info.origin.position.y) / map.info.resolution));
  if (grid_x < 0 || grid_y < 0 ||
      grid_x >= static_cast<int>(map.info.width) ||
      grid_y >= static_cast<int>(map.info.height)) {
    return std::numeric_limits<std::size_t>::max();
  }
  return static_cast<std::size_t>(grid_y) * map.info.width + grid_x;
}

}  // namespace

PlannerNode::PlannerNode()
    : Node("planner"), planner_(robot::PlannerCore(this->get_logger())) {
  map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/map", 10,
      std::bind(&PlannerNode::mapCallback, this, std::placeholders::_1));
  odometry_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom/filtered", 10,
      std::bind(&PlannerNode::odometryCallback, this,
                std::placeholders::_1));
  goal_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
      "/goal_point", 10,
      std::bind(&PlannerNode::goalCallback, this, std::placeholders::_1));
  path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10);
  timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&PlannerNode::timerCallback, this));
}

void PlannerNode::mapCallback(
    const nav_msgs::msg::OccupancyGrid::SharedPtr map) {
  latest_map_ = map;
  if (state_ == State::WAITING_FOR_ROBOT_TO_REACH_GOAL) {
    planPath();
  }
}

void PlannerNode::odometryCallback(
    const nav_msgs::msg::Odometry::SharedPtr odometry) {
  latest_odometry_ = odometry;
}

void PlannerNode::goalCallback(
    const geometry_msgs::msg::PointStamped::SharedPtr goal) {
  latest_goal_ = goal;
  state_ = State::WAITING_FOR_ROBOT_TO_REACH_GOAL;
  goal_start_time_ = this->now();
  // Do not let the controller continue following a route for an older goal
  // while this goal is being validated and planned.
  publishEmptyPath();
  planPath();
}

void PlannerNode::timerCallback() {
  if (state_ != State::WAITING_FOR_ROBOT_TO_REACH_GOAL) {
    return;
  }

  if (goalReached()) {
    RCLCPP_INFO(this->get_logger(), "Goal reached");
    publishEmptyPath();
    state_ = State::WAITING_FOR_GOAL;
    return;
  }

  if ((this->now() - goal_start_time_).seconds() >= kPlanTimeoutSeconds) {
    RCLCPP_INFO(this->get_logger(), "Planner timeout; replanning");
    goal_start_time_ = this->now();
    planPath();
  }
}

bool PlannerNode::goalReached() const {
  if (!latest_goal_ || !latest_odometry_) {
    return false;
  }
  const double dx = latest_goal_->point.x -
      latest_odometry_->pose.pose.position.x;
  const double dy = latest_goal_->point.y -
      latest_odometry_->pose.pose.position.y;
  return std::hypot(dx, dy) < kGoalTolerance;
}

void PlannerNode::planPath() {
  if (state_ != State::WAITING_FOR_ROBOT_TO_REACH_GOAL ||
      !latest_map_ || !latest_odometry_ || !latest_goal_) {
    return;
  }

  const auto &map = *latest_map_;
  const std::size_t start = worldToIndex(
      latest_odometry_->pose.pose.position.x,
      latest_odometry_->pose.pose.position.y, map);
  const std::size_t goal = worldToIndex(
      latest_goal_->point.x, latest_goal_->point.y, map);
  if (start == std::numeric_limits<std::size_t>::max() ||
      goal == std::numeric_limits<std::size_t>::max()) {
    publishEmptyPath();
    RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "Robot or goal is outside the global map");
    return;
  }

  const auto traversable = [&map](std::size_t index) {
    return index < map.data.size() && map.data[index] >= 0 && map.data[index] < 50;
  };
  if (!traversable(start) || !traversable(goal)) {
    publishEmptyPath();
    RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "Robot or goal cell is occupied or unknown");
    return;
  }

  const auto coordinates = [&map](std::size_t index) {
    return std::pair<int, int>{
        static_cast<int>(index % map.info.width),
        static_cast<int>(index / map.info.width)};
  };
  const auto heuristic = [&coordinates, goal](std::size_t index) {
    const auto [x, y] = coordinates(index);
    const auto [goal_x, goal_y] = coordinates(goal);
    return std::hypot(goal_x - x, goal_y - y);
  };

  const std::array<std::pair<int, int>, 8> directions = {{
      {1, 0}, {-1, 0}, {0, 1}, {0, -1},
      {1, 1}, {1, -1}, {-1, 1}, {-1, -1}}};
  const double infinity = std::numeric_limits<double>::infinity();
  std::vector<double> cost(map.data.size(), infinity);
  std::unordered_map<std::size_t, std::size_t> parent;
  std::priority_queue<QueueEntry, std::vector<QueueEntry>, std::greater<>> open;
  cost[start] = 0.0;
  open.push({heuristic(start), start});

  while (!open.empty()) {
    const auto current = open.top().index;
    open.pop();
    if (current == goal) {
      break;
    }

    const auto [current_x, current_y] = coordinates(current);
    for (const auto &[dx, dy] : directions) {
      const int next_x = current_x + dx;
      const int next_y = current_y + dy;
      if (next_x < 0 || next_y < 0 ||
          next_x >= static_cast<int>(map.info.width) ||
          next_y >= static_cast<int>(map.info.height)) {
        continue;
      }
      const std::size_t next = static_cast<std::size_t>(next_y) *
          map.info.width + next_x;
      if (!traversable(next)) {
        continue;
      }
      const double step_cost = (dx != 0 && dy != 0) ? 1.4142 : 1.0;
      const double new_cost = cost[current] + step_cost;
      if (new_cost < cost[next]) {
        cost[next] = new_cost;
        parent[next] = current;
        open.push({new_cost + heuristic(next), next});
      }
    }
  }

  if (start != goal && parent.find(goal) == parent.end()) {
    publishEmptyPath();
    RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "A* could not find a path to the goal");
    return;
  }

  std::vector<std::size_t> indices{goal};
  while (indices.back() != start) {
    indices.push_back(parent.at(indices.back()));
  }
  std::reverse(indices.begin(), indices.end());

  nav_msgs::msg::Path path;
  path.header = map.header;
  path.header.stamp = this->now();
  path.poses.reserve(indices.size());
  for (const auto index : indices) {
    const auto [grid_x, grid_y] = coordinates(index);
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position.x = map.info.origin.position.x +
        (grid_x + 0.5) * map.info.resolution;
    pose.pose.position.y = map.info.origin.position.y +
        (grid_y + 0.5) * map.info.resolution;
    pose.pose.orientation.w = 1.0;
    path.poses.push_back(pose);
  }
  path_pub_->publish(path);
}

void PlannerNode::publishEmptyPath() {
  nav_msgs::msg::Path empty_path;
  if (latest_map_) {
    empty_path.header = latest_map_->header;
  }
  empty_path.header.stamp = this->now();
  path_pub_->publish(empty_path);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
  return 0;
}
