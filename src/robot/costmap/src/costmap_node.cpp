#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>

#include "costmap_node.hpp"

CostmapNode::CostmapNode()
    : Node("costmap"),
      costmap_(robot::CostmapCore(this->get_logger())) {
  string_pub_ =
      this->create_publisher<std_msgs::msg::String>("/test_topic", 10);
  costmap_pub_ =
      this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 10);

  lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/lidar", rclcpp::SensorDataQoS(),
      std::bind(&CostmapNode::lidarCallback, this, std::placeholders::_1));

  timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&CostmapNode::publishMessage, this));
}

void CostmapNode::lidarCallback(
    const sensor_msgs::msg::LaserScan::SharedPtr scan) {
  nav_msgs::msg::OccupancyGrid costmap;
  costmap.header = scan->header;
  costmap.info.resolution = kResolution;
  costmap.info.width = kGridWidth;
  costmap.info.height = kGridHeight;
  costmap.info.origin.position.x =
      -static_cast<double>(kGridWidth) * kResolution / 2.0;
  costmap.info.origin.position.y =
      -static_cast<double>(kGridHeight) * kResolution / 2.0;
  costmap.info.origin.orientation.w = 1.0;
  costmap.data.assign(kGridWidth * kGridHeight, 0);

  const int inflation_cells = static_cast<int>(
      std::ceil(kInflationRadius / kResolution));
  const double origin_x = costmap.info.origin.position.x;
  const double origin_y = costmap.info.origin.position.y;
  std::size_t occupied_cells = 0;

  for (std::size_t i = 0; i < scan->ranges.size(); ++i) {
    const float range = scan->ranges[i];
    if (!std::isfinite(range) || range < scan->range_min ||
        range > scan->range_max) {
      continue;
    }

    const double angle = scan->angle_min +
        static_cast<double>(i) * scan->angle_increment;
    const double x = range * std::cos(angle);
    const double y = range * std::sin(angle);
    const int grid_x = static_cast<int>(std::floor((x - origin_x) / kResolution));
    const int grid_y = static_cast<int>(std::floor((y - origin_y) / kResolution));

    if (grid_x < 0 || grid_x >= static_cast<int>(kGridWidth) ||
        grid_y < 0 || grid_y >= static_cast<int>(kGridHeight)) {
      continue;
    }

    for (int dy = -inflation_cells; dy <= inflation_cells; ++dy) {
      for (int dx = -inflation_cells; dx <= inflation_cells; ++dx) {
        const int inflated_x = grid_x + dx;
        const int inflated_y = grid_y + dy;
        if (inflated_x < 0 || inflated_x >= static_cast<int>(kGridWidth) ||
            inflated_y < 0 || inflated_y >= static_cast<int>(kGridHeight)) {
          continue;
        }

        const double distance = std::hypot(dx, dy) * kResolution;
        if (distance > kInflationRadius) {
          continue;
        }

        const auto cost = static_cast<int8_t>(std::round(
            100.0 * (1.0 - distance / kInflationRadius)));
        const auto index = static_cast<std::size_t>(
            inflated_y * static_cast<int>(kGridWidth) + inflated_x);
        if (cost > costmap.data[index]) {
          costmap.data[index] = cost;
          ++occupied_cells;
        }
      }
    }
  }

  costmap_pub_->publish(costmap);

  RCLCPP_DEBUG(
      this->get_logger(), "Published %ux%u costmap from %zu laser ranges",
      kGridWidth, kGridHeight, scan->ranges.size());
  RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000,
      "Costmap contains %zu occupied or inflated cells", occupied_cells);
}

void CostmapNode::publishMessage() {
  auto message = std_msgs::msg::String();
  message.data = "Hello, ROS 2!";

  RCLCPP_INFO(
      this->get_logger(),
      "Publishing: '%s'",
      message.data.c_str());

  string_pub_->publish(message);
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();

  return 0;
}
