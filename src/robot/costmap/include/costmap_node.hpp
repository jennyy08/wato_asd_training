#ifndef COSTMAP_NODE_HPP_
#define COSTMAP_NODE_HPP_

#include <cstdint>

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/string.hpp"
#include "costmap_core.hpp"

class CostmapNode : public rclcpp::Node {
 public:
  CostmapNode();

  void publishMessage();
  void lidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan);

 private:
  robot::CostmapCore costmap_;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr string_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  static constexpr float kResolution = 0.1F;
  // A larger local grid and inflation radius make the safety buffer visible
  // and give the planner more room to route around obstacles.
  static constexpr std::uint32_t kGridWidth = 300;
  static constexpr std::uint32_t kGridHeight = 300;
  static constexpr float kInflationRadius = 1.0F;
};

#endif
