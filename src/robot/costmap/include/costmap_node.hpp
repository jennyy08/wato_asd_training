#ifndef COSTMAP_NODE_HPP_
#define COSTMAP_NODE_HPP_

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
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

#endif
