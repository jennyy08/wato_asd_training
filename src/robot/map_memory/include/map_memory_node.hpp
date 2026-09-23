#ifndef MAP_MEMORY_NODE_HPP_
#define MAP_MEMORY_NODE_HPP_

#include <cstdint>
#include <memory>

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"

#include "map_memory_core.hpp"

class MapMemoryNode : public rclcpp::Node {
  public:
    MapMemoryNode();

  private:
    void costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr costmap);
    void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr odometry);
    void updateMap();

    robot::MapMemoryCore map_memory_;

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    nav_msgs::msg::OccupancyGrid::SharedPtr latest_costmap_;
    nav_msgs::msg::Odometry::SharedPtr latest_odometry_;
    nav_msgs::msg::OccupancyGrid global_map_;
    bool map_initialized_{false};
    double last_update_x_{0.0};
    double last_update_y_{0.0};
    rclcpp::Time last_map_update_time_;

    static constexpr float kResolution = 0.1F;
    static constexpr std::uint32_t kMapWidth = 400;
    static constexpr std::uint32_t kMapHeight = 400;
    static constexpr double kUpdateDistance = 1.5;
    static constexpr double kPeriodicUpdateSeconds = 1.0;
};

#endif 
