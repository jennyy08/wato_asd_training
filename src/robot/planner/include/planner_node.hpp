#ifndef PLANNER_NODE_HPP_
#define PLANNER_NODE_HPP_

#include <cstdint>
#include <memory>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"

#include "planner_core.hpp"

class PlannerNode : public rclcpp::Node {
  public:
    PlannerNode();

  private:
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr map);
    void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr odometry);
    void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr goal);
    void planPath();

    robot::PlannerCore planner_;

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

    nav_msgs::msg::OccupancyGrid::SharedPtr latest_map_;
    nav_msgs::msg::Odometry::SharedPtr latest_odometry_;
    geometry_msgs::msg::PoseStamped::SharedPtr latest_goal_;
};

#endif 
