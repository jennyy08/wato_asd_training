#ifndef CONTROL_NODE_HPP_
#define CONTROL_NODE_HPP_

#include <memory>

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"

#include "control_core.hpp"

class ControlNode : public rclcpp::Node {
  public:
    ControlNode();

  private:
    void pathCallback(const nav_msgs::msg::Path::SharedPtr path);
    void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr odometry);
    void controlLoop();
    void publishStop();

    robot::ControlCore control_;

    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    nav_msgs::msg::Path::SharedPtr latest_path_;
    nav_msgs::msg::Odometry::SharedPtr latest_odometry_;

    static constexpr double kLookaheadDistance = 0.7;
    static constexpr double kGoalTolerance = 0.3;
    static constexpr double kLinearSpeed = 0.5;
    static constexpr double kMaxAngularSpeed = 1.5;
};

#endif
