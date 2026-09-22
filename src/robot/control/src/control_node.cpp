#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>

#include "control_node.hpp"

namespace {

double yawFromQuaternion(const geometry_msgs::msg::Quaternion &q) {
  return std::atan2(
      2.0 * (q.w * q.z + q.x * q.y),
      1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

}  // namespace

ControlNode::ControlNode()
    : Node("control"), control_(robot::ControlCore(this->get_logger())) {
  path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
      "/path", 10,
      std::bind(&ControlNode::pathCallback, this, std::placeholders::_1));
  odometry_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom/filtered", 10,
      std::bind(&ControlNode::odometryCallback, this,
                std::placeholders::_1));
  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10);
  timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&ControlNode::controlLoop, this));
}

void ControlNode::pathCallback(const nav_msgs::msg::Path::SharedPtr path) {
  latest_path_ = path;
}

void ControlNode::odometryCallback(
    const nav_msgs::msg::Odometry::SharedPtr odometry) {
  latest_odometry_ = odometry;
}

void ControlNode::publishStop() {
  cmd_vel_pub_->publish(geometry_msgs::msg::Twist());
}

void ControlNode::controlLoop() {
  if (!latest_path_ || latest_path_->poses.empty() || !latest_odometry_) {
    publishStop();
    return;
  }

  const auto &robot_pose = latest_odometry_->pose.pose;
  const double robot_x = robot_pose.position.x;
  const double robot_y = robot_pose.position.y;
  const double yaw = yawFromQuaternion(robot_pose.orientation);
  const auto &goal_pose = latest_path_->poses.back().pose;
  const double goal_distance = std::hypot(
      goal_pose.position.x - robot_x, goal_pose.position.y - robot_y);
  if (goal_distance < kGoalTolerance) {
    publishStop();
    return;
  }

  const geometry_msgs::msg::Pose *target = &goal_pose;
  for (const auto &pose_stamped : latest_path_->poses) {
    const double distance = std::hypot(
        pose_stamped.pose.position.x - robot_x,
        pose_stamped.pose.position.y - robot_y);
    if (distance >= kLookaheadDistance) {
      target = &pose_stamped.pose;
      break;
    }
  }

  const double dx = target->position.x - robot_x;
  const double dy = target->position.y - robot_y;
  const double target_x = std::cos(yaw) * dx + std::sin(yaw) * dy;
  const double target_y = -std::sin(yaw) * dx + std::cos(yaw) * dy;
  const double target_distance_squared =
      std::max(target_x * target_x + target_y * target_y, 1e-6);
  const double curvature = 2.0 * target_y / target_distance_squared;

  geometry_msgs::msg::Twist command;
  command.linear.x = kLinearSpeed;
  command.angular.z = std::clamp(
      kLinearSpeed * curvature, -kMaxAngularSpeed, kMaxAngularSpeed);
  cmd_vel_pub_->publish(command);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}
