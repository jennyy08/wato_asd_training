#include <chrono>
#include <cmath>
#include <functional>

#include "map_memory_node.hpp"

namespace {

double yawFromQuaternion(const geometry_msgs::msg::Quaternion &q) {
  return std::atan2(
      2.0 * (q.w * q.z + q.x * q.y),
      1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

}  // namespace

MapMemoryNode::MapMemoryNode()
    : Node("map_memory"),
      map_memory_(robot::MapMemoryCore(this->get_logger())) {
  costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/costmap", 10,
      std::bind(&MapMemoryNode::costmapCallback, this,
                std::placeholders::_1));
  odometry_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom/filtered", 10,
      std::bind(&MapMemoryNode::odometryCallback, this,
                std::placeholders::_1));
  map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);
  timer_ = this->create_wall_timer(
      std::chrono::milliseconds(200),
      std::bind(&MapMemoryNode::updateMap, this));

  global_map_.header.frame_id = "map";
  global_map_.info.resolution = kResolution;
  global_map_.info.width = kMapWidth;
  global_map_.info.height = kMapHeight;
  global_map_.info.origin.position.x =
      -static_cast<double>(kMapWidth) * kResolution / 2.0;
  global_map_.info.origin.position.y =
      -static_cast<double>(kMapHeight) * kResolution / 2.0;
  global_map_.info.origin.orientation.w = 1.0;
  global_map_.data.assign(kMapWidth * kMapHeight, -1);
}

void MapMemoryNode::costmapCallback(
    const nav_msgs::msg::OccupancyGrid::SharedPtr costmap) {
  latest_costmap_ = costmap;
  RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000,
      "Received costmap (%u x %u)", costmap->info.width, costmap->info.height);
}

void MapMemoryNode::odometryCallback(
    const nav_msgs::msg::Odometry::SharedPtr odometry) {
  latest_odometry_ = odometry;
  RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000,
      "Received filtered odometry at (%.2f, %.2f)",
      odometry->pose.pose.position.x, odometry->pose.pose.position.y);
}

void MapMemoryNode::updateMap() {
  if (!latest_costmap_ || !latest_odometry_) {
    RCLCPP_INFO_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "Waiting for costmap and odometry before updating map");
    return;
  }

  const double robot_x = latest_odometry_->pose.pose.position.x;
  const double robot_y = latest_odometry_->pose.pose.position.y;
  const double distance = std::hypot(
      robot_x - last_update_x_, robot_y - last_update_y_);
  if (map_initialized_ && distance < kUpdateDistance) {
    return;
  }

  const auto &local = *latest_costmap_;
  const double yaw = yawFromQuaternion(latest_odometry_->pose.pose.orientation);
  const double cos_yaw = std::cos(yaw);
  const double sin_yaw = std::sin(yaw);
  const double local_resolution = local.info.resolution;
  const double local_origin_x = local.info.origin.position.x;
  const double local_origin_y = local.info.origin.position.y;

  for (std::uint32_t local_y = 0; local_y < local.info.height; ++local_y) {
    for (std::uint32_t local_x = 0; local_x < local.info.width; ++local_x) {
      const std::size_t local_index =
          static_cast<std::size_t>(local_y) * local.info.width + local_x;
      const auto value = local.data[local_index];
      if (value < 0) {
        continue;
      }

      const double x_local = local_origin_x +
          (static_cast<double>(local_x) + 0.5) * local_resolution;
      const double y_local = local_origin_y +
          (static_cast<double>(local_y) + 0.5) * local_resolution;
      const double x_global = robot_x + cos_yaw * x_local - sin_yaw * y_local;
      const double y_global = robot_y + sin_yaw * x_local + cos_yaw * y_local;
      const int global_x = static_cast<int>(std::floor(
          (x_global - global_map_.info.origin.position.x) / kResolution));
      const int global_y = static_cast<int>(std::floor(
          (y_global - global_map_.info.origin.position.y) / kResolution));
      if (global_x < 0 || global_x >= static_cast<int>(kMapWidth) ||
          global_y < 0 || global_y >= static_cast<int>(kMapHeight)) {
        continue;
      }

      const std::size_t global_index = static_cast<std::size_t>(global_y) *
          kMapWidth + global_x;
      if (global_map_.data[global_index] < value) {
        global_map_.data[global_index] = value;
      }
    }
  }

  global_map_.header.stamp = latest_odometry_->header.stamp;
  map_pub_->publish(global_map_);
  RCLCPP_INFO(
      this->get_logger(), "Published global map after robot moved %.2f m",
      distance);
  map_initialized_ = true;
  last_update_x_ = robot_x;
  last_update_y_ = robot_y;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapMemoryNode>());
  rclcpp::shutdown();
  return 0;
}
