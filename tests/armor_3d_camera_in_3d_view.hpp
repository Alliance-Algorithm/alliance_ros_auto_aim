
#pragma once
#include "interfaces/armor_in_camera.hpp"
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace world_exetest::ros::test {}
static inline void armor_3d_camera_in_3d_view(
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
        marker_array_pub,
    const world_exe::interfaces::IArmorInCamera &armor_to_pub,
    void(*process(const world_exe::interfaces::IArmorInCamera &,
                  const std::string &id,
                  std::vector<visualization_msgs::msg::Marker> &))) {

  std::vector<visualization_msgs::msg::Marker> markers{};
  process(armor_to_pub, "armor_in_camera_test", markers);

  visualization_msgs::msg::MarkerArray msg;
  msg.set__markers(markers);
  marker_array_pub->publish(msg);
}