

#include <chrono>
#include <memory>
#include <string>

#include "./tests.hpp"

#include "interfaces/armor_in_camera.hpp"

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace world_exetest::ros::test {
static inline void armor_3d_camera_in_3d_view(
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
        marker_array_pub,
    const world_exe::interfaces::IArmorInCamera &armor_to_pub,
    void (*process)(const world_exe::interfaces::IArmorInCamera &,
                    const std::string &id,
                    std::vector<visualization_msgs::msg::Marker> &)) {

  std::vector<visualization_msgs::msg::Marker> markers{};
  process(armor_to_pub, "armor_in_camera_test", markers);

  visualization_msgs::msg::MarkerArray msg;
  msg.set__markers(markers);
  marker_array_pub->publish(msg);
}

class TestNode : public rclcpp::Node {
public:
  TestNode() : rclcpp::Node("alliance_ros_auto_aim_test") {
    pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        "/alliance_auto_aim/test/armor_3d_camera_in_3d_view", 10);
    timer_ = this->create_wall_timer(std::chrono::duration<long long>(1),
                                     [this]() { timer_callback(); });
  }

private:
  void timer_callback() {
    // world_exetest::ros::test::armor_3d_camera_in_3d_view({}, {});
    visualization_msgs::msg::MarkerArray msg{};
    pub_->publish(msg);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_;
};

int test_armor_3d_camera_in_3d_view(int argc, const char *const *argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TestNode>());
  rclcpp::shutdown();
  return 0;
}
} // namespace world_exetest::ros::test