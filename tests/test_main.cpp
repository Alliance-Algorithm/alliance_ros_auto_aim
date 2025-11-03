
#include "core/system_factory.hpp"
#include "./image_and_data.hpp"
#include <memory>
#include <rclcpp/executors.hpp>
#include <rclcpp/utilities.hpp>

int main(int argc, const char *const *argv) {
  rclcpp::init(argc, argv);
  world_exe::core::SystemFactory::Build(world_exe::enumeration::SystemVersion::V2);
  auto node = std::make_shared<alliance_auto_aim::ros::bulldup::DataNode>("1", "2",[](){return cv::Mat();});
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
