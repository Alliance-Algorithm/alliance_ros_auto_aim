#include "armor_marker.hpp"
#include <rclcpp/clock.hpp>
#include <rclcpp/time.hpp>
#include <string>

namespace world_exe::ros {

void ArmorMarkerGenerator::generate(
    const world_exe::interfaces::IArmorInCamera& armor_interface, const world_exe::enumeration::ArmorIdFlag& armor_id,
    const std::string& frame_id, visualization_msgs::msg::MarkerArray& marker_array) {

    ArmorMarkerInCameraSpacing armor_marker_camera;
    armor_marker_camera.set_frame_id(frame_id);
    marker_array = armor_marker_camera.generate(armor_interface.GetArmors(armor_id));

    // TODO:time stamp setting
}

void ArmorMarkerGenerator::generate(
    const world_exe::interfaces::IArmorInGimbalControl& armor_interface,
    const world_exe::enumeration::ArmorIdFlag& armor_id, const std::string& frame_id,
    visualization_msgs::msg::MarkerArray& marker_array) {

    ArmorMarkerInGimbalControlSpacing armor_marker_camera;
    armor_marker_camera.set_frame_id(frame_id);
    marker_array = armor_marker_camera.generate(armor_interface.GetArmors(armor_id));

    // TODO:time stamp setting
}
} // namespace world_exe::ros