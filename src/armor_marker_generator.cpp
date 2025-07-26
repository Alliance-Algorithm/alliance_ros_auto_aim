#include "armor_marker.hpp"
#include "enum/armor_id.hpp"
#include "enum/car_id.hpp"
#include <rclcpp/clock.hpp>
#include <rclcpp/time.hpp>
#include <string>

namespace world_exe::ros {

void ArmorMarkerGenerator::generate(
    const world_exe::interfaces::IArmorInCamera& armor_interface,
    const world_exe::enumeration::ArmorIdFlag& armor_id, const std::string& frame_id,
    visualization_msgs::msg::MarkerArray& marker_array) {

    ArmorMarkerInCameraSpacing armor_marker_camera;
    armor_marker_camera.set_frame_id(frame_id);
    armor_marker_camera.generate(armor_interface.GetArmors(armor_id), marker_array);

    // TODO:time stamp setting
}

void ArmorMarkerGenerator::generate(
    const world_exe::interfaces::IArmorInGimbalControl& armor_interface,
    const world_exe::enumeration::ArmorIdFlag& armor_id, const std::string& frame_id,
    visualization_msgs::msg::MarkerArray& marker_array) {

    ArmorMarkerInGimbalControlSpacing armor_marker_camera;
    armor_marker_camera.set_frame_id(frame_id);
    armor_marker_camera.generate(armor_interface.GetArmors(armor_id), marker_array);

    // TODO:time stamp setting
}
void ArmorMarkerGenerator::generate_all(
    const world_exe::interfaces::IArmorInGimbalControl& armor_interface,
    const std::string& frame_id, visualization_msgs::msg::MarkerArray& marker_array) {
    for (int i = 0; i < static_cast<int>(enumeration::ArmorIdFlag::Count); i++) {
        generate(
            armor_interface, static_cast<enumeration::CarIDFlag>(1 << i), frame_id, marker_array);
    }
}
void ArmorMarkerGenerator::generate_all(
    const world_exe::interfaces::IArmorInCamera& armor_interface, const std::string& frame_id,
    visualization_msgs::msg::MarkerArray& marker_array) {
    for (int i = 0; i < static_cast<int>(enumeration::ArmorIdFlag::Count); i++) {
        generate(
            armor_interface, static_cast<enumeration::CarIDFlag>(1 << i), frame_id, marker_array);
    }
}
} // namespace world_exe::ros