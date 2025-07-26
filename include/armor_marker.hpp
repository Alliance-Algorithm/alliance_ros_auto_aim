#pragma once
#include <memory>
#include <rclcpp/clock.hpp>
#include <rclcpp/time.hpp>
#include <string>

#include "data/armor_camera_spacing.hpp"
#include "data/armor_gimbal_control_spacing.hpp"
#include "interfaces/armor_in_camera.hpp"
#include "interfaces/armor_in_gimbal_control.hpp"

#include <visualization_msgs/msg/detail/marker_array__struct.hpp>
namespace world_exe::ros {

class ArmorMarkerGenerator {
public:
    static void generate(
        const world_exe::interfaces::IArmorInCamera&, const world_exe::enumeration::ArmorIdFlag&,
        const std::string&, visualization_msgs::msg::MarkerArray&);

    static void generate(
        const world_exe::interfaces::IArmorInGimbalControl&,
        const world_exe::enumeration::ArmorIdFlag&, const std::string&,
        visualization_msgs::msg::MarkerArray&);
    static void generate_all(
        const world_exe::interfaces::IArmorInCamera&, const std::string&,
        visualization_msgs::msg::MarkerArray&);
    static void generate_all(
        const world_exe::interfaces::IArmorInGimbalControl&, const std::string&,
        visualization_msgs::msg::MarkerArray&);
};

class ArmorMarkerInCameraSpacing {
public:
    ArmorMarkerInCameraSpacing();

    ~ArmorMarkerInCameraSpacing();

    void set_frame_id(const std::string&);

    std::string frame_id();

    void set_color(float, float, float, float);

    void set_time_stamp(const rclcpp::Time&);

    void generate(
        const std::vector<data::ArmorCameraSpacing>& armors_data,
        visualization_msgs::msg::MarkerArray& in_out_marker_arr);

private:
    class Impl;
    std::unique_ptr<Impl> pimpl_;
};

class ArmorMarkerInGimbalControlSpacing {
public:
    ArmorMarkerInGimbalControlSpacing();

    ~ArmorMarkerInGimbalControlSpacing();

    void set_frame_id(const std::string&);

    std::string frame_id();

    void set_color(float, float, float, float);

    void set_time_stamp(const rclcpp::Time&);

    void generate(
        const std::vector<data::ArmorGimbalControlSpacing>&, visualization_msgs::msg::MarkerArray&);

private:
    class Impl;
    std::unique_ptr<Impl> pimpl_;
};
} // namespace world_exe::ros