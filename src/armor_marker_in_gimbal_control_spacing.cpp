#include "armor_marker.hpp"
#include "data/armor_gimbal_control_spacing.hpp"
#include <geometry_msgs/msg/detail/vector3__struct.hpp>
#include <rclcpp/clock.hpp>
#include <rclcpp/time.hpp>
#include <string>

namespace world_exe::ros {

class ArmorMarkerInGimbalControlSpacing::Impl {
public:
    Impl() {
        color_.r = 0.0;
        color_.g = 1.0;
        color_.b = 0.0;
        color_.a = 1.0;

        small_armor_.x = 0.135;
        small_armor_.y = 0.020;
        small_armor_.z = 0.125;

        large_armor_.x = 0.230;
        large_armor_.y = 0.020;
        large_armor_.z = 0.127;
    }
    ~Impl() = default;

    void set_frame_id(const std::string& frame_id) { frame_id_ = frame_id; }
    std::string frame_id() { return frame_id_; }

    void set_color(float r, float g, float b, float a) {
        color_.r = r;
        color_.g = g;
        color_.b = b;
        color_.a = a;
    }

    void set_time_stamp(const rclcpp::Time& time) { time_stamp_ = time; }

    void generate(
        const std::vector<data::ArmorGimbalControlSpacing>& armors_data,
        visualization_msgs::msg::MarkerArray& in_out_marker_arr) {
        if (frame_id_.empty()) {
            throw std::runtime_error("ArmorMarker frame_id not setting!");
        }

        int marker_count = 0;
        for (const auto& armor : armors_data) {
            using namespace enumeration;
            visualization_msgs::msg::Marker marker;
            if (armor.id == ArmorIdFlag::Unknow && armor.id == ArmorIdFlag::None) {
                continue;
            } else if (armor.id == ArmorIdFlag::Hero && armor.id == ArmorIdFlag::Base) {
                marker.set__scale(large_armor_);
            } else {
                marker.set__scale(small_armor_);
            }
            marker.set__color(color_);

            if (time_stamp_.nanoseconds() == 0) {
                marker.header.stamp = rclcpp::Clock().now();
            } else {
                marker.header.stamp = time_stamp_;
            }
            marker.header.frame_id = frame_id_;

            marker.ns     = "armor";
            marker.id     = marker_count++;
            marker.type   = visualization_msgs::msg::Marker::CUBE;
            marker.action = visualization_msgs::msg::Marker::ADD;

            marker.pose.position.x = armor.position.x();
            marker.pose.position.y = armor.position.y();
            marker.pose.position.z = armor.position.z();

            marker.pose.orientation.x = armor.orientation.x();
            marker.pose.orientation.y = armor.orientation.y();
            marker.pose.orientation.z = armor.orientation.z();
            marker.pose.orientation.w = armor.orientation.w();
            marker.lifetime           = rclcpp::Duration::from_seconds(0.1);

            in_out_marker_arr.markers.push_back(marker);
        }
    }

private:
    std::string frame_id_;
    std_msgs::msg::ColorRGBA color_;
    rclcpp::Time time_stamp_;
    geometry_msgs::msg::Vector3 small_armor_;
    geometry_msgs::msg::Vector3 large_armor_;
};

ArmorMarkerInGimbalControlSpacing::ArmorMarkerInGimbalControlSpacing()
    : pimpl_(std::make_unique<Impl>()) {}

ArmorMarkerInGimbalControlSpacing::~ArmorMarkerInGimbalControlSpacing() = default;

void ArmorMarkerInGimbalControlSpacing::set_frame_id(const std::string& frame_id) {
    pimpl_->set_frame_id(frame_id);
}

std::string ArmorMarkerInGimbalControlSpacing::frame_id() { return pimpl_->frame_id(); }

void ArmorMarkerInGimbalControlSpacing::set_color(float r, float g, float b, float a) {
    pimpl_->set_color(r, g, b, a);
}

void ArmorMarkerInGimbalControlSpacing::set_time_stamp(const rclcpp::Time& time) {
    pimpl_->set_time_stamp(time);
}

void ArmorMarkerInGimbalControlSpacing::generate(
    const std::vector<data::ArmorGimbalControlSpacing>& armors_data,
    visualization_msgs::msg::MarkerArray& in_out_marker_arr) {
    return pimpl_->generate(armors_data, in_out_marker_arr);
}

} // namespace world_exe::ros