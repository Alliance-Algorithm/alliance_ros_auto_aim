#pragma once

#include "visualization_msgs/msg/marker.hpp"
#include <Eigen/Dense>
#include <rclcpp/time.hpp>

namespace world_exe::ros {

class VectorMarker {
public:
    static void generate(
        Eigen::Vector3d const& vec, const rclcpp::Time& time, const std::string& frame_id,
        visualization_msgs::msg::Marker& marker);
};
} // namespace world_exe::ros