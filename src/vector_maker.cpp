
#include "vector_maker.hpp"

namespace world_exe::ros {

void VectorMarker::generate(
    Eigen::Vector3d const& vec, const rclcpp::Time& time, const std::string& frame_id,
    visualization_msgs::msg::Marker& marker) {

    marker.type   = visualization_msgs::msg::Marker::ARROW;
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.header.frame_id = frame_id;
    marker.header.stamp    = time;
    marker.id              = 0;

    marker.color.r = 1.0;
    marker.color.g = 0;
    marker.color.b = 0;
    marker.color.a = 1.0;

    marker.scale.x = 0.03;
    marker.scale.y = 0.05;
    marker.scale.z = 0.0;

    marker.ns = "direction";

    marker.pose.position.x = 0.;
    marker.pose.position.y = 0.;
    marker.pose.position.z = 0.;

    geometry_msgs::msg::Point start_point;
    start_point.x = 0.;
    start_point.y = 0.;
    start_point.z = 0.;

    geometry_msgs::msg::Point end_point;
    end_point.x = start_point.x + vec.x();
    end_point.y = start_point.y + vec.y();
    end_point.z = start_point.z + vec.z();

    marker.points.push_back(start_point);
    marker.points.push_back(end_point);
};

} // namespace world_exe::ros
