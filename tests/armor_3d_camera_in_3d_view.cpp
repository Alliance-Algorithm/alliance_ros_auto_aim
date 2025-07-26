#include <chrono>
#include <cstdlib>
#include <memory>
#include <numbers>
#include <random>
#include <rclcpp/clock.hpp>
#include <rclcpp/logging.hpp>
#include <string>

#include "./tests.hpp"

#include "armor_marker.hpp"
#include "enum/armor_id.hpp"
#include "event_bus.hpp"
#include "interfaces/armor_in_camera.hpp"
#include "parameters/params_system_v1.hpp"

#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <visualization_msgs/msg/detail/marker_array__struct.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace world_exetest::ros::test {
static std::vector<world_exe::data::ArmorCameraSpacing> test_armor_generate();

static inline void armor_3d_camera_in_3d_view(
    const rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr& marker_array_pub,
    const world_exe::interfaces::IArmorInCamera& armor_to_pub,
    const world_exe::enumeration::ArmorIdFlag& armor_id,
    void (*process)(
        const world_exe::interfaces::IArmorInCamera&, const world_exe::enumeration::ArmorIdFlag&,
        const std::string&, visualization_msgs::msg::MarkerArray&)) {

    visualization_msgs::msg::MarkerArray msg;
    process(armor_to_pub, armor_id, "armor_in_camera_test", msg);
    marker_array_pub->publish(msg);
}

class TestNode : public rclcpp::Node {
public:
    TestNode()
        : rclcpp::Node("alliance_ros_auto_aim_test") {
        world_exe::core::EventBus::Subscript<
            std::shared_ptr<world_exe::interfaces::IArmorInCamera>>(
            world_exe::parameters::ParamsForSystemV1::armors_in_camera_pnp_event,
            [this](const auto& data) { armor_interface_ = data; });
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        pub_            = create_publisher<visualization_msgs::msg::MarkerArray>(
            "/alliance_auto_aim/test/armor_3d_camera_in_3d_view", 10);

        timer_ =
            this->create_wall_timer(std::chrono::milliseconds(100), [this]() { timer_callback(); });
    }

private:
    void timer_callback() {
        // world_exetest::ros::test::armor_3d_camera_in_3d_view({}, {});

        // world_exe::ros::ArmorMarkerGenerator::generate(armor_interface, armor_id,
        // frame_id, marker_array); a static func ,return
        visualization_msgs::msg::MarkerArray msg{};
        world_exe::ros::ArmorMarkerGenerator::generate(
            *armor_interface_, world_exe::enumeration::ArmorIdFlag::InfantryIV, "camera_link", msg);

        tf_broadcast();
        pub_->publish(msg);
    }

    void tf_broadcast() {
        geometry_msgs::msg::TransformStamped tf;
        tf.header.stamp    = rclcpp::Clock().now();
        tf.header.frame_id = "odom";
        tf.child_frame_id  = "camera_link";

        tf.transform.translation.x = 0.0;
        tf.transform.translation.y = 0.0;
        tf.transform.translation.z = 0.0;

        tf2::Quaternion q;
        q.setRPY(0, 0, 0);
        tf.transform.rotation.x = q.x();
        tf.transform.rotation.y = q.y();
        tf.transform.rotation.z = q.z();
        tf.transform.rotation.w = q.w();

        tf_broadcaster_->sendTransform(tf);
    }
    std::shared_ptr<world_exe::interfaces::IArmorInCamera> armor_interface_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int test_armor_3d_camera_in_3d_view(int argc, const char* const* argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TestNode>());
    rclcpp::shutdown();
    return 0;
}

static std::vector<world_exe::data::ArmorCameraSpacing> test_armor_generate() {
    std::vector<world_exe::data::ArmorCameraSpacing> armors_data;

    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::mt19937 engine(seed);

    std::uniform_int_distribution<int> dist(0, 90);
    auto random_angle = dist(engine) * std::numbers::pi / 180;

    for (int i = 0; i < 4; i++) {
        world_exe::data::ArmorCameraSpacing single_armor_;
        single_armor_.id = world_exe::enumeration::ArmorIdFlag::InfantryIII;
        auto angle       = i * std::numbers::pi / 2 + random_angle;

        single_armor_.position =
            Eigen::Vector3d(1.0 + 0.25 * sin(angle), 1.0 + 0.25 * cos(angle), 0.10);

        single_armor_.orientation.w() = sin(angle / 2);
        single_armor_.orientation.x() = 0;
        single_armor_.orientation.y() = 0;
        single_armor_.orientation.z() = cos(angle / 2);

        armors_data.push_back(single_armor_);
    }
    return armors_data;
}
} // namespace world_exetest::ros::test