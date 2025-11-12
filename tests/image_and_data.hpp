#include <functional>
#include <memory>
#include <print>
#include <thread>

#include "armor_marker.hpp"
#include "core/event_bus.hpp"
#include "data/fire_control.hpp"
#include "data/mat_stamped.hpp"
#include "interfaces/armor_in_camera.hpp"
#include "interfaces/armor_in_gimbal_control.hpp"
#include "parameters/params_system_v1.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"
#include "sync_data_processor.hpp"
#include "utils/mat_triple_buffer.hpp"
#include "utils/time_stamp.hpp"

#include "vector_maker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>

namespace alliance_auto_aim::ros::bulldup {

class DataNode : public rclcpp::Node {
public:
    using Data = world_exe::ros::SyncData_Feb_TimeCameraGimbal_8byteAlignas;
    DataNode(
        const std::string& image_event, const std::string& sync_data_event,
        std::function<cv::Mat()> func)
        : rclcpp::Node("image_and_data", "/alliacne_auto_aim")
        , buffer_(world_exe::util::time_stamp::SteadyClock{}) {
        sync_data_sub_ = create_subscription<std_msgs::msg::UInt8MultiArray>(
            "/alliacne_auto_aim/camera/sync_data", 10,
            [&](std_msgs::msg::UInt8MultiArray::UniquePtr data) {
                auto raw    = *reinterpret_cast<const Data*>(data->data.data());
                auto decode = world_exe::ros::sync_data_process(raw);
                world_exe::core::EventBus::Publish<world_exe::data::CameraGimbalMuzzleSyncData&>(
                    sync_data_event, decode);
            });

        publisher_pnp_ = create_publisher<visualization_msgs::msg::MarkerArray>(
            "/alliacne_auto_aim/armor_pnp", 10);

        publisher_gimbal_ = create_publisher<visualization_msgs::msg::MarkerArray>(
            "/alliacne_auto_aim/armor_in_gimbal", 10);

        publisher_fire_dir_ = create_publisher<visualization_msgs::msg::Marker>(
            "/alliacne_auto_aim/fire_control_dir", 10);

        publisher_predictor_ = create_publisher<visualization_msgs::msg::MarkerArray>(
            "/alliacne_auto_aim/predicted_snapshot", 10);

        capture_thread = std::thread([&func, this]() {
            while (true) {
                buffer_.set(func());
            }
        });

        publish_thread = std::thread([image_event, this]() {
            while (true) {
                auto mat = buffer_.get();
                if (!mat.has_value() || mat->get().mat.empty())
                    continue;

                world_exe::core::EventBus::Publish<world_exe::data::MatStamped>(
                    image_event, mat->get());
            }
        });

        world_exe::core::EventBus::Subscript<
            std::shared_ptr<world_exe::interfaces::IArmorInCamera>>(
            world_exe::parameters::ParamsForSystemV1::armors_in_camera_pnp_event,
            [&](const std::shared_ptr<world_exe::interfaces::IArmorInCamera>& data) -> void {
                visualization_msgs::msg::MarkerArray msg{};
                if (data == nullptr) {
                    return;
                }
                world_exe::ros::ArmorMarkerGenerator::generate_all(*data, "camera_link", msg);
                publisher_pnp_->publish(msg);
            });

        world_exe::core::EventBus::Subscript<
            std ::shared_ptr<world_exe::interfaces ::IArmorInGimbalControl>>(
            world_exe::parameters::ParamsForSystemV1::tracker_current_armors_event,
            [&](std ::shared_ptr<world_exe::interfaces ::IArmorInGimbalControl> const& data)
                -> void {
                visualization_msgs::msg::MarkerArray msg{};
                if (data == nullptr) {
                    return;
                }
                world_exe::ros::ArmorMarkerGenerator::generate_all(*data, "gimbal_link", msg);

                publisher_gimbal_->publish(msg);
            });

        world_exe::core::EventBus::Subscript<
            std ::shared_ptr<world_exe::interfaces ::IArmorInGimbalControl>>(
            world_exe::parameters::ParamsForSystemV1::get_lastest_predictor_event,
            [&](std ::shared_ptr<world_exe::interfaces ::IArmorInGimbalControl> const& data)
                -> void {
                visualization_msgs::msg::MarkerArray msg{};
                if (data == nullptr) {
                    return;
                }
                world_exe::ros::ArmorMarkerGenerator::generate_all(*data, "gimbal_link", msg);

                publisher_predictor_->publish(msg);
            });

        world_exe::core::EventBus::Subscript<world_exe::data::FireControl>(
            world_exe::parameters::ParamsForSystemV1::fire_control_event,
            [&](const world_exe::data::FireControl& command) -> void {
                if (!command.fire_allowance) {
                    return;
                }
                // std::println(
                //     " dir: ({:.3f}, {:.3f}, {:.3f})", command.gimbal_dir.x(),
                //     command.gimbal_dir.y(), command.gimbal_dir.z());
                visualization_msgs::msg::Marker marker;
                world_exe::ros::VectorMarker::generate(
                    command.gimbal_dir.normalized(),
                    rclcpp::Time(static_cast<int64_t>(command.time_stamp.to_nanosec())),
                    "gimbal_link", marker);

                publisher_fire_dir_->publish(marker);
                // geometry_msgs::msg::Vector3Stamped dir;
                // dir.header.stamp.nanosec    = command.time_stamp.to_nanosec();
                // dir.header.frame_id = "gimbal_link";

                // dir.vector.set__x(command.gimbal_dir.x());
                // dir.vector.set__y(command.gimbal_dir.y());
                // dir.vector.set__z(command.gimbal_dir.z());
            });
    }

    ~DataNode() = default;

private:
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_pnp_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_gimbal_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher_fire_dir_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_predictor_;

    rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr sync_data_sub_;
    world_exe::util::memory::MatTripleBuffer<world_exe::util::time_stamp::SteadyClock> buffer_;
    std::thread capture_thread;
    std::thread publish_thread;
};
} // namespace alliance_auto_aim::ros::bulldup