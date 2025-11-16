
#include <cmath>
#include <functional>
#include <memory>
#include <optional>
#include <thread>

#include "armor_marker.hpp"
#include "core/event_bus.hpp"
#include "data/fire_control.hpp"
#include "data/mat_stamped.hpp"
#include "data/sync_data.hpp"
#include "data/time_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "interfaces/armor_in_camera.hpp"
#include "interfaces/armor_in_gimbal_control.hpp"
#include "parameters/params_system_v1.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"
#include "sync_data_processor.hpp"
#include "tongji/../../tests/mocks/mock_camera_tranform.hpp"
#include "utils/fps_counter.hpp"
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
using namespace std::chrono_literals;

class DataNode final : public rclcpp::Node {
public:
    using Data = world_exe::ros::SyncData_Feb_TimeCameraGimbal_8byteAlignas;

    DataNode(
        const std::string& image_event, const std::string& sync_data_event,
        std::function<std::optional<std::reference_wrapper<world_exe::data::MatStamped>>()> func)
        : rclcpp::Node("image_and_data", "/alliacne_auto_aim")
        // , mock_yaw_link2gimbal_transform_data_(
        //       Eigen::Vector3d(5, 0., 0.), 0., 0., 0.0, M_PI / 6, M_PI / 6.)
        // , mock_pitch_link2yaw_link_transform_data_(
        //       Eigen::Vector3d(0, 0, 0), 0., 0., 0., M_PI / 6, M_PI / 12.)

        , buffer_(world_exe::util::time_stamp::SteadyClock{}) {
        sync_data_sub_ = create_subscription<std_msgs::msg::UInt8MultiArray>(
            "/alliacne_auto_aim/camera/sync_data", 10,
            [&](std_msgs::msg::UInt8MultiArray::UniquePtr data) {
                auto raw    = *reinterpret_cast<const Data*>(data->data.data());
                auto decode = world_exe::ros::sync_data_process(raw);
                world_exe::core::EventBus::Publish<world_exe::data::CameraGimbalMuzzleSyncData&>(
                    sync_data_event, decode);
            });

        camera_to_gimbal_subscription_ = create_subscription<geometry_msgs::msg::TransformStamped>(
            "/gimbal/camera_to_gimbal_transform", rclcpp::QoS(10),
            [&](geometry_msgs::msg::TransformStamped const& data) {
                const auto& t = data.transform;
                Eigen::Translation3d translation(t.translation.x, t.translation.y, t.translation.z);
                Eigen::Quaterniond rotation(t.rotation.w, t.rotation.x, t.rotation.y, t.rotation.z);
                transform_.camera_to_gimbal = translation * rotation;

                auto timestamp =
                    world_exe::data::TimeStamp::from_nanosec(data.header.stamp.nanosec);
                transform_.camera_capture_begin_time_stamp = timestamp;

                world_exe::core::EventBus::Publish<world_exe::data::CameraGimbalMuzzleSyncData>(
                    world_exe::parameters::ParamsForSystemV1::camera_capture_transforms,
                    transform_);
            });

        gimbal_to_muzzle_subscription_ = create_subscription<geometry_msgs::msg::TransformStamped>(
            "/gimbal/gimbal_to_muzzle_transform", rclcpp::QoS(10),
            [&](geometry_msgs::msg::TransformStamped const& data) {
                const auto& t = data.transform;
                Eigen::Translation3d translation(t.translation.x, t.translation.y, t.translation.z);
                Eigen::Quaterniond rotation(t.rotation.w, t.rotation.x, t.rotation.y, t.rotation.z);
                transform_.gimbal_to_muzzle = translation * rotation;

                auto timestamp =
                    world_exe::data::TimeStamp::from_nanosec(data.header.stamp.nanosec);
                transform_.camera_capture_begin_time_stamp = timestamp;

                world_exe::core::EventBus::Publish<world_exe::data::CameraGimbalMuzzleSyncData>(
                    world_exe::parameters::ParamsForSystemV1::camera_capture_transforms,
                    transform_);
            });

        publisher_pnp_ = create_publisher<visualization_msgs::msg::MarkerArray>(
            "/alliacne_auto_aim/armor_pnp", 10);

        publisher_gimbal_ = create_publisher<visualization_msgs::msg::MarkerArray>(
            "/alliacne_auto_aim/armor_in_gimbal", 10);

        publisher_fire_dir_ = create_publisher<visualization_msgs::msg::Marker>(
            "/alliacne_auto_aim/fire_control_dir", 10);

        publish_thread = std::thread([image_event, &func, this]() {
            world_exe::util::FpsCounter fps_{};
            while (true) {
                auto mat = func();
                if (!mat.has_value() || mat.value().get().mat.empty())
                    continue;
                if (fps_.count()) {
                    std::cout << "b: " << fps_.fps() << std::endl;
                }
                world_exe::core::EventBus::Publish<world_exe::data::MatStamped>(
                    image_event, mat.value().get());
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
                    // std::println("no predicted armors");
                    return;
                }
                world_exe::ros::ArmorMarkerGenerator::generate_all(*data, "gimbal_link", msg);

                publisher_gimbal_->publish(msg);
            });

        // world_exe::core::EventBus::Subscript<
        //     std ::shared_ptr<world_exe::interfaces ::IArmorInGimbalControl>>(
        //     world_exe::parameters::ParamsForSystemV1::get_lastest_predictor_event,
        //     [&](std ::shared_ptr<world_exe::interfaces ::IArmorInGimbalControl> const& data)
        //         -> void {
        //         visualization_msgs::msg::MarkerArray msg{};
        //         if (data == nullptr) {
        //             return;
        //         }
        //         world_exe::ros::ArmorMarkerGenerator::generate_all(*data, "gimbal_link", msg);

        //         publisher_predictor_->publish(msg);
        //     });

        world_exe::core::EventBus::Subscript<world_exe::data::FireControl>(
            world_exe::parameters::ParamsForSystemV1::fire_control_event,
            [&](const world_exe::data::FireControl& command) -> void {
                if (!command.fire_allowance) {
                    return;
                }
                visualization_msgs::msg::Marker marker;
                world_exe::ros::VectorMarker::generate(
                    command.gimbal_dir.normalized(),
                    rclcpp::Time(static_cast<int64_t>(command.time_stamp.to_nanosec())),
                    "gimbal_link", marker);

                publisher_fire_dir_->publish(marker);
            });

        // mock_data_generate_jthread = std::jthread([&](std::stop_token const& token) {
        //     while (!token.stop_requested()) {
        //         world_exe::core::EventBus::Publish<world_exe::data::CameraGimbalMuzzleSyncData>(
        //             world_exe::parameters::ParamsForSystemV1::camera_capture_transforms,
        //             transform_);
        //         std::this_thread::sleep_for(std::chrono::microseconds(10));
        //     }
        // });
    }

    ~DataNode() = default;

private:
    // auto GenerateMockData() {
    //     using namespace world_exe::ros;
    //     using namespace world_exe::enumeration;
    //     using namespace std::chrono_literals;
    //     auto transform      = Eigen::Affine3d::Identity();
    //     constexpr double dt = 0.01;

    //     while (rclcpp::ok()) {
    //         transform = this->mock_yaw_link2gimbal_transform_data_.updateAndGetTransform(dt);
    //     }
    //     return transform;
    // }

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_pnp_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_gimbal_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher_fire_dir_;

    rclcpp::Subscription<geometry_msgs::msg::TransformStamped>::SharedPtr
        camera_to_gimbal_subscription_;
    rclcpp::Subscription<geometry_msgs::msg::TransformStamped>::SharedPtr
        gimbal_to_muzzle_subscription_;

    world_exe::data::CameraGimbalMuzzleSyncData transform_;

    // rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_predictor_;

    // world_exe::tests::mock::Camera2GimbalTransformer mock_yaw_link2gimbal_transform_data_;
    // world_exe::tests::mock::Camera2GimbalTransformer mock_pitch_link2yaw_link_transform_data_;

    // rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
    //     mock_armor_in_camera_publisher_;
    // rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
    //     mock_armor_in_gimbal_publisher_;

    std::jthread mock_data_generate_jthread;

    // world_exe::data::CameraGimbalMuzzleSyncData mock_transform_data_;

    rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr sync_data_sub_;
    world_exe::util::memory::MatTripleBuffer<world_exe::util::time_stamp::SteadyClock> buffer_;
    std::thread capture_thread;
    std::thread publish_thread;
};
} // namespace alliance_auto_aim::ros::bulldup