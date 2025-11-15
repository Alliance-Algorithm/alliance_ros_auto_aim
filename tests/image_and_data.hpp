
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <print>
#include <stop_token>
#include <thread>
#include <vector>

#include "armor_marker.hpp"
#include "core/event_bus.hpp"
#include "data/armor_camera_spacing.hpp"
#include "data/fire_control.hpp"
#include "data/mat_stamped.hpp"
#include "data/sync_data.hpp"
#include "data/time_stamped.hpp"
#include "enum/armor_id.hpp"
#include "interfaces/armor_in_camera.hpp"
#include "interfaces/armor_in_gimbal_control.hpp"
#include "parameters/params_system_v1.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"
#include "sync_data_processor.hpp"
#include "tongji/../../tests/mocks/mock_camera_tranform.hpp"
#include "tongji/predictor/in_gimbal_control_armor.hpp"
#include "tongji/solver/solved_armor.hpp"
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
        , mock_yaw_link2gimbal_transform_data_(
              Eigen::Vector3d(5, 0., 0.), 0., 0., 0.0, M_PI / 6, M_PI / 6.)
        , mock_pitch_link2yaw_link_transform_data_(
              Eigen::Vector3d(0, 0, 0), 0., 0., 0., M_PI / 6, M_PI / 12.)

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

        mock_armor_in_camera_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>(
            "/alliance_auto_aim/mock_armor_in_camera", 10);

        mock_armor_in_gimbal_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>(
            "/alliance_auto_aim/mock_armor_in_gimbal", 10);

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

        // mock_visualization_thread = std::thread([this]() { this->mock_visualization_loop(); });

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
                    std::println("no predicted armors");
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
                visualization_msgs::msg::Marker marker;
                world_exe::ros::VectorMarker::generate(
                    command.gimbal_dir.normalized(),
                    rclcpp::Time(static_cast<int64_t>(command.time_stamp.to_nanosec())),
                    "gimbal_link", marker);

                publisher_fire_dir_->publish(marker);
            });

        // world_exe::core::EventBus::Publish<world_exe::data::CameraGimbalMuzzleSyncData>(
        //     world_exe::parameters::ParamsForSystemV1::camera_capture_transforms, [this]() {
        //         mock_transform_data_.camera_capture_begin_time_stamp =
        //             world_exe::data::TimeStamp(std::chrono::steady_clock::now().time_since_epoch());
        //         mock_transform_data_.camera_to_gimbal = Eigen::Affine3d::Identity();
        //         // mock_yaw_link2gimbal_transform_data_.updateAndGetTransform(0.000001);
        //         // .inverse();
        //         mock_transform_data_.gimbal_to_muzzle =
        //             // mock_pitch_link2yaw_link_transform_data_.updateAndGetTransform(0.01)
        //             //     .inverse();
        //             Eigen::Affine3d::Identity();
        //         return mock_transform_data_;
        //     }());

        mock_data_generate_jthread = std::jthread([&](std::stop_token const& token) {
            while (!token.stop_requested()) {
                world_exe::core::EventBus::Publish<world_exe::data::CameraGimbalMuzzleSyncData>(
                    world_exe::parameters::ParamsForSystemV1::camera_capture_transforms, [this]() {
                        mock_transform_data_.camera_capture_begin_time_stamp =
                            world_exe::data::TimeStamp(
                                std::chrono::steady_clock::now().time_since_epoch());
                        mock_transform_data_.camera_to_gimbal = Eigen::Affine3d::Identity();
                        // mock_yaw_link2gimbal_transform_data_.updateAndGetTransform(0.000001);
                        // .inverse();
                        mock_transform_data_.gimbal_to_muzzle =
                            // mock_pitch_link2yaw_link_transform_data_.updateAndGetTransform(0.01);
                            //     .inverse();
                            Eigen::Affine3d::Identity();
                        return mock_transform_data_;
                    }());
                std::this_thread::sleep_for(std::chrono::microseconds(1000));
            }
        });
    }

    ~DataNode() = default;

private:
    auto GenerateMockData() {
        using namespace world_exe::ros;
        using namespace world_exe::enumeration;
        using namespace std::chrono_literals;
        auto transform      = Eigen::Affine3d::Identity();
        constexpr double dt = 0.01;

        while (rclcpp::ok()) {
            transform = this->mock_yaw_link2gimbal_transform_data_.updateAndGetTransform(dt);
        }
        return transform;
    }

    void mock_visualization_loop() {
        using namespace world_exe::ros;
        using namespace world_exe::enumeration;
        using namespace std::chrono_literals;

        constexpr double dt = 0.01;

        std::vector<world_exe::data::ArmorCameraSpacing> armors_in_camera;
        armors_in_camera.emplace_back(
            ArmorIdFlag::InfantryIII, Eigen::Vector3d{0.1, 0.1, 0.},
            Quaterniond(AngleAxisd(M_PI / 2.0, Vector3d::UnitZ())));

        auto mock_armors_in_camera = world_exe::tongji::solver::SolvedArmor(
            armors_in_camera,
            world_exe::data::TimeStamp(std::chrono::steady_clock::now().time_since_epoch()));

        while (rclcpp::ok()) {
            Eigen::Affine3d T_C_to_G =
                this->mock_yaw_link2gimbal_transform_data_.updateAndGetTransform(dt);
            // std::cout << T_C_to_G.matrix() << std::endl;

            auto camera2gimbal = [&T_C_to_G](const auto& armors_in_camera) {
                world_exe::data::ArmorGimbalControlSpacing gimbal_armor;
                gimbal_armor.id       = armors_in_camera.id;
                gimbal_armor.position = T_C_to_G * armors_in_camera.position;
                gimbal_armor.orientation =
                    Quaterniond(T_C_to_G.rotation()) * armors_in_camera.orientation;
                return gimbal_armor;
            };

            std::vector<world_exe::data::ArmorGimbalControlSpacing> armors_in_gimbal;
            for (const auto& camera_armor : armors_in_camera) {
                world_exe::data::ArmorGimbalControlSpacing gimbal_armor =
                    camera2gimbal(camera_armor);
                armors_in_gimbal.emplace_back(gimbal_armor);
            }

            auto transformed_armor = world_exe::tongji::predictor::InGimbalControlArmor(
                armors_in_gimbal,
                world_exe::data::TimeStamp(std::chrono::steady_clock::now().time_since_epoch()));

            visualization_msgs::msg::MarkerArray marker_array_in_camera;
            world_exe::ros::ArmorMarkerGenerator::generate_all(
                mock_armors_in_camera, "camera_link", marker_array_in_camera);
            mock_armor_in_camera_publisher_->publish(marker_array_in_camera);

            visualization_msgs::msg::MarkerArray marker_array_in_gimbal;
            world_exe::ros::ArmorMarkerGenerator::generate_all(
                transformed_armor, "gimbal_link", marker_array_in_gimbal);
            mock_armor_in_gimbal_publisher_->publish(marker_array_in_gimbal);

            std::this_thread::sleep_for(std::chrono::duration<double>(dt));
        }
    }

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_pnp_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_gimbal_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher_fire_dir_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_predictor_;

    world_exe::tests::mock::Camera2GimbalTransformer mock_yaw_link2gimbal_transform_data_;
    world_exe::tests::mock::Camera2GimbalTransformer mock_pitch_link2yaw_link_transform_data_;

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
        mock_armor_in_camera_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
        mock_armor_in_gimbal_publisher_;

    std::thread mock_visualization_thread;
    std::jthread mock_data_generate_jthread;

    world_exe::data::CameraGimbalMuzzleSyncData mock_transform_data_;

    rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr sync_data_sub_;
    world_exe::util::memory::MatTripleBuffer<world_exe::util::time_stamp::SteadyClock> buffer_;
    std::thread capture_thread;
    std::thread publish_thread;
};
} // namespace alliance_auto_aim::ros::bulldup