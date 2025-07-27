
#include <chrono>
#include <cstring>
#include <memory>
#include <thread>

#include <opencv2/core/mat.hpp>
#include <opencv2/videoio.hpp>

#include <Eigen/Eigen>

#include <rclcpp/create_timer.hpp>
#include <rclcpp/executors.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include "armor_marker.hpp"
#include "data/sync_data.hpp"
#include "double_buffer.hpp"
#include "event_bus.hpp"
#include "fps_counter.hpp"
#include "interfaces/armor_in_camera.hpp"
#include "interfaces/predictor_update_package.hpp"
#include "parameters/params_system_v1.hpp"
#include "parameters/profile.hpp"
#include "sync/hik_camera_syncdata.hpp"
#include "system_factory.hpp"

class CarTrackerTest : public rclcpp::Node {
public:
    CarTrackerTest()
        : rclcpp::Node("test") {

        using namespace world_exe;
        using namespace parameters;
#pragma region 设置初始参数
        image_capturer_ = {};                        // set here;
        marker_pub_     = create_publisher<visualization_msgs::msg::MarkerArray>(
            "/alliance_auto_aim/pnp/camera_armor", 10);
        marker_pub2_ = create_publisher<visualization_msgs::msg::MarkerArray>(
            "/alliance_auto_aim/filter/camera_armor", 10);
        image_pub_ = create_publisher<sensor_msgs::msg::Image>("/alliance_auto_aim/img/raw", 10);
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        std::thread([this]() {
            auto begin                           = std::chrono::steady_clock::now();
            data.camera_capture_begin_time_stamp = begin.time_since_epoch().count();
            while (true) {

                data.camera_capture_begin_time_stamp =
                    (std::chrono::steady_clock::now() - begin).count();

                image_capturer_.read(mat_);
                double_buffer_->store(mat_, data);
                if (fps_counter1.count())
                    RCLCPP_INFO(get_logger(), "FPS: %lf", fps_counter1.fps());
            }
        }).detach();

        do {
            image_capturer_.read(mat_);
        } while (mat_.empty());
        const int& w = mat_.cols;
        const int& h = mat_.rows;
        world_exe::parameters::HikCameraProfile::set_width_height(w, h);
        world_exe::parameters::ParamsForSystemV1::set_szu_model_path(
            ament_index_cpp::get_package_share_directory("alliance_ros_auto_aim")
            + "/szu_identify_model.onnx");
#pragma endregion

#pragma region 可视化
        world_exe::core::EventBus::Subscript<
            std::shared_ptr<world_exe::interfaces::IArmorInCamera>>(
            world_exe::parameters::ParamsForSystemV1::armors_in_camera_pnp_event,
            [this](const auto& data) {
                visualization_msgs::msg::MarkerArray msg{};
                world_exe::ros::ArmorMarkerGenerator::generate_all(*data, "camera_link", msg);
                marker_pub_->publish(msg);
            });
        world_exe::core::EventBus::Subscript<
            std::shared_ptr<world_exe::interfaces::IArmorInGimbalControl>>(
            world_exe::parameters::ParamsForSystemV1::tracker_current_armors_event,
            [this](const std::shared_ptr<world_exe::interfaces::IArmorInGimbalControl>& data) {
                visualization_msgs::msg::MarkerArray msg{};
                world_exe::ros::ArmorMarkerGenerator::generate_all(*data, "odom_imu", msg);
                marker_pub2_->publish(msg);
            });

        core::EventBus::Subscript<std::shared_ptr<interfaces::IPreDictorUpdatePackage>>(
            ParamsForSystemV1::tracker_update_event, //
            [this](const std::shared_ptr<interfaces::IPreDictorUpdatePackage>& data) {
                // 这里同步过的，可以放心大胆的用
            });

#pragma endregion

        world_exe::core::SystemFactory::Build(world_exe::enumeration::SystemVersion::V1);
        double_buffer_ = std::make_unique<DoubleBuffer<
            world_exe::sync::HikCameraSyncData, cv::Mat,
            world_exe::data::CameraGimbalMuzzleSyncData>>(
            new world_exe::sync::HikCameraSyncData(mat_),
            new world_exe::sync::HikCameraSyncData(mat_), [](const auto& data) {
                const auto& [mat, d] = data;

                world_exe::core::EventBus::Publish<world_exe::data::CameraGimbalMuzzleSyncData>(
                    world_exe::parameters::ParamsForSystemV1::camera_capture_transforms, d);
                world_exe::core::EventBus::Publish<cv::Mat>(
                    world_exe::parameters::ParamsForSystemV1::raw_image_event, mat);
            });
    }

private:
    void tf_broadcast(const Eigen::Affine3d& affine) {
        geometry_msgs::msg::TransformStamped tf;
        tf.header.stamp    = rclcpp::Clock().now();
        tf.header.frame_id = "odom";
        tf.child_frame_id  = "camera_link";

        tf.transform.translation.x = affine.translation().x();
        tf.transform.translation.y = affine.translation().y();
        tf.transform.translation.z = affine.translation().z();
        Eigen::Quaterniond q{affine.rotation()};
        tf.transform.rotation.x = q.x();
        tf.transform.rotation.y = q.y();
        tf.transform.rotation.z = q.z();
        tf.transform.rotation.w = q.w();

        tf_broadcaster_->sendTransform(tf);
    }
    cv::Mat mat_;
    cv::Mat pnp_;
    cv::VideoCapture image_capturer_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub2_;

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr sync_sub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    std::unique_ptr<DoubleBuffer<
        world_exe::sync::HikCameraSyncData, cv::Mat, world_exe::data::CameraGimbalMuzzleSyncData>>
        double_buffer_;
    world_exe::util::FpsCounter fps_counter{};
    world_exe::util::FpsCounter fps_counter1{};

    world_exe::data::CameraGimbalMuzzleSyncData data{};
};

int car_tracker_test(int argc, char** argv) {

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CarTrackerTest>());
    rclcpp::shutdown();
}