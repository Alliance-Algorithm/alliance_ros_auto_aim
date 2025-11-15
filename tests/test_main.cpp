
#include "./image_and_data.hpp"

#include "core/system_factory.hpp"
#include "parameters/params_system_v1.hpp"
#include "parameters/profile.hpp"
#include <cassert>
#include <hikcamera/capturer.hpp>
#include <memory>
#include <opencv2/core/mat.hpp>
#include <opencv2/highgui.hpp>

#include <rclcpp/executors.hpp>
#include <rclcpp/utilities.hpp>

int main(int argc, const char* const* argv) {
    world_exe::parameters::HikCameraProfile::set_width_height(1440, 720);

    rclcpp::init(argc, argv);
    world_exe::core::SystemFactory::Build(world_exe::enumeration::SystemVersion::V2Debug);
    auto camera            = hikcamera::Camera{};
    auto config            = hikcamera::Config{};
    config.framerate       = 500;
    config.timeout_ms      = 2'000;
    config.exposure_us     = 800;
    config.fixed_framerate = true;

    if (auto ret = camera.initialize(config); !ret) {
        // std::println("Failed: {}", ret.error());
    }

    // world_exe::util::memory::MatTripleBuffer buffer{[] {
    //   return world_exe::data::TimeStamp{
    //       std::chrono::steady_clock::now().time_since_epoch()};
    // }};
    auto mat = cv::Mat();
    std::thread thread_capture{[&] {
        while (true) {
            if (auto ret = camera.read_image()) {
                mat = ret.value();
                // buffer.set(mat);

            } else {
                // std::println("Failed: {}", ret.error());
            }
        }
    }};

    auto node = std::make_shared<alliance_auto_aim::ros::bulldup::DataNode>(
        world_exe::parameters::ParamsForSystemV1::raw_image_event,
        world_exe::parameters::ParamsForSystemV1::camera_capture_transforms, [&mat]() -> cv::Mat {
            return mat;
            // auto optional = buffer.get();
            // if (optional.has_value()) {
            //   auto &mat = optional.value().get().mat;
            //   if (!mat.empty())
            //     return mat;
            // }
            // return cv::Mat();
        });

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
