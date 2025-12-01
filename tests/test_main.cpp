
#include "./image_and_data.hpp"

#include "core/system_factory.hpp"
#include "parameters/params_system_v1.hpp"
#include "parameters/profile.hpp"
#include <cassert>
#include <cstdio>
#include <hikcamera/capturer.hpp>
#include <iostream>
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
    config.framerate       = 100;
    config.timeout_ms      = 2'000;
    config.exposure_us     = 800;
    config.fixed_framerate = true;

    if (auto ret = camera.initialize(config); !ret) {
        std::cout << "Failed: " << ret.error() << std::endl;
    }

    world_exe::util::memory::MatTripleBuffer buffer{[] {
        return world_exe::data::TimeStamp{std::chrono::steady_clock::now().time_since_epoch()};
    }};
    auto mat = cv::Mat();
    std::thread thread_capture{[&] {
        world_exe::util::FpsCounter fps_{};
        while (true) {
            if (auto ret = camera.read_image(); ret.has_value()) {
                mat = ret.value();
                buffer.set(mat);
                // if (fps_.count())
                //     std::cout << "fps: " << fps_.fps() << std::endl;
            } else {
                std::cout << "Failed:" << ret.error() << std::endl;
            }
        }
    }};

    auto node = std::make_shared<alliance_auto_aim::ros::bulldup::DataNode>(
        world_exe::parameters::ParamsForSystemV1::raw_image_event,
        world_exe::parameters::ParamsForSystemV1::camera_capture_transforms,
        [&mat, &buffer]() -> std::optional<std::reference_wrapper<world_exe::data::MatStamped>> {
            return buffer.get();
        });

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
