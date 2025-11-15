
#include "./image_and_data.hpp"

#include "core/system_factory.hpp"
#include "parameters/params_system_v1.hpp"
#include "parameters/profile.hpp"
#include <cassert>
#include <filesystem>
#include <hikcamera/capturer.hpp>
#include <memory>
#include <opencv2/core/mat.hpp>
#include <opencv2/highgui.hpp>
#include <print>

#include <rclcpp/executors.hpp>
#include <rclcpp/utilities.hpp>

int main(int argc, const char* const* argv) {
    world_exe::parameters::HikCameraProfile::set_width_height(1440, 720);
    // world_exe::parameters::HikCameraProfile::set_width_height(1190, 595);

    // auto image_path =
    //     std::filesystem::path{__FILE__}.parent_path().parent_path() / "assets" / "armor.jpg";
    // std::cout << image_path << std::endl;
    // cv::Mat mat = cv::imread(image_path, cv::IMREAD_COLOR);
    // std::println("({},{})", mat.rows, mat.cols);

    // if (mat.empty()) {
    //     std::cerr << "错误: 无法读取图片或图片路径错误!" << std::endl;
    //     std::cerr << "尝试读取的路径是: " << image_path << std::endl;
    //     return -1; // 返回错误代码
    // }

    // cv::imshow("Loaded Image (C++17)", mat);
    // cv::waitKey(0);

    rclcpp::init(argc, argv);
    world_exe::core::SystemFactory::Build(world_exe::enumeration::SystemVersion::V2Debug);
    auto camera            = hikcamera::Camera{};
    auto config            = hikcamera::Config{};
    config.framerate       = 500;
    config.timeout_ms      = 2'000;
    config.exposure_us     = 800;
    config.fixed_framerate = true;

    if (auto ret = camera.initialize(config); !ret) {
        std::println("Failed: {}", ret.error());
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
                std::println("Failed: {}", ret.error());
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
