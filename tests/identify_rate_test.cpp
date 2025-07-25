// #include "enum/car_id.hpp"
// #include "enum/system_version.hpp"
// #include "event_bus.hpp"
// #include "parameters/params_system_v1.hpp"
// #include "parameters/profile.hpp"
// #include "system_factory.hpp"
// #include "visualization.hpp"
// #include <ament_index_cpp/get_package_share_directory.hpp>
// #include <cstdio>
// #include <interfaces/armor_in_camera.hpp>
// #include <memory>
// #include <opencv2/core/mat.hpp>
// #include <opencv2/highgui.hpp>

// int identifier_rate_test(int argc, char **argv) {
//   (void)argc;
//   (void)argv;
//   world_exe::parameters::ParamsForSystemV1::set_szu_model_path(
//       ament_index_cpp::get_package_share_directory("alliance_ros_auto_aim") +
//       "/szu_identify_model.onnx");
//   const auto &[w, h] = cap.get_width_height();
//   world_exe::parameters::HikCameraProfile::set_width_height(w, h);

//   world_exe::core::SystemFactory::Build(
//       world_exe::enumeration::SystemVersion::V1);

//   cv::Mat mat = cap.read();
//   double i = 0;
//   double d = 0;

//   world_exe::core::EventBus::Subscript<
//       std::shared_ptr<world_exe::interfaces::IArmorInImage>>(
//       world_exe::parameters::ParamsForSystemV1::armors_in_image_identify_event,
//       [&mat, &i](const auto &data) {
//         cv::Mat visual{mat};
//         world_exe::utils::visualization::draw_armor_in_image(*data, visual);
//         cv::imshow("identify", visual);
//         i++;
//       });

//   world_exe::core::EventBus::Subscript<cv::Mat>(
//       world_exe::parameters::ParamsForSystemV1::raw_image_event,
//       [&d, &i](const auto &) {
//         d++;
//         printf("identify rate{i / d} : %lf\n", i / d);
//       });
//   while (true) {
//     mat = cap.read();
//     cv::imshow("raw", mat);
//     world_exe::core::EventBus::Publish(
//         world_exe::parameters::ParamsForSystemV1::raw_image_event, mat);
//     world_exe::core::EventBus::Publish(
//         world_exe::parameters::ParamsForSystemV1::raw_image_event, mat);

//     cv::waitKey(1);
//   }
//   return 0;
// }
