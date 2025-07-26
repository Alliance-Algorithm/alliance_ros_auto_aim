
#include "./tests.hpp"
#include "enum/system_version.hpp"
#include "system_factory.hpp"

int main(int argc, const char *const *argv) {
  world_exe::core::SystemFactory::Build(
      world_exe::enumeration::SystemVersion::V1);
  world_exetest::ros::test::test_armor_3d_camera_in_3d_view(argc, argv);
}