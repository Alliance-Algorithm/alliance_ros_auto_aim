cmake_minimum_required(VERSION 3.8)
project(alliance_ros_auto_aim)


set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_BUILD_TYPE "Release")

# 查找ROS2基础依赖
find_package (ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies (REQUIRED)


# 添加子目录（包含核心库）
add_subdirectory(alliance_auto_aim)

install(
  DIRECTORY alliance_auto_aim/include/
  DESTINATION include/${PROJECT_NAME}
)

include_directories(
    alliance_auto_aim/include/
)

install(
  TARGETS alliance_auto_aim
  DESTINATION lib/${PROJECT_NAME}
)

# 导出库和依赖
ament_export_include_directories(include/${PROJECT_NAME})
ament_export_libraries(alliance_auto_aim)
ament_package()
