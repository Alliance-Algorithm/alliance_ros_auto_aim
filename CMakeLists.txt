cmake_minimum_required(VERSION 3.8)
project(alliance_ros_auto_aim)


set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_BUILD_TYPE "Release")

set(CMAKE_EXE_LINKER_FLAGS "-Wl,--copy-dt-needed-entries")


add_subdirectory(alliance_auto_aim)

# 查找ROS2基础依赖
find_package (ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies (REQUIRED)

file(GLOB_RECURSE sources_v1 CONFIGURE_DEPENDS  
src/*.cpp
)
file(GLOB_RECURSE sources_test CONFIGURE_DEPENDS  
tests/*.cpp
)

add_library(${PROJECT_NAME} SHARED
    ${sources_v1}
    )
add_executable(${PROJECT_NAME}_test
    ${sources_test}
    )

target_link_libraries(${PROJECT_NAME}       PRIVATE alliance_auto_aim)
target_link_libraries(${PROJECT_NAME}_test  PUBLIC  ${PROJECT_NAME} alliance_auto_aim rclcpp::rclcpp )

include_directories(./alliance_auto_aim/include ./alliance_auto_aim/src ./include)

ament_target_dependencies(${PROJECT_NAME}_test PUBLIC
  rclcpp
  std_msgs 
  visualization_msgs
  geometry_msgs
  tf2 
  tf2_ros
  tf2_geometry_msgs
  sensor_msgs
  cv_bridge
  ament_index_cpp
  hikcamera
)


ament_target_dependencies(${PROJECT_NAME} PUBLIC 
  rclcpp
  visualization_msgs
  geometry_msgs
  tf2 
  tf2_ros
  tf2_geometry_msgs
) 

install(DIRECTORY alliance_auto_aim/models/
DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY 
alliance_auto_aim/include/ 
include/
DESTINATION include/${PROJECT_NAME}
)

install(TARGETS ${PROJECT_NAME}
EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS ${PROJECT_NAME}_test
EXPORT ${PROJECT_NAME}_test
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# 导出库和依赖
ament_export_targets(${PROJECT_NAME})

ament_auto_package()
