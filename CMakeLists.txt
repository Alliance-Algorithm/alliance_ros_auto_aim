cmake_minimum_required(VERSION 3.8)
project(alliance_ros_auto_aim)


set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_BUILD_TYPE "Release")

set(CMAKE_EXE_LINKER_FLAGS "-Wl,--copy-dt-needed-entries")



# 查找ROS2基础依赖
find_package (ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies (REQUIRED)

file(GLOB_RECURSE sources_v1 CONFIGURE_DEPENDS  
alliance_auto_aim/include/*.hpp 
alliance_auto_aim/src/*.hpp 
alliance_auto_aim/src/v1/*.cpp 
alliance_auto_aim/src/v1/*.cc
alliance_auto_aim/src/core/*.cpp
alliance_auto_aim/src/util/*.cpp
src/*.cpp
)
file(GLOB_RECURSE sources_test CONFIGURE_DEPENDS  
tests/*.cpp
)

add_library(${PROJECT_NAME} SHARED
    ${sources_v1}
    )
add_executable(${PROJECT_NAME}_test
    ${sources_v1}
    ${sources_test}
    )
target_link_libraries(${PROJECT_NAME}_test rclcpp::rclcpp)
ament_target_dependencies(${PROJECT_NAME}_test 
rclcpp
std_msgs 
visualization_msgs
geometry_msgs
tf2 
tf2_ros
tf2_geometry_msgs
sensor_msgs
cv_bridge
    )

include_directories(./alliance_auto_aim/include ./alliance_auto_aim/src ./include)
### OpenCV ===
# set(OpenCV_DIR /path/to/opencv/build) # 如果需要这个可以方便你设置
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} PUBLIC  ${OpenCV_LIBS})
target_link_libraries(${PROJECT_NAME}_test   ${OpenCV_LIBS})
### OpenCV End ===

### Eigen ===
find_package(Eigen3 REQUIRED)
INCLUDE_DIRECTORIES(${EIGEN3_INCLUDE_DIR})
### Eigen End ===

### TBB ===
find_package(TBB REQUIRED)
include_directories(${TBB_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} PUBLIC ${TBB_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_test  ${TBB_LIBRARIES})
### TBB End ===

### OpenVINO ===
find_package(OpenVINO REQUIRED)
target_link_libraries(${PROJECT_NAME} PUBLIC openvino::runtime)
target_link_libraries(${PROJECT_NAME}_test  openvino::runtime)
### OpenVINO End ===

### ceres ===
find_package(Ceres REQUIRED)
include_directories(${CERES_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} PUBLIC ${CERES_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_test  ${CERES_LIBRARIES})
### ceres End ===

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
DESTINATION include
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
ament_export_targets(${PROJECT_NAME} )

ament_package()
