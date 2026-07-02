cmake_minimum_required(VERSION 3.5)
project(asr_sdm_vio_marker_adapter)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 14)
add_compile_options(-Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(marker_adapter_node
    src/marker_adapter_node.cpp
)

ament_target_dependencies(marker_adapter_node
    rclcpp sensor_msgs geometry_msgs visualization_msgs nav_msgs std_msgs
)

install(TARGETS marker_adapter_node
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()
