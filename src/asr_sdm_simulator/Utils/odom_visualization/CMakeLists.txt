cmake_minimum_required(VERSION 3.8)
project(odom_visualization)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(asr_sdm_control_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pose_utils REQUIRED)
find_package(Armadillo REQUIRED)

add_executable(odom_visualization src/odom_visualization.cpp)
target_include_directories(odom_visualization PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${ARMADILLO_INCLUDE_DIRS})
target_link_libraries(odom_visualization ${ARMADILLO_LIBRARIES})
ament_target_dependencies(odom_visualization
  rclcpp sensor_msgs nav_msgs geometry_msgs visualization_msgs std_msgs
  asr_sdm_control_msgs tf2 tf2_ros pose_utils)

install(TARGETS odom_visualization
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY meshes
  DESTINATION share/${PROJECT_NAME})

ament_package()
