cmake_minimum_required(VERSION 3.8)
project(so3_control)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(asr_sdm_control_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# Core controller (pure Eigen).
add_library(SO3Control src/SO3Control.cpp)
target_include_directories(SO3Control PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(SO3Control Eigen3::Eigen)

# rclcpp component (replaces the ROS 1 nodelet).
add_library(so3_control_component SHARED src/so3_control_nodelet.cpp)
target_include_directories(so3_control_component PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(so3_control_component SO3Control Eigen3::Eigen)
ament_target_dependencies(so3_control_component
  rclcpp rclcpp_components nav_msgs sensor_msgs std_msgs asr_sdm_control_msgs)

rclcpp_components_register_node(so3_control_component
  PLUGIN "SO3ControlComponent"
  EXECUTABLE so3_control_node)

install(TARGETS SO3Control so3_control_component
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(DIRECTORY include/
  DESTINATION include)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL)

ament_export_include_directories(include)
ament_export_dependencies(rclcpp asr_sdm_control_msgs)

ament_package()
