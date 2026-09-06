cmake_minimum_required(VERSION 3.8)
project(asr_sdm_control_manager)

# Default to RelWithDebInfo so plain `colcon build --packages-select asr_sdm_control_manager`
# still gets -O2 -g instead of an unoptimized, stripped binary.
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(asr_sdm_control_msgs REQUIRED)
find_package(asr_sdm_kinematic_dynamic_model REQUIRED)
find_package(asr_sdm_head_following_control REQUIRED)
find_package(asr_sdm_log_collector REQUIRED)
find_package(grampc REQUIRED)

add_executable(asr_sdm_control_manager
  src/asr_sdm_control_manager_node.cpp
)
target_link_libraries(asr_sdm_control_manager PUBLIC
  asr_sdm_kinematic_dynamic_model::asr_sdm_kinematic_model
  asr_sdm_head_following_control::front_unit_following_controller_3d_core
  asr_sdm_head_following_control::head_command_mpc
  asr_sdm_log_collector::asr_sdm_log_client
  ${asr_sdm_control_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${std_msgs_TARGETS}
  rclcpp::rclcpp
)

install(TARGETS asr_sdm_control_manager
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
