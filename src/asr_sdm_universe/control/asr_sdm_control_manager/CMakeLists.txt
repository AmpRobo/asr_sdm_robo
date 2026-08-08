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
find_package(ament_cmake_auto REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(asr_sdm_control_msgs REQUIRED)
find_package(asr_sdm_hardware_msgs REQUIRED)
# find_package(periphery REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter Development NumPy)
find_package(Matplot++ QUIET)
if(NOT TARGET Matplot++::matplot)
  set(MATPLOTPP_SOURCE_DIR "" CACHE PATH "Path to the Matplot++ source tree")
  if(NOT MATPLOTPP_SOURCE_DIR)
    message(FATAL_ERROR "Matplot++ was not found. Install Matplot++ so find_package(Matplot++) works, or configure with -DMATPLOTPP_SOURCE_DIR=/path/to/matplotplusplus.")
  endif()
  if(NOT EXISTS "${MATPLOTPP_SOURCE_DIR}/CMakeLists.txt")
    message(FATAL_ERROR "MATPLOTPP_SOURCE_DIR must point to a Matplot++ source tree containing CMakeLists.txt: ${MATPLOTPP_SOURCE_DIR}")
  endif()
  set(MATPLOTPP_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
  set(MATPLOTPP_BUILD_TESTS OFF CACHE BOOL "" FORCE)
  set(MATPLOTPP_BUILD_INSTALLER OFF CACHE BOOL "" FORCE)
  add_subdirectory("${MATPLOTPP_SOURCE_DIR}" ${CMAKE_CURRENT_BINARY_DIR}/matplotplusplus EXCLUDE_FROM_ALL)
endif()
# Find libgpiod
# find_package(PkgConfig REQUIRED)
# pkg_check_modules(GPIOD REQUIRED libgpiod)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

include_directories(
  include
  # ${GPIOD_INCLUDE_DIRS}
)

# add_executable(mcp_can src/mcp_can.cpp)
# target_link_libraries(mcp_can ${MRAA_LIB})
# ament_target_dependencies(mcp_can rclcpp)

add_executable(asr_sdm_control_manager src/asr_sdm_control_manager_node.cpp)
# target_link_libraries(asr_sdm_control_manager periphery)
target_link_libraries(asr_sdm_control_manager PUBLIC
  ${asr_sdm_control_msgs_TARGETS}
  ${asr_sdm_hardware_msgs_TARGETS}
  ${std_msgs_TARGETS}
  rclcpp::rclcpp
  # periphery
)
# ament_target_dependencies(asr_sdm_control_manager rclcpp std_msgs asr_sdm_control_msgs asr_sdm_hardware_msgs)

add_executable(front_unit_following_controller_test_2d
  test/front_unit_following_controller_test_2d.cpp
  src/front_unit_following_controller_2d.cpp
)
target_link_libraries(front_unit_following_controller_test_2d PUBLIC
  Python3::Python
  Python3::NumPy
)
target_include_directories(front_unit_following_controller_test_2d PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/test
)
# 抑制 vendored matplotlibcpp.h 中已弃用的 Python C API 警告
target_compile_options(front_unit_following_controller_test_2d PRIVATE -Wno-deprecated-declarations)

add_library(front_unit_following_controller_3d_core STATIC
  src/front_unit_following_controller_3d.cpp
)
target_include_directories(front_unit_following_controller_3d_core PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

add_executable(front_unit_following_controller_test_3d
  test/front_unit_following_controller_test_3d.cpp
)
target_link_libraries(front_unit_following_controller_test_3d PUBLIC
  front_unit_following_controller_3d_core
  Matplot++::matplot
)

add_executable(realtime_front_unit_controller_2d
  src/realtime_front_unit_controller_2d_node.cpp
  src/front_unit_following_controller_2d.cpp
)
target_link_libraries(realtime_front_unit_controller_2d PUBLIC
  ${asr_sdm_control_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${std_msgs_TARGETS}
  rclcpp::rclcpp
)

add_executable(realtime_front_unit_controller_3d
  src/realtime_front_unit_controller_3d_node.cpp
)
target_link_libraries(realtime_front_unit_controller_3d PUBLIC
  front_unit_following_controller_3d_core
  ${asr_sdm_control_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${std_msgs_TARGETS}
  rclcpp::rclcpp
)

add_executable(realtime_controller_visualizer_2d src/realtime_controller_visualizer_2d_node.cpp)
target_link_libraries(realtime_controller_visualizer_2d PUBLIC
  ${std_msgs_TARGETS}
  rclcpp::rclcpp
  Python3::Python
  Python3::NumPy
)
target_include_directories(realtime_controller_visualizer_2d PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/test
)
target_compile_options(realtime_controller_visualizer_2d PRIVATE -Wno-deprecated-declarations)

add_executable(realtime_controller_visualizer_3d
  src/realtime_controller_visualizer_3d_node.cpp
)
target_link_libraries(realtime_controller_visualizer_3d PUBLIC
  front_unit_following_controller_3d_core
  ${std_msgs_TARGETS}
  rclcpp::rclcpp
  Matplot++::matplot
)

install(TARGETS
  asr_sdm_control_manager
  front_unit_following_controller_test_2d
  front_unit_following_controller_test_3d
  realtime_front_unit_controller_2d
  realtime_front_unit_controller_3d
  realtime_controller_visualizer_2d
  realtime_controller_visualizer_3d
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
