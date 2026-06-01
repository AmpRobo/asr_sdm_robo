cmake_minimum_required(VERSION 3.8)
project(asr_sdm_guidance_planner)

# Keep third-party CMake packages quiet on Ubuntu 24.04 / ROS 2 Jazzy.
# These policies remove PCL/FLANN developer warnings without changing runtime behavior.
if(POLICY CMP0074)
  cmake_policy(SET CMP0074 NEW)
endif()

if(POLICY CMP0144)
  cmake_policy(SET CMP0144 NEW)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common)
find_package(asr_sdm_lbfgs_solver REQUIRED)

set(DEPENDENCIES
  rclcpp
  ament_index_cpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  visualization_msgs
  pcl_conversions
)

add_library(${PROJECT_NAME} SHARED
  src/map/voxel_esdf_map.cpp
  src/planner/astar_3d_planner.cpp
  src/planner/sphere_corridor.cpp
  src/optimizer/lbfgs_path_optimizer.cpp
  src/node/rviz_planning_node.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}
  Eigen3::Eigen
  ${PCL_LIBRARIES}
)

if(TARGET asr_sdm_lbfgs_solver::asr_sdm_lbfgs_solver)
  target_link_libraries(${PROJECT_NAME} asr_sdm_lbfgs_solver::asr_sdm_lbfgs_solver)
else()
  target_link_libraries(${PROJECT_NAME} asr_sdm_lbfgs_solver)
endif()

ament_target_dependencies(${PROJECT_NAME} ${DEPENDENCIES})

add_executable(rviz_astar_lbfgs_planner src/rviz_planning_node_main.cpp)
target_link_libraries(rviz_astar_lbfgs_planner ${PROJECT_NAME})
ament_target_dependencies(rviz_astar_lbfgs_planner ${DEPENDENCIES})

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS ${PROJECT_NAME} rviz_astar_lbfgs_planner
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  INCLUDES DESTINATION include
)

install(
  DIRECTORY config launch rviz maps
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(${DEPENDENCIES} Eigen3 pcl_conversions ament_index_cpp asr_sdm_lbfgs_solver)
ament_package()
