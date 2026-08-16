cmake_minimum_required(VERSION 3.8)
project(planning_simulator)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(asr_sdm_control_msgs REQUIRED)
find_package(uav_utils REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)

add_library(quadrotor_dynamics src/dynamics/Quadrotor.cpp)
target_include_directories(quadrotor_dynamics PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${Boost_INCLUDE_DIRS})
target_link_libraries(quadrotor_dynamics Eigen3::Eigen ${Boost_LIBRARIES})

add_executable(planning_simulator src/planning_simulator.cpp)
target_include_directories(planning_simulator PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(planning_simulator quadrotor_dynamics Eigen3::Eigen)
ament_target_dependencies(planning_simulator
  rclcpp asr_sdm_control_msgs uav_utils nav_msgs sensor_msgs geometry_msgs)

install(TARGETS planning_simulator
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS quadrotor_dynamics
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL)

ament_package()
