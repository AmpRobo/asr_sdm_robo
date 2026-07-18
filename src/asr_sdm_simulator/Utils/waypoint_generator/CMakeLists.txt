cmake_minimum_required(VERSION 3.8)
project(waypoint_generator)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)

add_executable(waypoint_generator src/waypoint_generator.cpp)
target_include_directories(waypoint_generator PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${Boost_INCLUDE_DIRS})
target_link_libraries(waypoint_generator Eigen3::Eigen ${Boost_LIBRARIES})
ament_target_dependencies(waypoint_generator
  rclcpp nav_msgs geometry_msgs tf2 tf2_geometry_msgs)

install(TARGETS waypoint_generator
  DESTINATION lib/${PROJECT_NAME})

ament_package()
