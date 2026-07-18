cmake_minimum_required(VERSION 3.8)
project(so3_disturbance_generator)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pose_utils REQUIRED)
find_package(Armadillo REQUIRED)

add_executable(so3_disturbance_generator src/so3_disturbance_generator.cpp)
target_include_directories(so3_disturbance_generator PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${ARMADILLO_INCLUDE_DIRS}
)
ament_target_dependencies(so3_disturbance_generator
  rclcpp std_msgs sensor_msgs nav_msgs geometry_msgs visualization_msgs pose_utils)
target_link_libraries(so3_disturbance_generator ${ARMADILLO_LIBRARIES})

install(TARGETS so3_disturbance_generator
  DESTINATION lib/${PROJECT_NAME})

ament_package()
