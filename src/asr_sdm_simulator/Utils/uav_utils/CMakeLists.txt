cmake_minimum_required(VERSION 3.8)
project(uav_utils)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# Header-only library.
add_library(uav_utils INTERFACE)
target_include_directories(uav_utils INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(uav_utils INTERFACE Eigen3::Eigen)
ament_target_dependencies(uav_utils INTERFACE nav_msgs geometry_msgs)

install(DIRECTORY include/
  DESTINATION include)

install(TARGETS uav_utils
  EXPORT export_${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(nav_msgs geometry_msgs Eigen3)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(${PROJECT_NAME}-test src/uav_utils_test.cpp)
  if(TARGET ${PROJECT_NAME}-test)
    target_link_libraries(${PROJECT_NAME}-test uav_utils)
  endif()
endif()

ament_package()
