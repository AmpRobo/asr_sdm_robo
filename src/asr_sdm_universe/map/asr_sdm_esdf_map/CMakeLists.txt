cmake_minimum_required(VERSION 3.8)

if(POLICY CMP0074)
  cmake_policy(SET CMP0074 NEW)
endif()
if(POLICY CMP0144)
  cmake_policy(SET CMP0144 NEW)
  set(CMAKE_POLICY_DEFAULT_CMP0144 NEW)
endif()

project(asr_sdm_esdf_map)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_compile_options(-Wall -O3)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/binary_map_io.cpp
  src/esdf_map.cpp
  src/edt_environment.cpp
  src/obj_predictor.cpp
  src/raycast.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

ament_target_dependencies(${PROJECT_NAME}
  ament_index_cpp
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
  cv_bridge
  message_filters
  pcl_conversions
)

target_link_libraries(${PROJECT_NAME}
  Eigen3::Eigen
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
)

add_executable(esdf_map_test
  test/esdf_map_test.cpp
)

target_include_directories(esdf_map_test PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${PCL_INCLUDE_DIRS}
)

ament_target_dependencies(esdf_map_test
  cv_bridge
  message_filters
  nav_msgs
  rclcpp
  sensor_msgs
  std_msgs
  geometry_msgs
  visualization_msgs
  pcl_conversions
)

target_link_libraries(esdf_map_test
  ${PROJECT_NAME}
  ${PCL_LIBRARIES}
)

# Export only the reusable library. Executables must not be part of the
# package export set, otherwise downstream packages try to link them as
# imported library targets.
install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(
  TARGETS esdf_map_test
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config launch maps test
  DESTINATION share/${PROJECT_NAME}
  PATTERN "*.cpp" EXCLUDE
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(
  ament_index_cpp
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
  cv_bridge
  message_filters
  pcl_conversions
  Eigen3
  PCL
  OpenCV
)

ament_package()
