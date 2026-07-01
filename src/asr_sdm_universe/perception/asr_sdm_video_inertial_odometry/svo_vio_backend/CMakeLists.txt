cmake_minimum_required(VERSION 3.8)
project(svo_vio_backend)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-unused-parameter)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Sophus REQUIRED)
find_package(Ceres REQUIRED)

# Create library
add_library(svo_vio_backend SHARED
  src/vins_types.cpp
  src/vins_imu_preintegrator.cpp
  src/vins_factors.cpp
  src/vins_sliding_window.cpp
  src/vins_optimizer.cpp
  src/vio_backend.cpp
)

target_include_directories(svo_vio_backend PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Sophus_INCLUDE_DIRS}
)

target_link_libraries(svo_vio_backend PUBLIC
  ${OpenCV_LIBS}
  Sophus::Sophus
  ${CERES_LIBRARIES}
)

ament_target_dependencies(svo_vio_backend PUBLIC
  rclcpp
)

# Install
install(TARGETS svo_vio_backend
  EXPORT svo_vio_backendTargets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

# Export
ament_export_targets(svo_vio_backendTargets HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_libraries(svo_vio_backend)
ament_export_dependencies(
  rclcpp
  Eigen3
  OpenCV
  Sophus
  Ceres
)

ament_package()
