cmake_minimum_required(VERSION 3.8)
project(local_sensing_node)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release")
endif()

set(ENABLE_CUDA false)
# set(ENABLE_CUDA true)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)

if(ENABLE_CUDA)
  find_package(CUDA REQUIRED)
  set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS};-O3 -use_fast_math)
  set(CUDA_NVCC_FLAGS
      -gencode arch=compute_61,code=sm_61;
  )
  set(CUDA_PROPAGATE_HOST_FLAGS OFF)

  find_package(OpenCV REQUIRED)
  find_package(Boost REQUIRED COMPONENTS system filesystem)
  find_package(cv_bridge REQUIRED)
  find_package(image_transport REQUIRED)
  find_package(asr_sdm_control_msgs REQUIRED)

  cuda_add_library(depth_render_cuda
      src/depth_render.cu
  )

  add_executable(pcl_render_node src/pcl_render_node.cpp)
  target_include_directories(pcl_render_node PRIVATE
    ${OpenCV_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${PCL_INCLUDE_DIRS}
  )
  ament_target_dependencies(pcl_render_node
    rclcpp sensor_msgs geometry_msgs nav_msgs std_msgs pcl_conversions
    cv_bridge image_transport asr_sdm_control_msgs)
  target_link_libraries(pcl_render_node
    depth_render_cuda
    ${OpenCV_LIBS}
    ${Boost_LIBRARIES}
    ${PCL_LIBRARIES}
  )

  install(TARGETS depth_render_cuda
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib)
  install(TARGETS pcl_render_node
    DESTINATION lib/${PROJECT_NAME})
else()
  add_executable(pcl_render_node src/pointcloud_render_node.cpp)
  target_include_directories(pcl_render_node PRIVATE
    ${EIGEN3_INCLUDE_DIR}
    ${PCL_INCLUDE_DIRS}
  )
  ament_target_dependencies(pcl_render_node
    rclcpp sensor_msgs geometry_msgs nav_msgs std_msgs pcl_conversions)
  target_link_libraries(pcl_render_node ${PCL_LIBRARIES})

  install(TARGETS pcl_render_node
    DESTINATION lib/${PROJECT_NAME})
endif()

install(DIRECTORY params
  DESTINATION share/${PROJECT_NAME})

ament_package()
