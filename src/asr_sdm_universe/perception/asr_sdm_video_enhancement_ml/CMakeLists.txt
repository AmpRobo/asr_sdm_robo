cmake_minimum_required(VERSION 3.8)
project(asr_sdm_video_enhancement_ml)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Prefer CUDA-enabled OpenCV (same hint as tensorrt_common)
if(DEFINED ENV{OpenCV_DIR} AND EXISTS "$ENV{OpenCV_DIR}")
  set(OpenCV_DIR "$ENV{OpenCV_DIR}")
elseif(EXISTS "$ENV{HOME}/.local/opencv-cuda/lib/cmake/opencv4")
  set(OpenCV_DIR "$ENV{HOME}/.local/opencv-cuda/lib/cmake/opencv4")
endif()

find_package(ament_cmake REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tensorrt_common REQUIRED)

add_executable(asr_sdm_video_enhancement_ml_node src/asr_sdm_video_enhancement_ml_node.cpp)
target_include_directories(asr_sdm_video_enhancement_ml_node PRIVATE
  include
  ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(asr_sdm_video_enhancement_ml_node
  tensorrt_common::tensorrt_common
  ${OpenCV_LIBS}
)
ament_target_dependencies(asr_sdm_video_enhancement_ml_node
  cv_bridge
  image_transport
  rclcpp
  sensor_msgs
)

install(TARGETS asr_sdm_video_enhancement_ml_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY config launch models
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_flake8_FOUND TRUE)
  set(ament_cmake_pep257_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
