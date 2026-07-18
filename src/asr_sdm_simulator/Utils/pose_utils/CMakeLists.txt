cmake_minimum_required(VERSION 3.8)
project(pose_utils)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(Armadillo REQUIRED)

add_library(pose_utils src/pose_utils.cpp)
target_include_directories(pose_utils PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${ARMADILLO_INCLUDE_DIRS})
target_link_libraries(pose_utils ${ARMADILLO_LIBRARIES})

install(DIRECTORY include/
  DESTINATION include)

install(TARGETS pose_utils
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

ament_export_include_directories(include)
ament_export_libraries(pose_utils)
ament_export_targets(export_${PROJECT_NAME})

ament_package()
