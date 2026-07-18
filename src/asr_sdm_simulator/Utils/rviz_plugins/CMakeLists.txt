cmake_minimum_required(VERSION 3.8)
project(rviz_plugins)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(multi_map_server REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core Widgets)

set(CMAKE_AUTOMOC ON)

qt5_wrap_cpp(MOC_FILES
  src/goal_tool.h
  src/static_obstacle_tool.h
  src/probmap_display.h
  src/aerialmap_display.h
  src/multi_probmap_display.h
)

set(SOURCE_FILES
  src/pose_tool.cpp
  src/goal_tool.cpp
  src/static_obstacle_tool.cpp
  src/probmap_display.cpp
  src/aerialmap_display.cpp
  src/multi_probmap_display.cpp
  ${MOC_FILES}
)

add_library(${PROJECT_NAME} SHARED ${SOURCE_FILES})

target_include_directories(${PROJECT_NAME} PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_link_libraries(${PROJECT_NAME}
  rviz_common::rviz_common
  rviz_rendering::rviz_rendering
  Qt5::Core
  Qt5::Widgets
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  pluginlib
  nav_msgs
  geometry_msgs
  std_msgs
  multi_map_server
  tf2
  tf2_geometry_msgs
)

pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(FILES plugin_description.xml
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME})

ament_export_dependencies(
  rclcpp pluginlib rviz_common rviz_rendering
  nav_msgs geometry_msgs std_msgs multi_map_server tf2 tf2_geometry_msgs)

ament_package()
