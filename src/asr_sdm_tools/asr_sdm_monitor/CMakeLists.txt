cmake_minimum_required(VERSION 3.21)
project(asr_sdm_monitor)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_AUTOMOC ON)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(Qt6 REQUIRED COMPONENTS Core Gui Widgets)
find_package(Slint REQUIRED)

add_executable(${PROJECT_NAME}
    src/main.cpp
    src/slint_ui_controller.cpp
    src/chart_image.cpp
    src/ros_executor_manager.cpp
    src/ros_ui_bridge.cpp
    src/ros_ui_bridge_hardware.cpp
    src/ros_ui_bridge_plot.cpp
    src/ros_ui_bridge_video.cpp
    src/ros_ui_bridge_simulator.cpp
    src/system_monitor/monitor_utils.cpp
    src/system_monitor/cpu_monitor.cpp
    src/system_monitor/mem_monitor.cpp
    src/system_monitor/hdd_monitor.cpp
    src/system_monitor/net_monitor.cpp
    src/system_monitor/system_monitor.cpp
    include/asr_sdm_monitor/ros_ui_bridge.hpp
    include/asr_sdm_monitor/ros_executor_manager.hpp
    include/asr_sdm_monitor/slint_ui_controller.hpp
    include/asr_sdm_monitor/chart_image.hpp
    include/asr_sdm_monitor/system_monitor/monitor_utils.hpp
    include/asr_sdm_monitor/system_monitor/cpu_monitor.hpp
    include/asr_sdm_monitor/system_monitor/mem_monitor.hpp
    include/asr_sdm_monitor/system_monitor/hdd_monitor.hpp
    include/asr_sdm_monitor/system_monitor/net_monitor.hpp
    include/asr_sdm_monitor/system_monitor/system_monitor.hpp
)

slint_target_sources(${PROJECT_NAME} ui/main_window.slint NAMESPACE asr_sdm_monitor_ui)

set_source_files_properties(
    src/main.cpp
    src/slint_ui_controller.cpp
    PROPERTIES OBJECT_DEPENDS "${CMAKE_CURRENT_BINARY_DIR}/main_window.h")

target_include_directories(${PROJECT_NAME} PRIVATE
    include
    ${rosbag2_cpp_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME} PRIVATE
    Slint::Slint
    Qt6::Core
    Qt6::Gui
    Qt6::Widgets
    rclcpp::rclcpp
    ament_index_cpp::ament_index_cpp
    diagnostic_updater::diagnostic_updater
    ${diagnostic_msgs_TARGETS}
    ${sensor_msgs_TARGETS}
    ${geometry_msgs_TARGETS}
    ${std_msgs_TARGETS}
    ${rosbag2_cpp_TARGETS}
    ${rosbag2_cpp_LIBRARIES}
)

if(TARGET rosbag2_cpp::rosbag2_cpp)
    target_link_libraries(${PROJECT_NAME} PRIVATE rosbag2_cpp::rosbag2_cpp)
endif()

install(TARGETS ${PROJECT_NAME}
    RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config
    DESTINATION share/${PROJECT_NAME})

ament_package()
