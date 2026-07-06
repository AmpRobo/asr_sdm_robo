cmake_minimum_required(VERSION 3.20)
project(asr_sdm_monitor)

set(CMAKE_CXX_STANDARD 17)
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
find_package(Qt6 REQUIRED COMPONENTS Core Gui Widgets Qml Quick QuickControls2)


qt_standard_project_setup()

qt_add_executable(${PROJECT_NAME}
  src/main.cpp
  src/ros_ui_bridge.cpp
  src/ros_video_image_provider.cpp
  src/system_monitor/monitor_utils.cpp
  src/system_monitor/cpu_monitor.cpp
  src/system_monitor/mem_monitor.cpp
  src/system_monitor/hdd_monitor.cpp
  src/system_monitor/net_monitor.cpp
  src/system_monitor/system_monitor.cpp
  include/ros_ui_bridge.hpp
  include/ros_video_image_provider.hpp
  include/system_monitor/monitor_utils.hpp
  include/system_monitor/cpu_monitor.hpp
  include/system_monitor/mem_monitor.hpp
  include/system_monitor/hdd_monitor.hpp
  include/system_monitor/net_monitor.hpp
  include/system_monitor/system_monitor.hpp
)

qt_add_resources(${PROJECT_NAME} "qml_resources"
  PREFIX "/qt/qml/AsrSdmMonitor"
  FILES
    qml/Main.qml
    qml/components/SelectableButton.qml
    qml/components/ActionButton.qml
    qml/components/TopControlBar.qml
    qml/components/SidebarMenu.qml
    qml/components/HardwareTabBar.qml
    qml/components/ValueCard.qml
    qml/components/SectionTitle.qml
    qml/components/SimpleTable.qml
    qml/components/AreaLineChart.qml
    qml/components/PlotCanvas.qml
    qml/pages/HardwarePage.qml
    qml/pages/VideoPage.qml
    qml/pages/PlotPage.qml
    qml/pages/hardware/CpuPage.qml
    qml/pages/hardware/MemoryPage.qml
    qml/pages/hardware/HddPage.qml
    qml/pages/hardware/NetPage.qml
    qml/pages/hardware/NtpPage.qml
    qml/utils/Theme.js
    qml/utils/I18n.js
    qml/utils/UiHelpers.js
)

target_include_directories(${PROJECT_NAME} PRIVATE
  include
  ${rosbag2_cpp_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME} PRIVATE
  Qt6::Core
  Qt6::Gui
  Qt6::Widgets
  Qt6::Qml
  Qt6::Quick
  Qt6::QuickControls2
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
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
