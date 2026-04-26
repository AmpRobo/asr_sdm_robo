cmake_minimum_required(VERSION 3.20)
project(asr_sdm_monitor)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Qt6 REQUIRED COMPONENTS Core Gui Qml Quick QuickControls2)

qt_standard_project_setup()

qt_add_executable(${PROJECT_NAME}
  src/main.cpp
  src/ros_ui_bridge.cpp
  src/ros_video_image_provider.cpp
  include/asr_sdm_monitor/ros_ui_bridge.hpp
  include/asr_sdm_monitor/ros_video_image_provider.hpp
)

qt_add_resources(${PROJECT_NAME} "qml_resources"
  PREFIX "/qt/qml/AsrSdmMonitor"
  FILES
    qml/Main.qml
    qml/components/SelectableButton.qml
    qml/components/TopControlBar.qml
    qml/components/SidebarMenu.qml
    qml/components/HardwareTabBar.qml
    qml/components/ValueCard.qml
    qml/components/SectionTitle.qml
    qml/components/SimpleTable.qml
    qml/components/AreaLineChart.qml
    qml/pages/HardwarePage.qml
    qml/pages/VideoPage.qml
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
)

target_link_libraries(${PROJECT_NAME} PRIVATE
  Qt6::Core
  Qt6::Gui
  Qt6::Qml
  Qt6::Quick
  Qt6::QuickControls2
  rclcpp::rclcpp
  ${diagnostic_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${std_msgs_TARGETS}
)

install(TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_package()
