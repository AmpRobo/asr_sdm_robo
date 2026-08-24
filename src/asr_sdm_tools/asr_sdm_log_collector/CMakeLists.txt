cmake_minimum_required(VERSION 3.20)
project(asr_sdm_log_collector)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(spdlog REQUIRED)

# Sending side. Deliberately free of any ROS dependency so that plain processes
# on the robot can report into the same collector as the nodes do.
add_library(asr_sdm_log_client SHARED
  src/log_record.cpp
  src/path_utils.cpp
  src/collector_sink.cpp
  src/log_client.cpp
)
target_include_directories(asr_sdm_log_client PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
target_link_libraries(asr_sdm_log_client PUBLIC spdlog::spdlog)

# Receiving side.
add_executable(asr_sdm_log_collector_node
  src/datagram_receiver.cpp
  src/log_writer.cpp
  src/log_collector_node.cpp
  src/asr_sdm_log_collector_node.cpp
)
target_link_libraries(asr_sdm_log_collector_node PRIVATE
  asr_sdm_log_client
  rclcpp::rclcpp
  ${rcl_interfaces_TARGETS}
)

add_executable(asr_sdm_log_client_demo src/log_client_demo.cpp)
target_link_libraries(asr_sdm_log_client_demo PRIVATE asr_sdm_log_client)
# The SPDLOG_* macros below info are compiled out unless the active level is
# lowered, and the demo is meant to show every level reaching the collector.
target_compile_definitions(asr_sdm_log_client_demo PRIVATE
  SPDLOG_ACTIVE_LEVEL=SPDLOG_LEVEL_TRACE
)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(
  TARGETS asr_sdm_log_client
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  TARGETS
    asr_sdm_log_collector_node
    asr_sdm_log_client_demo
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

install(FILES README.md
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_log_record test/test_log_record.cpp)
  target_link_libraries(test_log_record asr_sdm_log_client)

  ament_add_gtest(test_path_utils test/test_path_utils.cpp)
  target_link_libraries(test_path_utils asr_sdm_log_client)

  find_package(ament_lint_auto REQUIRED)
  # This workspace formats with its own .clang-format and carries no per-file
  # copyright blocks, so the ament equivalents would only fight it.
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(spdlog)

ament_package()
