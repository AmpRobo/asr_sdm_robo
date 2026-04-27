cmake_minimum_required(VERSION 3.20)
project(asr_sdm_geometry)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

add_library(${PROJECT_NAME} INTERFACE)

file(GLOB_RECURSE ${PROJECT_NAME}_HEADERS
  RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}"
  CONFIGURE_DEPENDS
  "include/${PROJECT_NAME}/*.hpp"
)

set(${PROJECT_NAME}_BUILD_HEADERS "")
set(${PROJECT_NAME}_INSTALL_HEADERS "")
foreach(_header IN LISTS ${PROJECT_NAME}_HEADERS)
  list(APPEND ${PROJECT_NAME}_BUILD_HEADERS
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/${_header}>")
  list(APPEND ${PROJECT_NAME}_INSTALL_HEADERS
    "$<INSTALL_INTERFACE:${_header}>")
endforeach()

target_sources(${PROJECT_NAME} INTERFACE
  ${${PROJECT_NAME}_BUILD_HEADERS}
  ${${PROJECT_NAME}_INSTALL_HEADERS}
)

target_include_directories(${PROJECT_NAME} INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

install(TARGETS ${PROJECT_NAME}
  EXPORT "export_${PROJECT_NAME}"
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib
  INCLUDES DESTINATION include
)

install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Remove these two lines when adding proper copyright headers.
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_targets("export_${PROJECT_NAME}")
ament_package()
