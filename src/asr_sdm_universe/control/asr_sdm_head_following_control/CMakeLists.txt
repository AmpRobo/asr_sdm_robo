cmake_minimum_required(VERSION 3.8)
project(asr_sdm_head_following_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(asr_sdm_control_msgs REQUIRED)

option(BUILD_CONTROLLER_DEMOS "Build the controller plotting demos" OFF)

add_library(front_unit_following_controller_2d_core STATIC
  src/front_unit_following_controller_2d.cpp
)
target_include_directories(front_unit_following_controller_2d_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_library(front_unit_following_controller_3d_core STATIC
  src/front_unit_following_controller_3d.cpp
)
target_include_directories(front_unit_following_controller_3d_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(front_unit_following_controller_3d_core PUBLIC
  ${asr_sdm_control_msgs_TARGETS}
)

install(TARGETS front_unit_following_controller_2d_core front_unit_following_controller_3d_core
  EXPORT asr_sdm_head_following_control_targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_targets(asr_sdm_head_following_control_targets HAS_LIBRARY_TARGET)
ament_export_dependencies(asr_sdm_control_msgs)

if(BUILD_CONTROLLER_DEMOS)
  find_package(Python3 REQUIRED COMPONENTS Interpreter Development NumPy)
  find_package(Matplot++ QUIET)
  if(NOT TARGET Matplot++::matplot)
    set(MATPLOTPP_SOURCE_DIR "" CACHE PATH "Path to the Matplot++ source tree")
    if(NOT MATPLOTPP_SOURCE_DIR OR NOT EXISTS "${MATPLOTPP_SOURCE_DIR}/CMakeLists.txt")
      message(FATAL_ERROR
        "Matplot++ was not found. Install Matplot++ or configure "
        "-DMATPLOTPP_SOURCE_DIR=/path/to/matplotplusplus.")
    endif()
    set(MATPLOTPP_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
    set(MATPLOTPP_BUILD_TESTS OFF CACHE BOOL "" FORCE)
    set(MATPLOTPP_BUILD_INSTALLER OFF CACHE BOOL "" FORCE)
    add_subdirectory(
      "${MATPLOTPP_SOURCE_DIR}"
      ${CMAKE_CURRENT_BINARY_DIR}/matplotplusplus
      EXCLUDE_FROM_ALL
    )
  endif()

  add_executable(front_unit_following_controller_test_2d
    test/front_unit_following_controller_test_2d.cpp
  )
  target_link_libraries(front_unit_following_controller_test_2d PUBLIC
    front_unit_following_controller_2d_core
    Python3::Python
    Python3::NumPy
  )
  target_include_directories(front_unit_following_controller_test_2d PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/test
  )
  target_compile_options(front_unit_following_controller_test_2d PRIVATE
    -Wno-deprecated-declarations
  )

  add_executable(front_unit_following_controller_test_3d
    test/front_unit_following_controller_test_3d.cpp
  )
  target_link_libraries(front_unit_following_controller_test_3d PUBLIC
    front_unit_following_controller_3d_core
    Matplot++::matplot
  )

  install(TARGETS
    front_unit_following_controller_test_2d
    front_unit_following_controller_test_3d
    DESTINATION lib/${PROJECT_NAME}
  )
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  # matplotlibcpp.h is vendored test code and does not follow this package's style.
  set(AMENT_LINT_AUTO_EXCLUDE "ament_cmake_uncrustify;ament_uncrustify;uncrustify")
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
