cmake_minimum_required(VERSION 3.8)
project(cmake_utils)

find_package(ament_cmake REQUIRED)

# Install the CMake module finders (FindEigen.cmake, FindGSL.cmake, ...) and the
# helper macros (arch.cmake, color.cmake). They are made available to downstream
# packages through the config-extras file below.
install(DIRECTORY cmake_modules
  DESTINATION share/${PROJECT_NAME}/cmake)
install(DIRECTORY cmake/
  DESTINATION share/${PROJECT_NAME}/cmake/cmake_helpers)

ament_package(CONFIG_EXTRAS "cmake_utils-extras.cmake")
