# Config extras for cmake_utils: exposes the shared CMake modules and macros to
# any package that does find_package(cmake_utils).

# Make FindEigen.cmake / FindGSL.cmake / ... discoverable.
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake_modules")

# Provide the color variables and target_architecture() macro.
include("${CMAKE_CURRENT_LIST_DIR}/cmake_helpers/color.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/cmake_helpers/arch.cmake")
