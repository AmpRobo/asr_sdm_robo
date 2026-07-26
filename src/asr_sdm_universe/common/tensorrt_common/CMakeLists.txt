cmake_minimum_required(VERSION 3.8)
project(tensorrt_common)

# Honor <Package>_ROOT hints (e.g. CUDAToolkit_ROOT)
if(POLICY CMP0074)
  cmake_policy(SET CMP0074 NEW)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-deprecated-declarations)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ---------------------------------------------------------------------------
# ccache
# ---------------------------------------------------------------------------
find_program(CCACHE_PROGRAM ccache)
if(CCACHE_PROGRAM)
  set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE "${CCACHE_PROGRAM}")
  message(STATUS "ccache: found")
else()
  message(STATUS "ccache: not found")
endif()

find_package(ament_cmake REQUIRED)

# Prefer user-local CUDA-enabled OpenCV if present / OpenCV_DIR is set
if(DEFINED ENV{OpenCV_DIR} AND EXISTS "$ENV{OpenCV_DIR}")
  set(OpenCV_DIR "$ENV{OpenCV_DIR}")
elseif(EXISTS "$ENV{HOME}/.local/opencv-cuda/lib/cmake/opencv4")
  set(OpenCV_DIR "$ENV{HOME}/.local/opencv-cuda/lib/cmake/opencv4")
endif()

find_package(OpenCV REQUIRED)

# This package requires OpenCV built with CUDA modules
find_path(OpenCV_CUDAIMGPROC_HEADER opencv2/cudaimgproc.hpp
  HINTS ${OpenCV_INCLUDE_DIRS}
  PATHS
    "$ENV{HOME}/.local/opencv-cuda/include"
    /usr/local/include
    /usr/include
  PATH_SUFFIXES opencv4
)
if(NOT OpenCV_CUDAIMGPROC_HEADER)
  message(WARNING
    "OpenCV CUDA headers not found (opencv2/cudaimgproc.hpp). "
    "Run: ./scripts/install_opencv_cuda.sh && source ./scripts/setup_opencv_cuda_env.sh "
    "(see README.md). Skipping ${PROJECT_NAME} build.")
  ament_package()
  return()
endif()
message(STATUS "OpenCV CUDA header: ${OpenCV_CUDAIMGPROC_HEADER}")
message(STATUS "OpenCV Version: ${OpenCV_VERSION}")
message(STATUS "OpenCV_DIR: ${OpenCV_DIR}")

# ---------------------------------------------------------------------------
# CUDA (prefer CUDAToolkit; do not use deprecated FindCUDA / CMP0146)
# ---------------------------------------------------------------------------
# Resolve CUDA toolkit root from env or common install locations.
if(DEFINED ENV{CUDA_HOME} AND EXISTS "$ENV{CUDA_HOME}")
  set(_CUDA_CANDIDATE "$ENV{CUDA_HOME}")
elseif(DEFINED ENV{CUDA_PATH} AND EXISTS "$ENV{CUDA_PATH}")
  set(_CUDA_CANDIDATE "$ENV{CUDA_PATH}")
elseif(DEFINED ENV{CUDA_VERSION} AND EXISTS "/usr/local/cuda-$ENV{CUDA_VERSION}")
  set(_CUDA_CANDIDATE "/usr/local/cuda-$ENV{CUDA_VERSION}")
elseif(EXISTS "/usr/local/cuda")
  set(_CUDA_CANDIDATE "/usr/local/cuda")
else()
  file(GLOB _CUDA_GLOB_CANDIDATES
    "/usr/local/cuda-*"
    "$ENV{HOME}/.local/cuda-*-root/usr/local/cuda-*"
  )
  if(_CUDA_GLOB_CANDIDATES)
    list(SORT _CUDA_GLOB_CANDIDATES ORDER DESCENDING)
    list(GET _CUDA_GLOB_CANDIDATES 0 _CUDA_CANDIDATE)
  endif()
endif()

if(DEFINED _CUDA_CANDIDATE AND EXISTS "${_CUDA_CANDIDATE}")
  set(CUDA_TOOLKIT_ROOT_DIR "${_CUDA_CANDIDATE}" CACHE PATH "CUDA toolkit root" FORCE)
  set(CUDAToolkit_ROOT "${_CUDA_CANDIDATE}" CACHE PATH "CUDAToolkit root" FORCE)
  if(EXISTS "${_CUDA_CANDIDATE}/bin/nvcc")
    set(CMAKE_CUDA_COMPILER "${_CUDA_CANDIDATE}/bin/nvcc")
  endif()
  message(STATUS "Using CUDA toolkit root: ${_CUDA_CANDIDATE}")
else()
  message(STATUS "CUDA toolkit root not auto-detected; set CUDA_HOME or CUDAToolkit_ROOT")
endif()

find_package(CUDAToolkit)
if(NOT CUDAToolkit_FOUND)
  # Fallback: locate runtime headers / libcudart without a full CUDAToolkit package
  find_path(CUDA_INCLUDE_DIRS cuda_runtime.h
    HINTS
      ${CUDA_TOOLKIT_ROOT_DIR}
      ${CUDA_TOOLKIT_ROOT_DIR}/targets/x86_64-linux
      /usr/local/cuda
      /usr
    PATH_SUFFIXES include include/cuda
  )
  find_library(CUDA_CUDART_LIBRARY cudart
    HINTS
      ${CUDA_TOOLKIT_ROOT_DIR}
      ${CUDA_TOOLKIT_ROOT_DIR}/targets/x86_64-linux
      /usr/local/cuda
      /usr
    PATH_SUFFIXES lib64 lib lib/x86_64-linux-gnu
  )
  if(CUDA_INCLUDE_DIRS AND CUDA_CUDART_LIBRARY)
    set(CUDAToolkit_FOUND TRUE)
    if(NOT TARGET CUDA::cudart)
      add_library(CUDA::cudart UNKNOWN IMPORTED)
      set_target_properties(CUDA::cudart PROPERTIES
        IMPORTED_LOCATION "${CUDA_CUDART_LIBRARY}"
        INTERFACE_INCLUDE_DIRECTORIES "${CUDA_INCLUDE_DIRS}"
      )
    endif()
    message(STATUS "CUDAToolkit package missing; using fallback cudart: ${CUDA_CUDART_LIBRARY}")
  endif()
endif()

# ---------------------------------------------------------------------------
# TensorRT (Debian/Ubuntu multiarch: include/x86_64-linux-gnu)
# ---------------------------------------------------------------------------
include(FindPackageHandleStandardArgs)

if(NOT DEFINED TENSORRT_ROOT)
  if(DEFINED ENV{TENSORRT_ROOT})
    set(TENSORRT_ROOT "$ENV{TENSORRT_ROOT}")
  elseif(DEFINED ENV{TENSORRT_DIR})
    set(TENSORRT_ROOT "$ENV{TENSORRT_DIR}")
  elseif(DEFINED TENSORRT_DIR)
    set(TENSORRT_ROOT "${TENSORRT_DIR}")
  elseif(DEFINED ENV{TensorRT_DIR})
    set(TENSORRT_ROOT "$ENV{TensorRT_DIR}")
  elseif(DEFINED TensorRT_DIR)
    set(TENSORRT_ROOT "${TensorRT_DIR}")
  else()
    set(TENSORRT_ROOT "/usr" CACHE PATH "Folder contains NVIDIA TensorRT")
  endif()
endif()

find_path(TENSORRT_INCLUDE_DIR NvInfer.h
  HINTS ${TENSORRT_ROOT} ${CUDA_TOOLKIT_ROOT_DIR}
  PATH_SUFFIXES
    include
    include/x86_64-linux-gnu
    targets/x86_64-linux/include
)
message(STATUS "Found TensorRT headers at ${TENSORRT_INCLUDE_DIR}")

if(TENSORRT_INCLUDE_DIR)
  if(EXISTS "${TENSORRT_INCLUDE_DIR}/NvInferVersion.h")
    file(READ "${TENSORRT_INCLUDE_DIR}/NvInferVersion.h" TENSORRT_H_CONTENTS)
  else()
    file(READ "${TENSORRT_INCLUDE_DIR}/NvInfer.h" TENSORRT_H_CONTENTS)
  endif()

  string(REGEX MATCH "define[ \t]+NV_TENSORRT_MAJOR[ \t]+([0-9]+)" _ "${TENSORRT_H_CONTENTS}")
  set(TENSORRT_VERSION_MAJOR "${CMAKE_MATCH_1}")

  string(REGEX MATCH "define[ \t]+NV_TENSORRT_MINOR[ \t]+([0-9]+)" _ "${TENSORRT_H_CONTENTS}")
  set(TENSORRT_VERSION_MINOR "${CMAKE_MATCH_1}")

  string(REGEX MATCH "define[ \t]+NV_TENSORRT_PATCH[ \t]+([0-9]+)" _ "${TENSORRT_H_CONTENTS}")
  set(TENSORRT_VERSION_PATCH "${CMAKE_MATCH_1}")

  # TRT 10.11+ aliases NV_TENSORRT_* to TRT_*_ENTERPRISE
  if(NOT TENSORRT_VERSION_MAJOR)
    string(REGEX MATCH "define[ \t]+TRT_MAJOR_ENTERPRISE[ \t]+([0-9]+)" _ "${TENSORRT_H_CONTENTS}")
    set(TENSORRT_VERSION_MAJOR "${CMAKE_MATCH_1}")

    string(REGEX MATCH "define[ \t]+TRT_MINOR_ENTERPRISE[ \t]+([0-9]+)" _ "${TENSORRT_H_CONTENTS}")
    set(TENSORRT_VERSION_MINOR "${CMAKE_MATCH_1}")

    string(REGEX MATCH "define[ \t]+TRT_PATCH_ENTERPRISE[ \t]+([0-9]+)" _ "${TENSORRT_H_CONTENTS}")
    set(TENSORRT_VERSION_PATCH "${CMAKE_MATCH_1}")
  endif()

  set(TENSORRT_VERSION
    "${TENSORRT_VERSION_MAJOR}.${TENSORRT_VERSION_MINOR}.${TENSORRT_VERSION_PATCH}"
    CACHE STRING "TensorRT version"
  )

  unset(TENSORRT_H_CONTENTS)
endif()

find_library(TENSORRT_NVINFER_LIBRARY nvinfer
  HINTS ${TENSORRT_ROOT} ${CUDA_TOOLKIT_ROOT_DIR}
  PATH_SUFFIXES lib lib64 lib/x64 lib/x86_64-linux-gnu)
find_library(TENSORRT_NVINFER_PLUGIN_LIBRARY nvinfer_plugin
  HINTS ${TENSORRT_ROOT} ${CUDA_TOOLKIT_ROOT_DIR}
  PATH_SUFFIXES lib lib64 lib/x64 lib/x86_64-linux-gnu)
find_library(TENSORRT_NVONNXPARSER_LIBRARY nvonnxparser
  HINTS ${TENSORRT_ROOT} ${CUDA_TOOLKIT_ROOT_DIR}
  PATH_SUFFIXES lib lib64 lib/x64 lib/x86_64-linux-gnu)
set(TENSORRT_LIBRARY
  ${TENSORRT_NVINFER_LIBRARY}
  ${TENSORRT_NVINFER_PLUGIN_LIBRARY}
  ${TENSORRT_NVONNXPARSER_LIBRARY})
if(TENSORRT_VERSION_MAJOR VERSION_LESS 9)
  find_library(TENSORRT_NVPARSERS_LIBRARY nvparsers
    HINTS ${TENSORRT_ROOT} ${CUDA_TOOLKIT_ROOT_DIR}
    PATH_SUFFIXES lib lib64 lib/x64 lib/x86_64-linux-gnu)
  list(APPEND TENSORRT_LIBRARY ${TENSORRT_NVPARSERS_LIBRARY})
endif()
message(STATUS "Found TensorRT libs at ${TENSORRT_LIBRARY}")

find_package_handle_standard_args(TENSORRT
  FOUND_VAR TENSORRT_FOUND
  REQUIRED_VARS
    TENSORRT_INCLUDE_DIR TENSORRT_LIBRARY
  VERSION_VAR TENSORRT_VERSION
)

if(TENSORRT_FOUND)
  set(TENSORRT_LIBRARIES ${TENSORRT_LIBRARY})
  set(TENSORRT_INCLUDE_DIRS ${TENSORRT_INCLUDE_DIR})
endif()

mark_as_advanced(
  TENSORRT_LIBRARY
  TENSORRT_INCLUDE_DIR
  TENSORRT_VERSION
)

if(NOT CUDAToolkit_FOUND)
  message(WARNING
    "CUDA not found. Install CUDA Toolkit or set CUDA_HOME "
    "(e.g. export CUDA_HOME=$HOME/.local/cuda-13.3-root/usr/local/cuda-13.3). "
    "Skipping ${PROJECT_NAME} build.")
  ament_package()
  return()
endif()

if(NOT TENSORRT_FOUND)
  message(WARNING
    "TensorRT not found. Install libnvinfer-dev / libnvonnxparsers-dev "
    "or set TENSORRT_ROOT. Skipping ${PROJECT_NAME} build.")
  ament_package()
  return()
endif()

message(STATUS "CUDA toolkit: ${CUDA_TOOLKIT_ROOT_DIR}")
message(STATUS "CUDAToolkit version: ${CUDAToolkit_VERSION}")
message(STATUS "TensorRT Include: ${TENSORRT_INCLUDE_DIRS}")
message(STATUS "TensorRT Libraries: ${TENSORRT_LIBRARIES}")
message(STATUS "TensorRT Version: ${TENSORRT_VERSION}")
message(STATUS "OpenCV Version: ${OpenCV_VERSION}")

add_library(${PROJECT_NAME} SHARED
  src/engine.cpp
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${OpenCV_INCLUDE_DIRS}
    ${TENSORRT_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}
  PUBLIC
    ${OpenCV_LIBS}
    CUDA::cudart
    ${CMAKE_THREAD_LIBS_INIT}
    ${TENSORRT_LIBRARIES}
)

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)

# Optional test / benchmark executable
option(TENSORRT_COMMON_BUILD_BENCHMARK "Build tensorrt_common_test executable" ON)
if(TENSORRT_COMMON_BUILD_BENCHMARK)
  add_executable(tensorrt_common_test test/tensorrt_common_test.cpp)
  target_link_libraries(tensorrt_common_test PRIVATE ${PROJECT_NAME})
endif()

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(TENSORRT_COMMON_BUILD_BENCHMARK)
  install(
    TARGETS tensorrt_common_test
    RUNTIME DESTINATION lib/${PROJECT_NAME}
  )
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  OpenCV
  CUDAToolkit
)

ament_package()
