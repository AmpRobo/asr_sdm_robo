cmake_minimum_required(VERSION 3.8)
project(asr_sdm_esdf_map)

set(CMAKE_SUPPRESS_DEVELOPER_WARNINGS 1 CACHE BOOL "Suppress CMake developer warnings" FORCE)

if(POLICY CMP0074)
  cmake_policy(SET CMP0074 NEW)
endif()

if(POLICY CMP0144)
  cmake_policy(SET CMP0144 NEW)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(pcl_conversions REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)
find_package(OpenCV REQUIRED)
set(Eigen3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})

include_directories(${PROJECT_SOURCE_DIR}/include)

include_directories(
    include
    SYSTEM
    ${Eigen3_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})

set(CMAKE_CXX_FLAGS "-std=c++17 ${CMAKE_CXX_FLAGS} -O3 -Wall -pthread") # -Wextra -Werror
set(CMAKE_BUILD_TYPE "RELEASE")
#add_executable(test_esdf_map
#        include/ESDFMap.h
#        src/ESDFMap.cpp
#        include/timing.h
#        src/timing.cc
#        test/test_ESDF_Map.cpp
#        )
#target_link_libraries(test_esdf_map
#        ${catkin_LIBRARIES}
#        )
#FIND_PACKAGE( OpenMP REQUIRED)
#if(OPENMP_FOUND)
#message("OPENMP FOUND")
#set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
#set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
#endif()

add_executable(test_fiesta
        include/asr_sdm_esdf_map/parameters.hpp
        src/parameters.cpp
        include/asr_sdm_esdf_map/timing.hpp
        src/timing.cc
        include/asr_sdm_esdf_map/raycast.hpp
        src/raycast.cpp
        include/asr_sdm_esdf_map/ESDFMap.hpp
        src/ESDFMap.cpp
        include/asr_sdm_esdf_map/asr_sdm_esdf_map.hpp
        test/test_fiesta.cpp
        )
target_link_libraries(test_fiesta
        ${PCL_LIBRARIES}
        ${OpenCV_LIBS}
        )
ament_target_dependencies(test_fiesta
        rclcpp
        std_msgs
        nav_msgs
        visualization_msgs
        sensor_msgs
        geometry_msgs
        cv_bridge
        pcl_conversions
)

install(TARGETS test_fiesta
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*_launch.py"
)

install(
  FILES demo.rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

