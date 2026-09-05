cmake_minimum_required(VERSION 3.15)
project(grampc VERSION 2.3.0 LANGUAGES C CXX)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(ament_cmake REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/discrete.c
  src/finite_diff.c
  src/grampc_alloc.c
  src/grampc_configreader.c
  src/grampc_erk.c
  src/grampc_fixedsize.c
  src/grampc_init.c
  src/grampc_mess.c
  src/grampc_run.c
  src/grampc_setopt.c
  src/grampc_setparam.c
  src/grampc_util.c
  src/rodas.c
  src/ruku45.c
  src/simpson.c
  src/timing.c
  src/trapezoidal.c
  src/grampc.cpp
  src/problem_description.cpp
)
add_library(${PROJECT_NAME}::${PROJECT_NAME} ALIAS ${PROJECT_NAME})

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

find_library(MATH_LIBRARY m)
if(MATH_LIBRARY)
  target_link_libraries(${PROJECT_NAME} PUBLIC ${MATH_LIBRARY})
endif()

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include/${PROJECT_NAME}
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include/${PROJECT_NAME})

ament_package()
