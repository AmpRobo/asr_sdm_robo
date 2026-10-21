cmake_minimum_required(VERSION 3.14)
project(pseudo_random)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 11)
endif()

if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# Shared library so pcg32_global is one state per process.
# rng_zig_layers.c holds the precomputed Ziggurat tables used by rng_norm.
add_library(${PROJECT_NAME} SHARED
  src/rng/rng.c
  src/rng/rng_zig_layers.c
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME} PUBLIC m)

add_library(${PROJECT_NAME}::${PROJECT_NAME} ALIAS ${PROJECT_NAME})

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY src/rng/
  DESTINATION include/rng
  FILES_MATCHING
  PATTERN "*.h"
)

ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)

if(BUILD_TESTING)
  add_executable(${PROJECT_NAME}_smoke test/smoke.c)
  target_link_libraries(${PROJECT_NAME}_smoke ${PROJECT_NAME})

  ament_add_test(${PROJECT_NAME}_smoke
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
    COMMAND "$<TARGET_FILE:${PROJECT_NAME}_smoke>"
    APPEND_LIBRARY_DIRS "$<TARGET_FILE_DIR:${PROJECT_NAME}>"
  )
endif()

ament_package()
