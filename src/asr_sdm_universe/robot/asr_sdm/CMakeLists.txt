cmake_minimum_required(VERSION 3.8)
project(asr_sdm)

find_package(ament_cmake REQUIRED)
find_package(xacro REQUIRED)

set(GENERATED_URDF_DIR "${CMAKE_CURRENT_BINARY_DIR}/generated")
set(GENERATED_URDF "${GENERATED_URDF_DIR}/asr_sdm_segments_4.urdf")
file(MAKE_DIRECTORY "${GENERATED_URDF_DIR}")

xacro_add_xacro_file(
  "${CMAKE_CURRENT_SOURCE_DIR}/urdf/asr_sdm_wrapper.urdf.xacro"
  "${GENERATED_URDF}"
)
add_custom_target(generate_asr_sdm_segments_4_urdf ALL
  DEPENDS "${GENERATED_URDF}"
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  # 模型资产的上游版权信息仍需确认，暂不执行版权头检查。
  set(ament_cmake_copyright_FOUND TRUE)
  # 本包不包含 C/C++ 源码，跳过 cpplint。
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(DIRECTORY config launch meshes rviz urdf worlds
  DESTINATION share/${PROJECT_NAME}
)
install(FILES "${GENERATED_URDF}"
  DESTINATION share/${PROJECT_NAME}/urdf/generated
)
install(FILES README.md
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
