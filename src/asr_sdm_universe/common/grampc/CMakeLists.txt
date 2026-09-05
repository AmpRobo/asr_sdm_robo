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
find_package(Eigen3 REQUIRED NO_MODULE)

option(GRAMPC_BUILD_EXAMPLES "Build the GRAMPC-S example executables" ON)

# ---------------------------------------------------------------------------
# grampc: the plain C solver core
# ---------------------------------------------------------------------------
add_library(grampc SHARED
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
)
add_library(grampc::grampc ALIAS grampc)

target_include_directories(grampc PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

find_library(MATH_LIBRARY m)
if(MATH_LIBRARY)
  target_link_libraries(grampc PUBLIC ${MATH_LIBRARY})
endif()

# ---------------------------------------------------------------------------
# grampc_s: stochastic MPC, Eigen-based C++ interface.
#
# Provides grampc::Grampc, grampc::ProblemDescription and the probfct callbacks
# (ffct, lfct, ...) on top of the C core.
# ---------------------------------------------------------------------------
add_library(grampc_s SHARED
  src/grampc_s/constraint_approx/chebyshev_constraint_approximation.cpp
  src/grampc_s/constraint_approx/Gaussian_constraint_approximation.cpp
  src/grampc_s/constraint_approx/symmetric_constraint_approximation.cpp
  src/grampc_s/distribution/distribution.cpp
  src/grampc_s/distribution/Gaussian_distribution.cpp
  src/grampc_s/distribution/multivariate_uncorrelated_distribution.cpp
  src/grampc_s/distribution/univariate_beta_distribution.cpp
  src/grampc_s/distribution/univariate_chi_squared_distribution.cpp
  src/grampc_s/distribution/univariate_exponential_distribution.cpp
  src/grampc_s/distribution/univariate_extreme_value_distribution.cpp
  src/grampc_s/distribution/univariate_f_distribution.cpp
  src/grampc_s/distribution/univariate_gamma_distribution.cpp
  src/grampc_s/distribution/univariate_lognormal_distribution.cpp
  src/grampc_s/distribution/univariate_piecewise_constant_distribution.cpp
  src/grampc_s/distribution/univariate_student-t_distribution.cpp
  src/grampc_s/distribution/univariate_uniform_distribution.cpp
  src/grampc_s/distribution/univariate_weibull_distribution.cpp
  src/grampc_s/gaussian_process/gaussian_process.cpp
  src/grampc_s/gaussian_process/kernel_product.cpp
  src/grampc_s/gaussian_process/kernel_sum.cpp
  src/grampc_s/gaussian_process/locally_periodic_kernel.cpp
  src/grampc_s/gaussian_process/Matern32_kernel.cpp
  src/grampc_s/gaussian_process/Matern52_kernel.cpp
  src/grampc_s/gaussian_process/periodic_kernel.cpp
  src/grampc_s/gaussian_process/squared_exponential_kernel.cpp
  src/grampc_s/grampc_interface/grampc_interface.cpp
  src/grampc_s/point_transformation/composed_quadrature.cpp
  src/grampc_s/point_transformation/Hermite_quadrature.cpp
  src/grampc_s/point_transformation/Legendre_quadrature.cpp
  src/grampc_s/point_transformation/monte_carlo.cpp
  src/grampc_s/point_transformation/PCE_transformation.cpp
  src/grampc_s/point_transformation/stirling_interpolation_first_order.cpp
  src/grampc_s/point_transformation/stirling_interpolation_second_order.cpp
  src/grampc_s/point_transformation/unscented_transformation.cpp
  src/grampc_s/polynomial/Hermite_polynomial_generator.cpp
  src/grampc_s/polynomial/Legendre_polynomial_generator.cpp
  src/grampc_s/polynomial/multivariate_polynomial.cpp
  src/grampc_s/polynomial/polynomial.cpp
  src/grampc_s/problem_description/monte_carlo_problem_description.cpp
  src/grampc_s/problem_description/problem_description.cpp
  src/grampc_s/problem_description/resampling_GP_problem_description.cpp
  src/grampc_s/problem_description/resampling_problem_description.cpp
  src/grampc_s/problem_description/sigma_point_problem_description.cpp
  src/grampc_s/problem_description/taylor_problem_description.cpp
  src/grampc_s/simulator/simulator.cpp
  src/grampc_s/util/grampc_s_util.cpp
)
add_library(grampc::grampc_s ALIAS grampc_s)
target_link_libraries(grampc_s PUBLIC grampc Eigen3::Eigen)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(
  TARGETS grampc grampc_s
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include/${PROJECT_NAME}
)

if(GRAMPC_BUILD_EXAMPLES)
  add_subdirectory(examples)
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include/${PROJECT_NAME})
ament_export_dependencies(Eigen3)

ament_package()
