# This file is part of GRAMPC-S - (https://github.com/grampc/grampc-s)
#
# GRAMPC-S -- A software framework for stochastic model predictive control (SMPC)
#
# Copyright 2024 Daniel Landgraf, Andreas Voelz, Knut Graichen.
# All rights reserved.
#
# GRAMPC-S is distributed under the BSD-3-Clause license, see LICENSE.txt

set(grampc_s_examples
  reactor
  double_integrator
  double_integrator_GP
  inverted_pendulum
  vehicle
  PMSM
  crane_2D
  mass_spring_damper
  nonlinear_chain
)

foreach(example IN LISTS grampc_s_examples)
  add_subdirectory(${example})
  get_property(example_targets DIRECTORY ${example} PROPERTY BUILDSYSTEM_TARGETS)
  install(TARGETS ${example_targets} DESTINATION lib/${PROJECT_NAME})
endforeach()
