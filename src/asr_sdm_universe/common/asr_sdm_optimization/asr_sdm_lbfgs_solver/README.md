# asr_sdm_lbfgs_solver

## Package layout
```sh
asr_sdm_lbfgs_solver/
├── CMakeLists.txt          # shared lib + optional example test
├── package.xml
├── include/asr_sdm_lbfgs_solver/lbfgs.hpp
├── src/lbfgs.cpp
└── test/lbfgs_test.cpp     # Rosenbrock demo (installed as executable)
```

## Build & run
```sh
colcon build --packages-select asr_sdm_lbfgs_solver
source install/setup.bash
ros2 run asr_sdm_lbfgs_solver lbfgs_test
```

## Use from another package
package.xml:
```sh
<depend>asr_sdm_lbfgs_solver</depend>
```

CMakeLists.txt:
```sh
find_package(asr_sdm_lbfgs_solver REQUIRED)
target_link_libraries(your_target asr_sdm_lbfgs_solver::asr_sdm_lbfgs_solver)
```