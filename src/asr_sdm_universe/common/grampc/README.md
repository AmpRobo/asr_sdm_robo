# GRAMPC (ROS 2 package)

GRAMPC is a nonlinear MPC framework that is suitable for dynamical systems with sampling times in the (sub)millisecond range and that allows for an efficient implementation on embedded hardware. The algorithm is based on an augmented Lagrangian formulation with a tailored gradient method for the inner minimization problem.

This repository packages the plain C solver core together with its C++ interface as a single `ament_cmake` ROS 2 library target. The MATLAB/Simulink and Python interfaces, the standalone examples and the plain Makefile build have been removed; see the [upstream repository](https://github.com/grampc/grampc) for those.

## Building

Place the package in the `src` directory of a ROS 2 workspace and build it:

```bash
colcon build --packages-select grampc
```

## Using the library

Add the dependency to your `package.xml`:

```xml
<depend>grampc</depend>
```

and link against the exported target in your `CMakeLists.txt`:

```cmake
find_package(grampc REQUIRED)
target_link_libraries(my_node grampc::grampc)
```

Headers are installed under the `grampc/` prefix. Derive from `grampc::ProblemDescription` to define an optimal control problem and hand it to `grampc::Grampc`:

```cpp
#include <grampc/grampc.hpp>

class MyProblem : public grampc::ProblemDescription
{
  void ocp_dim(typeInt *Nx, typeInt *Nu, typeInt *Np, typeInt *Ng,
               typeInt *Nh, typeInt *NgT, typeInt *NhT) override;
  void ffct(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u,
            ctypeRNum *p, const typeGRAMPCparam *param) override;
  void dfdx_vec(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u,
                ctypeRNum *p, ctypeRNum *vec, const typeGRAMPCparam *param) override;
  // cost terms and constraints as needed
};

MyProblem problem;
grampc::Grampc solver(&problem);
solver.setparam_real("Thor", 1.0);
solver.run();
```

The plain C API remains available through `<grampc/grampc.h>`.

## Documentation

The structure and usage of GRAMPC are described in the [online documentation](https://grampc.github.io/grampc). More details about the algorithm and its performance are available in the corresponding open access article: https://doi.org/10.1007/s11081-018-9417-2.

Please cite the paper when you are using results obtained with GRAMPC.

## License

BSD-3-Clause, see [LICENSE.txt](LICENSE.txt).
