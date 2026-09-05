# GRAMPC (ROS 2 package)

GRAMPC is a nonlinear MPC framework that is suitable for dynamical systems with sampling times in the (sub)millisecond range and that allows for an efficient implementation on embedded hardware. The algorithm is based on an augmented Lagrangian formulation with a tailored gradient method for the inner minimization problem.

This package bundles [GRAMPC](https://github.com/grampc/grampc) and [GRAMPC-S](https://github.com/grampc/grampc-s), the stochastic MPC extension, into a single `ament_cmake` package. The MATLAB/Simulink and Python interfaces and the plain Makefile builds have been removed; see the upstream repositories for those.

## Library targets

The package exports two libraries:

| Target | Contents |
| --- | --- |
| `grampc::grampc` | the plain C solver core |
| `grampc::grampc_s` | GRAMPC-S: stochastic MPC, C++ interface using Eigen |

`grampc_s` is the only C++ frontend: it provides `grampc::Grampc`, `grampc::ProblemDescription` and the `probfct` callbacks (`ffct`, `lfct`, ...). The raw-pointer C++ interface that upstream GRAMPC ships under `cpp/` has been dropped, because it defines the same symbols and could therefore never be linked alongside GRAMPC-S.

## Building

Place the package in the `src` directory of a ROS 2 workspace and build it:

```bash
colcon build --packages-select grampc
```

The GRAMPC-S examples are built by default; pass `-DGRAMPC_BUILD_EXAMPLES=OFF` via `--cmake-args` to skip them.

## Using the library

Add the dependency to your `package.xml`:

```xml
<depend>grampc</depend>
```

and link against the exported target in your `CMakeLists.txt`:

```cmake
find_package(grampc REQUIRED)
target_link_libraries(my_node grampc::grampc_s)
```

Include the umbrella header, derive from `grampc::ProblemDescription` and hand the problem to `grampc::Grampc`. Working examples for nine systems are in [examples/](examples).

```cpp
#include <grampc_s/grampc_s.hpp>

class MyProblem : public grampc::ProblemDescription
{
public:
  MyProblem() : ProblemDescription(/*Nx*/ 2, /*Nu*/ 1, /*Np*/ 0,
                                   /*Ng*/ 0, /*Nh*/ 0, /*NgT*/ 0, /*NhT*/ 0) {}

  void ffct(VectorRef out, ctypeRNum t, VectorConstRef x, VectorConstRef u,
            VectorConstRef p, const grampc::GrampcParam& param) override;
  void dfdx_vec(VectorRef out, ctypeRNum t, VectorConstRef x, VectorConstRef u,
                VectorConstRef p, VectorConstRef vec,
                const grampc::GrampcParam& param) override;
  // cost terms and constraints as needed
};
```

Note the argument order of the `dfd*_vec` family: `(out, t, x, u, p, vec, param)`, as changed in GRAMPC 2.3.

The Eigen-based `grampc::ProblemDescription` lives in `<grampc/eigen/problem_description.hpp>`; upstream keeps it under `python/include` because it is shared with the Python bindings.

The plain C API remains available through `<grampc/grampc.h>`.

## Documentation

The structure and usage of GRAMPC are described in the [online documentation](https://grampc.github.io/grampc). More details about the algorithm and its performance are available in the corresponding open access article: https://doi.org/10.1007/s11081-018-9417-2.

Please cite the paper when you are using results obtained with GRAMPC.

## License

BSD-3-Clause, see [LICENSE.txt](LICENSE.txt) and [LICENSE-grampc-s.txt](LICENSE-grampc-s.txt).
