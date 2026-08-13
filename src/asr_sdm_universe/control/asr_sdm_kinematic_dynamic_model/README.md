# ASR-SDM kinematic and dynamic model

This package provides the URDF-free Pinocchio model used by the ASR-SDM
control manager. `asr_sdm_kinematic_model` constructs the model directly
from the configured link and joint parameters and exposes configuration,
kinematics, Jacobian, integration, mass-matrix, and gravity helpers.

The package also retains `pinocchio_dynamics_node` for the existing URDF-based
dynamics utility. The control manager uses the URDF-free model library.

## Pinocchio installation

The supported binary setup is Ubuntu 24.04 amd64 with ROS 2 Jazzy. This
workspace installs Pinocchio, coal, and eigenpy from robotpkg under
`/opt/openrobots`.

From the workspace root:

```bash
cd ~/asr_sdm_robo
./setup-dev-env.sh --pinocchio
source /opt/ros/jazzy/setup.bash
source ~/.bashrc
```

If ROS 2 Jazzy is not installed yet, install both environments with:

```bash
./setup-dev-env.sh --ros --pinocchio
```

Install the remaining workspace dependencies without replacing the robotpkg
Pinocchio installation:

```bash
rosdep install \
  --from-paths src \
  --ignore-src \
  -r -y \
  --skip-keys pinocchio
```

Verify that the installation is visible:

```bash
pkg-config --modversion pinocchio
python3 -c "import pinocchio; print(pinocchio.__version__)"
```

Do not mix the robotpkg Pinocchio/coal libraries in `/opt/openrobots` with
`ros-jazzy-pinocchio` and `ros-jazzy-coal` in the same build environment.

Older Pinocchio releases may print messages asking users to update `hpp-fcl`
to `coal`. This package does not include hpp-fcl directly; the message comes
from the compatibility layer used by the installed Pinocchio release. Do not
install a second hpp-fcl stack only to suppress that message.

For an advanced source installation, use the repository-level
`install_pinocchio_from_source.py`. The script discovers the newest stable
compatible Pinocchio, eigenpy, and Coal releases at runtime instead of following
`devel` or fixing version tags. Preview the selected tags and commits without
installing anything with:

```bash
python3 install_pinocchio_from_source.py --resolve-only
```

Coal is selected and built automatically. Collision support remains disabled by
default and can be enabled with `PINOCCHIO_BUILD_COLLISION=ON`. All three
projects build with half of the detected logical CPUs (at least one job). See
[`docs/pinocchio_installation.md`](../../../../docs/pinocchio_installation.md)
for source-directory, ref, and install-prefix overrides.

## Model verification

The model-check executable is built only when `BUILD_TESTING` is enabled and
has a test dependency on `asr_sdm_head_following_control`.

```bash
cd ~/asr_sdm_robo
source /opt/ros/jazzy/setup.bash
source ~/.bashrc

colcon build \
  --packages-up-to asr_sdm_kinematic_dynamic_model \
  --parallel-workers 1 \
  --cmake-args -DBUILD_TESTING=ON

source install/setup.bash
ros2 run asr_sdm_kinematic_dynamic_model asr_sdm_kinematic_model_check
```
