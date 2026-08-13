# ASR-SDM control packages

This directory contains the control-side packages used by the ASR-SDM robot and
its simulators.

## Packages

| Package | Responsibility | Main products |
|---|---|---|
| [`asr_sdm_head_following_control`](asr_sdm_head_following_control/) | ROS-independent 2D and 3D head-following algorithms | `front_unit_following_controller_2d_core`, `front_unit_following_controller_3d_core` |
| [`asr_sdm_kinematic_dynamic_model`](asr_sdm_kinematic_dynamic_model/) | Pinocchio kinematic/dynamic model and the retained legacy URDF dynamics utility | `asr_sdm_kinematic_model`, `pinocchio_dynamics_node` |
| [`asr_sdm_control_manager`](asr_sdm_control_manager/) | ROS 2 topics, parameters, command handling, state integration, and robot-state publication | `asr_sdm_control_manager` |

Production dependencies are one-way:

```text
asr_sdm_head_following_control -----------+
                                           +--> asr_sdm_control_manager
asr_sdm_kinematic_dynamic_model ----------+
```

The runtime control path is:

```text
cmd_vel
  -> asr_sdm_control_manager
  -> head-following controller + Pinocchio model
  -> joint_states + odometry + optional hardware command
```

`planning_simulator` includes the control-manager launch file and uses the 3D
manager as the supported ROS control entry point.

## Build

```bash
cd ~/asr_sdm_robo
source /opt/ros/jazzy/setup.bash
colcon build \
  --packages-up-to asr_sdm_control_manager \
  --symlink-install \
  --parallel-workers 1
source install/setup.bash
```

See the model package README for the Pinocchio installation procedure. The
controller plotting demos are optional and are disabled in a normal library
build.
