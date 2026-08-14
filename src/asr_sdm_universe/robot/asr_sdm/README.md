# ASR-SDM Robot Model

This directory contains the ROS 2 model, meshes, RViz configuration, and Gazebo Sim launch files for the ASR-SDM screw-propelled snake robot.

## Current Model Parameters

- ROS 2 package name: `asr_sdm`
- Robot name: `asr_sdm`
- Number of body links: `4`
  - `screwdrive_segment_0`
  - `screwdrive_segment_1`
  - `screwdrive_segment_2`
  - `screwdrive_segment_3`
- Number of inter-segment joint groups: `3`
  - One A/Cross/B joint group follows each of segments 0, 1, and 2
  - Each group contains two revolute joints
  - Adjacent body segments are connected by one fixed joint
- Number of screw rotors: `8`
  - Each body link has a left and right rotor connected by continuous joints
- Complete URDF structure:
  - Total links: `23`
  - Total joints: `22`
  - Fixed joints: `8`
  - Revolute joints: `6`
  - Continuous joints: `8`
- Root link: `base` (odometry / IMU frame; `base_link` heading along +X aligns with the camera frustum +Z)
- RViz fixed frame: `world`
- Model entry point: `urdf/asr_sdm_wrapper.urdf.xacro`
- Generated plain URDF: `urdf/generated/asr_sdm_segments_4.urdf`

The model is fixed at four body links and three inter-segment joint groups. The number of segments cannot be changed at build time or runtime.

## Build

Run the following commands from the workspace root:

```bash
cd /home/wwlwwl/asr_sdm_robo
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select asr_sdm
source install/setup.bash
```

## Start Robot Description

```bash
ros2 launch asr_sdm asr_sdm_description.launch.py
```

Parameters come from `config/robot_model.yaml`. Pass another file with the same schema via `config_file:=<path>`.

## Display the Model in RViz

Start RViz with the Joint State Publisher GUI:

```bash
ros2 launch asr_sdm display.launch.py
```

Start RViz without the Joint State Publisher GUI:

```bash
ros2 launch asr_sdm display.launch.py gui:=false
```

## Start Gazebo Sim

The `ros_gz_sim` and `ros_gz_bridge` packages must be installed first.

Start the empty world included in this package:

```bash
ros2 launch asr_sdm gazebo.launch.py \
  use_custom_world:=true \
  custom_world:=empty.sdf
```

Start the cars-and-trees world included in this package:

```bash
ros2 launch asr_sdm gazebo.launch.py \
  use_custom_world:=true \
  custom_world:=cars_and_trees.sdf
```

`cars_and_trees.sdf` uses online models from Gazebo Fuel and therefore requires a working network connection.

Start an external world:

```bash
ros2 launch asr_sdm gazebo.launch.py \
  use_custom_world:=false \
  gazebo_world:=/absolute/path/to/world.sdf
```

Start Gazebo Sim together with RViz:

```bash
ros2 launch asr_sdm gazebo.launch.py use_rviz:=true
```

## Launch Arguments

### `asr_sdm_description.launch.py`

- `config_file`: Absolute path to a YAML file with the `robot_model.yaml` schema; default: `config/robot_model.yaml`

### `display.launch.py`

- `model`: Path to a URDF or Xacro model file
- `use_sim_time`: Whether to use simulation time; default: `false`
- `gui`: Whether to start the Joint State Publisher GUI; default: `true`
- `rvizconfig`: Path to the RViz configuration file

### `gazebo.launch.py`

- `model`: Path to a URDF or Xacro model file
- `use_sim_time`: Whether to use Gazebo simulation time; default: `true`
- `use_rviz`: Whether to start RViz together with Gazebo Sim; default: `false`
- `use_custom_world`: Whether to load a world from this package; default: `true`
- `custom_world`: SDF filename under this package's `worlds/` directory; default: `empty.sdf`
- `gazebo_world`: External world name or path

## Known Model Limitations

- The `joint_unit_a_*`, `joint_unit_cross_*`, and `joint_unit_b_*` links currently have no inertial or collision elements.
- The root link contains an inertial element. The Robot State Publisher KDL parser reports a root-link inertia warning, but this does not prevent the model from being displayed in RViz.
- The current model does not include `ros2_control`, transmissions, or actuator plugins.
