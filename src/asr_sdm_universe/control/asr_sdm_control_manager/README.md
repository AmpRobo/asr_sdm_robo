# asr_sdm_control_manager

Front-unit following kinematic controller: turns `cmd_vel` into joint states and odometry to drive the robot model.
Default node is the **3D controller** `realtime_front_unit_controller_3d` (used by this package’s launch and by `planning_simulator`).

[English](#english) · [中文](#中文)

---

<a id="english"></a>

## English

Topic names live under `topics` in `config/asr_sdm_control_manager.yaml`. Callers (e.g. `planning_simulator`) can pass another file with the same schema via `config_file`.

### Topics

#### 3D controller (default, `realtime_front_unit_controller_3d`)

Main link with teleop / planning: `/control/asr_sdm/cmd_vel`.

**Subscribe**

| Topic | Type | Role |
|---|---|---|
| `/control/asr_sdm/cmd_vel` | `geometry_msgs/msg/Twist` | Velocity command (teleop / planning) |
| `/control/initial_pose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | Reset pose (e.g. RViz 2D Pose Estimate) |

**Publish**

| Topic | Type | Role |
|---|---|---|
| `/control/asr_sdm/odom` | `nav_msgs/msg/Odometry` | Controller odometry |
| `/control/joint_states` | `sensor_msgs/msg/JointState` | Joint states for `robot_state_publisher` |
| `/control/asr_sdm/controller_state_3d` | `std_msgs/msg/Float64MultiArray` | Internal state (visualizer) |
| `/control/asr_sdm/control_cmd_3d` | `asr_sdm_control_msgs/msg/ControlCmd` | Hardware command; **off by default** (`publish_control_cmd: false`) |

YAML keys:

```yaml
topics:
  odom: /control/asr_sdm/odom
  joint_states: /control/joint_states
  initialpose: /control/initial_pose
  cmd_vel: /control/asr_sdm/cmd_vel
  controller_state: /control/asr_sdm/controller_state_3d
  control_cmd: /control/asr_sdm/control_cmd_3d
```

#### 2D controller (`realtime_front_unit_controller_2d`)

For standalone `ros2 run` debugging; not started by the default launch.

**Subscribe**

| Topic | Type | Role |
|---|---|---|
| `/control/asr_sdm/cmd_vel` | `geometry_msgs/msg/Twist` | Velocity command |

**Publish**

| Topic | Type | Role |
|---|---|---|
| `/control/asr_sdm/control_cmd` | `asr_sdm_control_msgs/msg/ControlCmd` | Hardware command |
| `/control/asr_sdm/controller_state` | `std_msgs/msg/Float64MultiArray` | Internal state (2D visualizer) |

#### Visualizers

| Node | Subscribe |
|---|---|
| `realtime_controller_visualizer_3d` | `/control/asr_sdm/controller_state_3d` |
| `realtime_controller_visualizer_2d` | `/control/asr_sdm/controller_state` |

### Build

```bash
cd ~/asr_sdm_robo
colcon build --packages-up-to asr_sdm_control_manager
source install/setup.bash
```

Offline 3D sims / visualizers need Matplot++ (and optional Python deps):

```bash
sudo apt install python3-dev python3-numpy python3-matplotlib python3-tk
./install_matplotplusplus.sh
```

### Launch

This package’s launch starts only the kinematic controller:

```bash
ros2 launch asr_sdm_control_manager asr_sdm_control_manager.launch.py
```

Optional: `config_file:=/path/to.yaml` (same schema as `config/asr_sdm_control_manager.yaml`).

With `planning_simulator`:

```bash
ros2 launch planning_simulator planning_simulator_launch.py control:=enable
# gamepad teleop
ros2 launch planning_simulator planning_simulator_launch.py control:=enable teleop:=enable
```

`planning_simulator` includes this package’s launch and passes its own `config/planning_simulator.yaml` as `config_file`.

Standalone nodes:

```bash
ros2 run asr_sdm_control_manager realtime_front_unit_controller_3d
ros2 run asr_sdm_control_manager realtime_controller_visualizer_3d
ros2 run asr_sdm_control_manager realtime_front_unit_controller_2d
ros2 run asr_sdm_control_manager realtime_controller_visualizer_2d
```

Gamepad input comes from `asr_sdm_teleop` (+ `joy`), which publishes to `/control/asr_sdm/cmd_vel`.

### Topic smoke test

Without a gamepad:

```bash
ros2 topic pub --rate 50 /control/asr_sdm/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.10, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.12, z: 0.30}}"
```

Stop:

```bash
ros2 topic pub --once /control/asr_sdm/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

### Offline simulation

```bash
ros2 run asr_sdm_control_manager front_unit_following_controller_test_2d
ros2 run asr_sdm_control_manager front_unit_following_controller_test_3d
```

### Main parameters (3D)

Override with `--ros-args -p name:=value` or edit the yaml.

| Parameter | Default |
|---|---|
| `cmd_vel_topic` | `/control/asr_sdm/cmd_vel` |
| `initialpose_topic` | `/control/initial_pose` |
| `odom_topic` | `/control/asr_sdm/odom` |
| `joint_state_topic` | `/control/joint_states` |
| `controller_state_topic` | `/control/asr_sdm/controller_state_3d` |
| `control_cmd_topic` | `/control/asr_sdm/control_cmd_3d` |
| `control_period_ms` | `20` |
| `cmd_timeout_sec` | `0.3` |
| `max_linear_velocity` | `0.12` |
| `max_pitch_rate` | `0.35` |
| `max_yaw_rate` | `0.35` |
| `publish_control_cmd` | `false` |
| `link_length` | `0.25` |
| `joint_rate_limit` | `2.0` |
| `joint_limit` | `~π/2` |
| `max_curvature` | `1.2` |
| `damping` | `0.02` |

---

<a id="中文"></a>

## 中文

前端跟随运动学控制器：将 `cmd_vel` 转为关节状态与里程计，驱动机器人模型。  
默认使用 **3D 控制节点** `realtime_front_unit_controller_3d`（本包 launch 与 `planning_simulator` 均如此）。

话题名集中在 `config/asr_sdm_control_manager.yaml` 的 `topics` 段；`planning_simulator` 可通过 `config_file` 传入同 schema 配置覆盖。

### Topics

#### 3D 控制节点（默认，`realtime_front_unit_controller_3d`）

与 teleop / planning 的主通道是 `/control/asr_sdm/cmd_vel`。

**Subscribe（输入）**

| Topic | 类型 | 用途 |
|---|---|---|
| `/control/asr_sdm/cmd_vel` | `geometry_msgs/msg/Twist` | 速度指令（teleop / planning） |
| `/control/initial_pose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | 重置位姿（如 RViz 2D Pose Estimate） |

**Publish（输出）**

| Topic | 类型 | 用途 |
|---|---|---|
| `/control/asr_sdm/odom` | `nav_msgs/msg/Odometry` | 控制器里程计 |
| `/control/joint_states` | `sensor_msgs/msg/JointState` | 关节状态（供给 `robot_state_publisher`） |
| `/control/asr_sdm/controller_state_3d` | `std_msgs/msg/Float64MultiArray` | 内部状态（可视化） |
| `/control/asr_sdm/control_cmd_3d` | `asr_sdm_control_msgs/msg/ControlCmd` | 硬件指令；**默认不发布**（`publish_control_cmd: false`） |

对应 yaml 键：

```yaml
topics:
  odom: /control/asr_sdm/odom
  joint_states: /control/joint_states
  initialpose: /control/initial_pose
  cmd_vel: /control/asr_sdm/cmd_vel
  controller_state: /control/asr_sdm/controller_state_3d
  control_cmd: /control/asr_sdm/control_cmd_3d
```

#### 2D 控制节点（`realtime_front_unit_controller_2d`）

单独 `ros2 run` 调试时使用，不由默认 launch 启动。

**Subscribe**

| Topic | 类型 | 用途 |
|---|---|---|
| `/control/asr_sdm/cmd_vel` | `geometry_msgs/msg/Twist` | 速度指令 |

**Publish**

| Topic | 类型 | 用途 |
|---|---|---|
| `/control/asr_sdm/control_cmd` | `asr_sdm_control_msgs/msg/ControlCmd` | 硬件控制指令 |
| `/control/asr_sdm/controller_state` | `std_msgs/msg/Float64MultiArray` | 内部状态（2D 可视化） |

#### 可视化节点

| 节点 | Subscribe |
|---|---|
| `realtime_controller_visualizer_3d` | `/control/asr_sdm/controller_state_3d` |
| `realtime_controller_visualizer_2d` | `/control/asr_sdm/controller_state` |

### 编译

```bash
cd ~/asr_sdm_robo
colcon build --packages-up-to asr_sdm_control_manager
source install/setup.bash
```

3D 离线仿真与可视化需要 Matplot++（及可选 Python 依赖）：

```bash
sudo apt install python3-dev python3-numpy python3-matplotlib python3-tk
./install_matplotplusplus.sh
```

### 启动

本包 launch 只起运动学控制器：

```bash
ros2 launch asr_sdm_control_manager asr_sdm_control_manager.launch.py
```

可选：`config_file:=/path/to.yaml`（与 `config/asr_sdm_control_manager.yaml` 同 schema）。

与 `planning_simulator` 一起：

```bash
ros2 launch planning_simulator planning_simulator_launch.py control:=enable
# 手柄遥控
ros2 launch planning_simulator planning_simulator_launch.py control:=enable teleop:=enable
```

`planning_simulator` 会 include 本包 launch，并用自己的 `config/planning_simulator.yaml` 作为 `config_file`。

手动 `ros2 run`：

```bash
ros2 run asr_sdm_control_manager realtime_front_unit_controller_3d
ros2 run asr_sdm_control_manager realtime_controller_visualizer_3d
ros2 run asr_sdm_control_manager realtime_front_unit_controller_2d
ros2 run asr_sdm_control_manager realtime_controller_visualizer_2d
```

手柄链路由 `asr_sdm_teleop`（及 `joy`）提供，发布到 `/control/asr_sdm/cmd_vel`。

### Topic 输入测试

无手柄时直接发速度：

```bash
ros2 topic pub --rate 50 /control/asr_sdm/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.10, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.12, z: 0.30}}"
```

停止：

```bash
ros2 topic pub --once /control/asr_sdm/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

### 离线仿真

```bash
ros2 run asr_sdm_control_manager front_unit_following_controller_test_2d
ros2 run asr_sdm_control_manager front_unit_following_controller_test_3d
```

### 主要参数（3D）

覆盖方式：`--ros-args -p 参数名:=参数值`，或改 yaml。

| 参数 | 默认 |
|---|---|
| `cmd_vel_topic` | `/control/asr_sdm/cmd_vel` |
| `initialpose_topic` | `/control/initial_pose` |
| `odom_topic` | `/control/asr_sdm/odom` |
| `joint_state_topic` | `/control/joint_states` |
| `controller_state_topic` | `/control/asr_sdm/controller_state_3d` |
| `control_cmd_topic` | `/control/asr_sdm/control_cmd_3d` |
| `control_period_ms` | `20` |
| `cmd_timeout_sec` | `0.3` |
| `max_linear_velocity` | `0.12` |
| `max_pitch_rate` | `0.35` |
| `max_yaw_rate` | `0.35` |
| `publish_control_cmd` | `false` |
| `link_length` | `0.25` |
| `joint_rate_limit` | `2.0` |
| `joint_limit` | `约 π/2` |
| `max_curvature` | `1.2` |
| `damping` | `0.02` |
