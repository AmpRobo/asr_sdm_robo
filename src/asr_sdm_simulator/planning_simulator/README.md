# planning_simulator

Planning simulation stack: map generation, dynamics simulation, RViz, plus optional robot model, kinematic control, gamepad teleop, and planning.

[English](#english) · [中文](#中文)

---

<a id="english"></a>

## English

### Build

From the workspace root:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-up-to planning_simulator
source install/setup.bash
```

### Launch

```bash
ros2 launch planning_simulator planning_simulator_launch.py
```

By default this starts the simulator, random map, RViz, the `asr_sdm` robot model, and the kinematic controller. The model appears at `(-5, 0, 0)`. Gamepad teleop and planning are off.

List all launch arguments:

```bash
ros2 launch planning_simulator planning_simulator_launch.py --show-args
```

### Launch arguments

| Argument | Values | Default | Role |
|---|---|---|---|
| `robot_model` | installed model package name | `asr_sdm` | Selects the robot model package; must provide `launch/<name>_description.launch.py` |
| `control` | `enable` / `disable` | `enable` | Starts `asr_sdm_control_manager` (kinematic controller) |
| `teleop` | `enable` / `disable` | `disable` | Starts `asr_sdm_teleop` (joy driver + teleop node) |
| `planning` | `enable` / `disable` | `disable` | Starts `asr_sdm_planning_manager` (topological replanning) |

Common combinations:

```bash
# Default: sim + model + controller (drive with cmd_vel)
ros2 launch planning_simulator planning_simulator_launch.py

# Add gamepad teleop
ros2 launch planning_simulator planning_simulator_launch.py teleop:=enable

# Add planning
ros2 launch planning_simulator planning_simulator_launch.py planning:=enable

# Everything on
ros2 launch planning_simulator planning_simulator_launch.py \
  robot_model:=asr_sdm control:=enable teleop:=enable planning:=enable

# Sim only, no controller (model will not move; may be invisible in RViz if Fixed Frame is world)
ros2 launch planning_simulator planning_simulator_launch.py control:=disable
```

### Parameters and config

Node parameters and topic names live in:

```text
config/planning_simulator.yaml
```

The `robot_model` and `control` optional stacks receive the same file via `config_file` and read their sections from it:

- Model: `features.use_asr_sdm_model`, `topics`, `robot_model`
- Controller: `features.use_kinematic_controller`, `topics`, `kinematic_controller`

To change initial pose, topic names, or controller gains, edit that YAML and restart the launch (no rebuild needed; this package uses symlink install).

### Topics

| Topic | Description |
|---|---|
| `/control/asr_sdm/cmd_vel` | Input from teleop / planning; drives the kinematic controller |
| `/control/asr_sdm/odom` | Odometry published by the controller |
| `/control/joint_states` | Joint states for `robot_state_publisher` |
| `/control/initial_pose` | Reset controller pose (RViz 2D Pose Estimate) |
| `/visual_slam/odom` | Simulator dynamics odometry |
| `/simulator/planning_simulator/add_static_obstacle` | Click-to-add pillar obstacles (RViz Static Obstacle tool) |

Without a gamepad, publish Twist directly to exercise the controller:

```bash
ros2 topic pub --rate 20 /control/asr_sdm/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.10, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.20}}"
```

### Package conventions

Optional stacks are included by name:

```text
<package>/launch/<package>.launch.py
```

Robot models use:

```text
<robot_model>/launch/<robot_model>_description.launch.py
```

Example: default `asr_sdm` → `asr_sdm/launch/asr_sdm_description.launch.py`.

Responsibilities:

| Package / launch | Responsibility |
|---|---|
| `planning_simulator` | Simulation, map, RViz, top-level assemble |
| `asr_sdm` / `asr_sdm_description.launch.py` | URDF and static TF |
| `asr_sdm_control_manager` | Kinematic controller |
| `asr_sdm_teleop` | `joy` + teleop |
| `asr_sdm_planning_manager` | Planning and trajectory server |

### Standalone launches (debug)

```bash
ros2 launch asr_sdm asr_sdm_description.launch.py
ros2 launch asr_sdm_control_manager asr_sdm_control_manager.launch.py
ros2 launch asr_sdm_teleop asr_sdm_teleop.launch.py
ros2 launch asr_sdm_planning_manager asr_sdm_planning_manager.launch.py
```

The model launch alone does not publish a world-frame pose. The controller’s odom only becomes `world → base` when this package’s `odom_visualization` (`tf45`) is running. For normal use, start `planning_simulator_launch.py`.

---

<a id="中文"></a>

## 中文

### 编译

在工作空间根目录：

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-up-to planning_simulator
source install/setup.bash
```

### 启动

```bash
ros2 launch planning_simulator planning_simulator_launch.py
```

默认会启动仿真、随机地图、RViz，并加载 `asr_sdm` 机器人模型与运动学控制器。模型会显示在 `(-5, 0, 0)`，手柄遥控和规划模块默认关闭。

查看全部启动参数：

```bash
ros2 launch planning_simulator planning_simulator_launch.py --show-args
```

### 启动参数

| 参数 | 可选值 | 默认 | 作用 |
|---|---|---|---|
| `robot_model` | 已安装的模型包名 | `asr_sdm` | 选择机器人模型包，需提供 `launch/<包名>_description.launch.py` |
| `control` | `enable` / `disable` | `enable` | 启动 `asr_sdm_control_manager`（运动学控制器） |
| `teleop` | `enable` / `disable` | `disable` | 启动 `asr_sdm_teleop`（手柄驱动 + teleop 节点） |
| `planning` | `enable` / `disable` | `disable` | 启动 `asr_sdm_planning_manager`（拓扑重规划） |

常用组合：

```bash
# 默认：仿真 + 模型 + 控制器（可用 cmd_vel 驱动）
ros2 launch planning_simulator planning_simulator_launch.py

# 加手柄遥控
ros2 launch planning_simulator planning_simulator_launch.py teleop:=enable

# 加规划
ros2 launch planning_simulator planning_simulator_launch.py planning:=enable

# 全开
ros2 launch planning_simulator planning_simulator_launch.py \
  robot_model:=asr_sdm control:=enable teleop:=enable planning:=enable

# 只看仿真，不启动控制器（模型不会动，RViz Fixed Frame 为 world 时可能看不见模型）
ros2 launch planning_simulator planning_simulator_launch.py control:=disable
```

### 参数与配置

节点参数和话题名集中在：

```text
config/planning_simulator.yaml
```

`robot_model`、`control` 两个可选栈通过 `config_file` 读取同一份配置里的对应段落：

- 模型：`features.use_asr_sdm_model`、`topics`、`robot_model`
- 控制器：`features.use_kinematic_controller`、`topics`、`kinematic_controller`

改初始位姿、话题名、控制器增益等，优先改这份 yaml，然后重新启动 launch（不必单独重编，本包用 symlink install）。

### 相关话题

| 话题 | 说明 |
|---|---|
| `/control/asr_sdm/cmd_vel` | 手柄 / 规划侧输入，驱动运动学控制器 |
| `/control/asr_sdm/odom` | 控制器发布的里程计 |
| `/control/joint_states` | 控制器发布的关节状态，供给 `robot_state_publisher` |
| `/control/initial_pose` | 重置控制器位姿（RViz 2D Pose Estimate） |
| `/visual_slam/odom` | 仿真器动力学里程计 |
| `/simulator/planning_simulator/add_static_obstacle` | RViz Static Obstacle 工具点击加点柱障碍 |

无手柄时可用 topic 直接发速度测试控制器：

```bash
ros2 topic pub --rate 20 /control/asr_sdm/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.10, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.20}}"
```

### 模块约定

可选栈按「包名等于目录名」约定被 include：

```text
<package>/launch/<package>.launch.py
```

机器人模型额外约定：

```text
<robot_model>/launch/<robot_model>_description.launch.py
```

例如当前默认模型是 `asr_sdm` → `asr_sdm/launch/asr_sdm_description.launch.py`。

责任划分：

| 包 / launch | 负责 |
|---|---|
| `planning_simulator` | 仿真、地图、RViz、一键拼装 |
| `asr_sdm` / `asr_sdm_description.launch.py` | URDF 与静态 TF |
| `asr_sdm_control_manager` | 运动学控制器 |
| `asr_sdm_teleop` | `joy` + teleop |
| `asr_sdm_planning_manager` | 规划与轨迹服务 |

### 独立启动（调试用）

```bash
ros2 launch asr_sdm asr_sdm_description.launch.py
ros2 launch asr_sdm_control_manager asr_sdm_control_manager.launch.py
ros2 launch asr_sdm_teleop asr_sdm_teleop.launch.py
ros2 launch asr_sdm_planning_manager asr_sdm_planning_manager.launch.py
```

模型独立启动时没有世界坐标系下的位姿；控制器提供的 odom 需要配合本包的 `odom_visualization`（`tf45`）才会出现 `world → base`。正常使用请直接启动本包的 `planning_simulator_launch.py`。
