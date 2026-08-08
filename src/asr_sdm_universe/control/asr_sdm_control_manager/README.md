# asr_sdm_controller

`asr_sdm_controller` 提供 2D/3D 前端跟随控制器、离线仿真、实时控制节点和可视化节点。

## 依赖安装

```bash
sudo apt install python3-dev python3-numpy python3-matplotlib python3-tk
sudo apt install ros-${ROS_DISTRO}-joy
```

3D 离线仿真和 3D 可视化需要 Matplot++：

```bash
cd ~/asr_sdm_robo
./install_matplotplusplus.sh
```

## 编译

```bash
cd ~/asr_sdm_robo
colcon build --packages-up-to asr_sdm_controller asr_sdm_teleop
source install/setup.bash
```

如果 Matplot++ 没有安装到系统路径，可以指定源码目录：

```bash
colcon build --packages-up-to asr_sdm_controller asr_sdm_teleop --cmake-args -DMATPLOTPP_SOURCE_DIR=/path/to/matplotplusplus
```

## 离线仿真

运行 2D 离线仿真：

```bash
ros2 run asr_sdm_controller front_unit_following_controller_test_2d
```

运行 3D 离线仿真：

```bash
ros2 run asr_sdm_controller front_unit_following_controller_test_3d
```

## 2D 实时控制

分别打开 4 个终端，并确保都已执行：

```bash
source ~/asr_sdm_robo/install/setup.bash
```

终端 1：启动手柄节点。

```bash
ros2 run joy joy_node
```

终端 2：启动 teleop 节点。

```bash
ros2 run asr_sdm_teleop asr_sdm_teleop_node
```

终端 3：启动 2D 控制节点。

```bash
ros2 run asr_sdm_controller realtime_front_unit_controller_2d
```

终端 4：启动 2D 可视化节点。

```bash
ros2 run asr_sdm_controller realtime_controller_visualizer_2d
```

## 3D 实时控制

分别打开 4 个终端，并确保都已执行：

```bash
source ~/asr_sdm_robo/install/setup.bash
```

终端 1：启动手柄节点。

```bash
ros2 run joy joy_node
```

终端 2：启动 teleop 节点。

```bash
ros2 run asr_sdm_teleop asr_sdm_teleop_node
```

终端 3：启动 3D 控制节点。

```bash
ros2 run asr_sdm_controller realtime_front_unit_controller_3d
```

终端 4：启动 3D 可视化节点。

```bash
ros2 run asr_sdm_controller realtime_controller_visualizer_3d
```

## Topic 输入测试

不使用手柄时，可以直接向 `/asr_sdm/cmd_vel` 发布命令测试 3D 控制节点：

例如
```bash
ros2 topic pub --rate 50 /asr_sdm/cmd_vel geometry_msgs/msg/Twist \
"{linear: {x: 0.10, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.12, z: 0.30}}"
```

停止输入：

```bash
ros2 topic pub --once /asr_sdm/cmd_vel geometry_msgs/msg/Twist \
"{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

## 默认参数

如需覆盖默认参数，可在启动命令后追加：

```bash
--ros-args -p 参数名:=参数值
```

### teleop 节点

| 参数 | 默认值 |
|---|---:|
| `linear_axis` | `1` |
| `angular_axis` | `0` |
| `pitch_axis` | `4` |
| `linear_scale` | `0.12` |
| `angular_scale` | `0.5` |
| `pitch_scale` | `0.5` |
| `use_trigger_linear` | `true` |
| `axis_l2` | `2` |
| `axis_r2` | `5` |
| `l2_released_value` | `1.0` |
| `r2_released_value` | `1.0` |
| `trigger_pressed_value` | `-1.0` |
| `deadzone` | `0.08` |
| `invert_linear` | `false` |
| `invert_pitch` | `true` |
| `invert_yaw` | `false` |

### 2D 控制节点

| 参数 | 默认值 |
|---|---:|
| `cmd_vel_topic` | `/asr_sdm/cmd_vel` |
| `control_cmd_topic` | `/asr_sdm/control_cmd` |
| `controller_state_topic` | `/asr_sdm/controller_state` |
| `control_period_ms` | `20` |
| `phi_dot_limit` | `2.0` |
| `phi_limit` | `0.85 * pi` |
| `cmd_timeout_sec` | `0.3` |
| `screw_velocity_scale` | `1.0` |
| `joint_angle_scale` | `1.0` |
| `screw_velocity_limit` | `2147483647` |
| `joint_angle_limit` | `2147483647` |
| `link_length` | `0.25` |
| `screw_radius` | `0.075` |

### 3D 控制节点

| 参数 | 默认值 |
|---|---:|
| `cmd_vel_topic` | `/asr_sdm/cmd_vel` |
| `controller_state_topic` | `/asr_sdm/controller_state_3d` |
| `control_cmd_topic` | `/asr_sdm/control_cmd_3d` |
| `control_period_ms` | `20` |
| `cmd_timeout_sec` | `0.3` |
| `max_linear_velocity` | `0.12` |
| `max_pitch_rate` | `0.35` |
| `max_yaw_rate` | `0.35` |
| `publish_control_cmd` | `false` |
| `joint_angle_scale` | `1.0` |
| `joint_angle_limit` | `2147483647` |
| `link_length` | `0.25` |
| `joint_rate_limit` | `2.0` |
| `joint_limit` | `0.85 * pi` |
| `max_curvature` | `1.2` |
| `curvature_velocity_epsilon` | `1.0e-3` |
| `damping` | `0.02` |

### 2D 可视化节点

| 参数 | 默认值 |
|---|---:|
| `state_topic` | `/asr_sdm/controller_state` |
| `link_length` | `0.25` |
| `draw_period_ms` | `100` |
| `max_history_size` | `3000` |

### 3D 可视化节点

| 参数 | 默认值 |
|---|---:|
| `state_topic` | `/asr_sdm/controller_state_3d` |
| `draw_period_ms` | `250` |
| `max_history_size` | `3000` |
| `trail_samples` | `360` |
| `body_snapshot_stride` | `80` |
| `follow_current_body` | `true` |
| `view_half_range` | `1.0` |
