# ASR-SDM planning packages

This directory contains the planning-side packages used by the ASR-SDM stack.
Four executables publish ROS 2 topics; the remaining packages are libraries
called by those nodes.

[English](#english) · [中文](#中文)

---

<a id="english"></a>

## English

### Packages

| Package | Responsibility | Main products |
|---|---|---|
| [`asr_sdm_guidance_planner`](asr_sdm_guidance_planner/) | 3D A* + sphere corridor + L-BFGS guidance waypoints | library `GuidancePlanner`; test node `rviz_astar_lbfgs_planner` |
| [`asr_sdm_local_path_modifier`](asr_sdm_local_path_modifier/) | TopologyPRM-style local detour around a blocked path segment | library `TopoPathModifier`; test node `local_path_modifier_test` |
| [`asr_sdm_trajectory_optimizer`](asr_sdm_trajectory_optimizer/) | Gradient-based B-spline trajectory optimization | library only, no topics |
| [`asr_sdm_trajectory_generator`](asr_sdm_trajectory_generator/) | Closed-form minimum-jerk / mini-snap trajectory | library; standalone node `traj_generator` |
| [`asr_sdm_trajectory_visualizer`](asr_sdm_trajectory_visualizer/) | RViz markers for goals, paths, B-splines, yaw | library used by `planning_manager_node` |
| [`asr_sdm_planning_manager`](asr_sdm_planning_manager/) | Topological replan FSM, B-spline publication, trajectory execution | `planning_manager_node`, `traj_server` |

Data flow:

```text
guidance waypoints → local modifier (optional) → planning_manager (B-spline) → traj_server (position command)
```

Guidance and local-modifier topics currently come from **test nodes**. Those
libraries are not yet wired into `planning_manager`.

### Published topics

#### 1. `planning_manager_node` (`asr_sdm_planning_manager`)

`TopoReplanFSM` publishes the planning outputs:

| Topic | Type | Role |
|---|---|---|
| `/planning/bspline` | `asr_sdm_planning_manager/msg/Bspline` | B-spline after a successful plan: control points, knots, yaw |
| `/planning/replan` | `std_msgs/msg/Empty` | Replan trigger; tells `traj_server` to drop the current trajectory |
| `/planning/new` | `std_msgs/msg/Empty` | New trajectory is ready |

Visualization is published through `asr_sdm_trajectory_visualizer`:

| Topic | Type | Role |
|---|---|---|
| `/planning_vis/trajectory` | `visualization_msgs/msg/Marker` | Goal, geometric path, polynomial global trajectory, B-spline |
| `/planning_vis/topo_path` | `visualization_msgs/msg/Marker` | Topological candidate paths (phase 1 / phase 2) |
| `/planning_vis/prediction` | `visualization_msgs/msg/Marker` | Dynamic obstacle prediction |
| `/planning_vis/visib_constraint` | `visualization_msgs/msg/Marker` | Visibility constraints |
| `/planning_vis/frontier` | `visualization_msgs/msg/Marker` | Frontier |
| `/planning_vis/yaw` | `visualization_msgs/msg/Marker` | Yaw trajectory |

#### 2. `traj_server` (`asr_sdm_planning_manager`)

Subscribes to `/planning/bspline` and samples it at 100 Hz.

| Topic | Type | Role |
|---|---|---|
| `/position_cmd` | `asr_sdm_control_msgs/msg/RobotCommand` | Position / velocity / acceleration / yaw command. Remapped to `planning/pos_cmd` in `asr_sdm_planning_manager.launch.py` |
| `planning/position_cmd_vis` | `visualization_msgs/msg/Marker` | Arrow at the current command pose |
| `planning/travel_traj` | `visualization_msgs/msg/Marker` | Flown trajectory so far |

With no namespace the last two resolve to `/planning/position_cmd_vis` and
`/planning/travel_traj`.

#### 3. `rviz_astar_lbfgs_planner` (`asr_sdm_guidance_planner` test node)

The library does not publish. The RViz test node publishes:

| Topic | Type | Role |
|---|---|---|
| `/planning/waypoints` | `nav_msgs/msg/Path` | L-BFGS-optimized guidance waypoints (QoS: reliable + transient_local) |
| `/planning/astar_path_marker` | `visualization_msgs/msg/Marker` | Raw A* path |
| `/planning/waypoints_marker` | `visualization_msgs/msg/Marker` | Optimized waypoint polyline |
| `/planning/start_goal_marker` | `visualization_msgs/msg/Marker` | Start and goal |
| `/planning/safe_corridor` | `visualization_msgs/msg/MarkerArray` | Spherical safe corridor |
| `/esdf_map/occupied_map` | `visualization_msgs/msg/Marker` | Occupied-map mesh (`TRIANGLE_LIST`) |

#### 4. `local_path_modifier_test` (`asr_sdm_local_path_modifier` test node)

The library does not publish. The test node publishes:

| Topic | Type | Role |
|---|---|---|
| `/planning/virtual_obstacles` | `visualization_msgs/msg/Marker` | Temporary spherical obstacles from RViz clicks |
| `/planning/topo_candidate_paths` | `visualization_msgs/msg/MarkerArray` | Selected TopologyPRM candidate detours (yellow) |

It does **not** publish `/planning/modified_path`; it publishes all selected
candidates.

#### 5. `traj_generator` (`asr_sdm_trajectory_generator`)

Standalone minimum-jerk generator; it does not go through `planning_manager`.

| Topic | Type | Role |
|---|---|---|
| `/traj_generator/traj_vis` | `visualization_msgs/msg/Marker` | Generated trajectory |
| `/traj_generator/cmd_vis` | `visualization_msgs/msg/Marker` | Current command-state arrow |
| `/drone_commander/onboard_command` | `swarmtal_msgs/msg/DroneOnboardCommand` | Onboard control command |

### Packages that do not publish

- `asr_sdm_trajectory_optimizer`: B-spline optimizer library
- Library parts of `asr_sdm_guidance_planner` and `asr_sdm_local_path_modifier`: called by the manager or test nodes

### Production interfaces

The three topics that matter for the production stack:

1. `/planning/bspline` — planning output
2. `/planning/replan` and `/planning/new` — trajectory lifetime
3. `/position_cmd` (launch remaps to `planning/pos_cmd`) — position command for the controller

### Launch

```bash
# Topological replan + traj_server
ros2 launch asr_sdm_planning_manager asr_sdm_planning_manager.launch.py

# Guidance planner test (RViz start/goal clicks)
ros2 launch asr_sdm_guidance_planner astar_lbfgs_planner.launch.py

# Local path modifier test (usually after guidance)
ros2 launch asr_sdm_local_path_modifier local_path_modifier_test.launch.py
```

---

<a id="中文"></a>

## 中文

`planning` 目录里真正会 **publish** 的是 4 个可执行节点；其余包是库，不直接发 topic。

数据流大致是：

```text
guidance waypoints → local modifier（可选）→ planning_manager（B-spline）→ traj_server（位置指令）
```

guidance / local modifier 的 `/planning/waypoints` 等目前只在测试节点上发，还没有接到 `planning_manager`。

### 发布的 topic

#### 1. `planning_manager_node`（正式规划栈）

包：`asr_sdm_planning_manager`，由 `TopoReplanFSM` 发布。

| Topic | 类型 | 内容 |
|---|---|---|
| `/planning/bspline` | `asr_sdm_planning_manager/msg/Bspline` | 规划成功后的 B-spline：控制点、knots、yaw |
| `/planning/replan` | `std_msgs/msg/Empty` | 触发重规划，通知 `traj_server` 丢掉当前轨迹 |
| `/planning/new` | `std_msgs/msg/Empty` | 新轨迹生成完成 |

可视化通过库 `asr_sdm_trajectory_visualizer` 发出：

| Topic | 类型 | 内容 |
|---|---|---|
| `/planning_vis/trajectory` | `visualization_msgs/msg/Marker` | 目标点、几何路径、多项式全局轨、B-spline |
| `/planning_vis/topo_path` | `visualization_msgs/msg/Marker` | 拓扑候选路径（phase1/phase2） |
| `/planning_vis/prediction` | `visualization_msgs/msg/Marker` | 动态障碍预测 |
| `/planning_vis/visib_constraint` | `visualization_msgs/msg/Marker` | 可见性约束 |
| `/planning_vis/frontier` | `visualization_msgs/msg/Marker` | frontier |
| `/planning_vis/yaw` | `visualization_msgs/msg/Marker` | yaw 轨迹 |

#### 2. `traj_server`（轨迹执行）

同一包，订阅 `/planning/bspline`，按 100 Hz 采样后下发控制指令。

| Topic | 类型 | 内容 |
|---|---|---|
| `/position_cmd` | `asr_sdm_control_msgs/msg/RobotCommand` | 位置/速度/加速度/yaw 指令。launch 里 remap 成 `planning/pos_cmd` |
| `planning/position_cmd_vis` | `visualization_msgs/msg/Marker` | 当前指令位置的箭头可视化 |
| `planning/travel_traj` | `visualization_msgs/msg/Marker` | 已飞过的轨迹 |

无 namespace 时后两个实际是 `/planning/position_cmd_vis`、`/planning/travel_traj`。

#### 3. `rviz_astar_lbfgs_planner`（guidance 测试节点）

包：`asr_sdm_guidance_planner`。库本身不发 topic；测试节点发：

| Topic | 类型 | 内容 |
|---|---|---|
| `/planning/waypoints` | `nav_msgs/msg/Path` | L-BFGS 优化后的引导路点（QoS：reliable + transient_local） |
| `/planning/astar_path_marker` | `visualization_msgs/msg/Marker` | A* 原始路径 |
| `/planning/waypoints_marker` | `visualization_msgs/msg/Marker` | 优化路点折线 |
| `/planning/start_goal_marker` | `visualization_msgs/msg/Marker` | 起终点 |
| `/planning/safe_corridor` | `visualization_msgs/msg/MarkerArray` | 球形安全走廊 |
| `/esdf_map/occupied_map` | `visualization_msgs/msg/Marker` | 占据地图 mesh（TRIANGLE_LIST） |

#### 4. `local_path_modifier_test`（局部改路测试节点）

包：`asr_sdm_local_path_modifier`。库本身不发 topic。

| Topic | 类型 | 内容 |
|---|---|---|
| `/planning/virtual_obstacles` | `visualization_msgs/msg/Marker` | RViz 点选的临时球形障碍 |
| `/planning/topo_candidate_paths` | `visualization_msgs/msg/MarkerArray` | TopologyPRM 选出的黄色候选绕行路径 |

不发 `/planning/modified_path`，只发全部候选路径。

#### 5. `traj_generator`（独立最小 jerk 轨迹）

包：`asr_sdm_trajectory_generator`，不走 planning_manager。

| Topic | 类型 | 内容 |
|---|---|---|
| `/traj_generator/traj_vis` | `visualization_msgs/msg/Marker` | 生成轨迹 |
| `/traj_generator/cmd_vis` | `visualization_msgs/msg/Marker` | 当前指令状态箭头 |
| `/drone_commander/onboard_command` | `swarmtal_msgs/msg/DroneOnboardCommand` | 机载控制指令 |

### 不发布 topic 的包

- `asr_sdm_trajectory_optimizer`：B-spline 优化库
- `asr_sdm_local_path_modifier` / `asr_sdm_guidance_planner` 的库部分：被 manager 或测试节点调用，不自己建 publisher

### 正式栈对外接口（最关键的 3 个）

1. `/planning/bspline`：规划输出
2. `/planning/replan` / `/planning/new`：轨迹生命周期
3. `/position_cmd`（launch 后为 `planning/pos_cmd`）：给控制器的位置指令
