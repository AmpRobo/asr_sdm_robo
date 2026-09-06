# ASR-SDM TF Tree

**Scope**: `asr_sdm` robot model, `logging_simulator`, `planning_simulator`, and VINS visualization  
**RViz Fixed Frame**: `world`  
**Date**: 2026-08

**English** | [中文](#中文)

---

## 1. Full TF tree

```
world
 └── base                          ROS body frame: +X forward, +Y left, +Z up
      ├── camera                   VINS extrinsics tic / ric (only when VINS is running in logging)
      ├── laser / vision / height  extra sensor frames from odom_visualization
      └── base_link                coincident with base (identity)
           └── screwdrive_segment_0     CAD: −π/2 about Y, heading along base_link +X
                ├── screw_rotor_left_0 / right_0
                └── joint_unit_a_0 → cross_0 → b_0
                     └── screwdrive_segment_1 → … → segment_3
```

`world → base` is published only by `odom_visualization`, so `base` never has two parents.

---

## 2. Frame conventions

| Frame | Meaning | Axes |
|---|---|---|
| `world` | World / map frame | RViz Fixed Frame |
| `base` | Odometry child frame and controller body frame | ROS: +X forward, +Y left, +Z up |
| `base_link` | URDF link coincident with `base` | Same as `base` |
| `screwdrive_segment_*` | CAD segment frame | Length along +Z; head toward −Z |
| `camera` | Camera optical frame | VINS extrinsics; optical axis along camera +Z |
| `laser` / `vision` / `height` | Extra frames from `odom_visualization` | Attached under `base`; not part of the kinematics |

The CAD mesh heading (segment −Z) is rotated by −π/2 about Y in `base_link_to_screwdrive_segment_0` so it lines up with `base_link +X`. That matches the controller, `cmd_vel.linear.x`, and the RViz 2D Pose Estimate heading.

---

## 3. Who publishes which transform

| Transform | Publisher | Notes |
|---|---|---|
| `world → base` | `odom_visualization` | Built from odometry; no static TF when `parent_frame == child_frame == base` |
| `base → camera` | VINS `pubTF()` | IMU-to-camera extrinsics `tic` / `ric` |
| `base → laser / vision / height` | `odom_visualization` | Visualization-only frames |
| `base → base_link → segments / rotors` | `robot_state_publisher` | URDF plus `/control/joint_states` |

The camera frustum is **not** a TF. It is a MarkerArray in `world`:

- Topic: `/localization/video_inertial_navigation_systems/camera_pose_visual`
- `header.frame_id`: `world`
- Pose: IMU pose times the extrinsics, i.e. the camera optical pose in `world`

---

## 4. Where `world → base` comes from

| Scene | Pose source | Heading |
|---|---|---|
| planning_simulator | Controller `/control/asr_sdm/odom` | Already ROS: `yaw = 0` faces world +X; 2D Pose Estimate / Nav Goal use the same convention |
| logging_simulator (before VINS) | Initial pose in `logging_simulator.yaml` | `yaw = 0`, faces world +X |
| logging_simulator (after VINS) | VINS odometry (IMU, optical axis +Z) | TF is post-multiplied by `tf_pitch = −π/2`, `tf_yaw = π` to reach ROS `base` |

The logging compensation lives under `odom_visualization`:

```yaml
odom_visualization:
  tf_roll: 0.0
  tf_pitch: -1.5707963267948966
  tf_yaw: 3.141592653589793
```

planning leaves these at zero because controller odometry is already in the ROS body frame.

---

## 5. Related files

| File | Role |
|---|---|
| `src/asr_sdm_universe/robot/asr_sdm/urdf/asr_sdm.urdf.xacro` | URDF root `base`, plus `base_link` and segment joints |
| `src/asr_sdm_universe/robot/asr_sdm/launch/asr_sdm_description.launch.py` | Starts `robot_state_publisher`; skips the static TF when the root is already `base` |
| `src/asr_sdm_simulator/Utils/odom_visualization/src/odom_visualization.cpp` | `world → base`, and the logging IMU-to-ROS rotation |
| `src/asr_sdm_universe/control/asr_sdm_control_manager/` | Publishes `/control/asr_sdm/odom` with `child_frame_id = base` |
| `src/asr_sdm_universe/localization/.../vins_estimator/src/utility/visualization.cpp` | `base → camera`; camera-frustum MarkerArray |
| `src/asr_sdm_simulator/logging_simulator/config/logging_simulator.yaml` | Logging initial pose and `tf_*` |
| `src/asr_sdm_universe/control/asr_sdm_control_manager/config/asr_sdm_control_manager.yaml` | Planning initial pose (ROS yaw) |

---

# 中文

[English](#asr-sdm-tf-tree) | **中文**

**适用范围**：`asr_sdm` 机器人模型、`logging_simulator`、`planning_simulator`、VINS 可视化  
**RViz Fixed Frame**：`world`  
**日期**：2026-08

---

## 1. 整棵 TF 树

```
world
 └── base                          ROS 机体系：+X 前、+Y 左、+Z 上
      ├── camera                   VINS 外参 tic / ric（仅 logging 有 VINS 时）
      ├── laser / vision / height  odom_visualization 附带的传感器框
      └── base_link                与 base 重合（单位变换）
           └── screwdrive_segment_0     CAD：绕 Y 转 −π/2，头朝 base_link +X
                ├── screw_rotor_left_0 / right_0
                └── joint_unit_a_0 → cross_0 → b_0
                     └── screwdrive_segment_1 → … → segment_3
```

`world → base` 只由 `odom_visualization` 发布，避免 `base` 出现两个父坐标系。

---

## 2. 各坐标系约定

| 坐标系 | 含义 | 轴向 |
|---|---|---|
| `world` | 世界 / 地图系 | RViz Fixed Frame |
| `base` | 里程计子坐标系、控制器机体系 | ROS：+X 前、+Y 左、+Z 上 |
| `base_link` | URDF 中与 `base` 重合的连杆 | 与 `base` 相同 |
| `screwdrive_segment_*` | CAD 节段系 | 沿节段 +Z 生长，头部朝 −Z |
| `camera` | 相机光心系 | VINS 外参；光轴沿相机 +Z |
| `laser` / `vision` / `height` | `odom_visualization` 附带框 | 挂在 `base` 下，不参与运动学 |

CAD 网格头部（节段 −Z）经 `base_link_to_screwdrive_segment_0` 绕 Y 转 −π/2，对齐到 `base_link +X`，因此和控制器、`cmd_vel.linear.x`、RViz 2D Pose Estimate 的前进方向一致。

---

## 3. 谁发布哪一段

| 变换 | 发布者 | 说明 |
|---|---|---|
| `world → base` | `odom_visualization` | 由里程计转成 TF；`parent_frame == child_frame == base` 时不再发静态 TF |
| `base → camera` | VINS `pubTF()` | IMU→相机外参 `tic` / `ric` |
| `base → laser / vision / height` | `odom_visualization` | 可视化附属框 |
| `base → base_link → 各节 / 转子` | `robot_state_publisher` | 读 URDF + `/control/joint_states` |

相机框本身**不是 TF**，而是 `world` 下的 MarkerArray：

- topic：`/localization/video_inertial_navigation_systems/camera_pose_visual`
- `header.frame_id`：`world`
- 位姿：IMU 位姿乘外参，即相机在 `world` 下的光心位姿

---

## 4. `world → base` 在两个仿真器中的来源

| 场景 | 位姿来源 | 朝向 |
|---|---|---|
| planning_simulator | 控制器 `/control/asr_sdm/odom` | 已是 ROS：`yaw = 0` 朝世界 +X；2D Pose Estimate / Nav Goal 直接用这套 |
| logging_simulator（VINS 未到） | `logging_simulator.yaml` 初始位姿 | `yaw = 0`，朝世界 +X |
| logging_simulator（VINS 已到） | VINS 里程计（IMU，光轴 +Z） | 发布 TF 时再乘 `tf_pitch = −π/2`、`tf_yaw = π`，转到 ROS `base` |

logging 里的补偿写在 `odom_visualization` 段：

```yaml
odom_visualization:
  tf_roll: 0.0
  tf_pitch: -1.5707963267948966
  tf_yaw: 3.141592653589793
```

planning 不设这三项（全 0），因为控制器里程计已经是 ROS 机体系。

---

## 5. 相关文件

| 文件 | 作用 |
|---|---|
| `src/asr_sdm_universe/robot/asr_sdm/urdf/asr_sdm.urdf.xacro` | URDF 根连杆 `base`，以及 `base_link` / 节段连接 |
| `src/asr_sdm_universe/robot/asr_sdm/launch/asr_sdm_description.launch.py` | 启动 `robot_state_publisher`；根连杆已是 `base` 时不发静态 TF |
| `src/asr_sdm_simulator/Utils/odom_visualization/src/odom_visualization.cpp` | `world → base`，以及 logging 的 IMU→ROS 旋转 |
| `src/asr_sdm_universe/control/asr_sdm_control_manager/` | 发布 `/control/asr_sdm/odom`，`child_frame_id = base` |
| `src/asr_sdm_universe/localization/.../vins_estimator/src/utility/visualization.cpp` | `base → camera`；相机框 MarkerArray |
| `src/asr_sdm_simulator/logging_simulator/config/logging_simulator.yaml` | logging 初始位姿与 `tf_*` |
| `src/asr_sdm_universe/control/asr_sdm_control_manager/config/asr_sdm_control_manager.yaml` | planning 初始位姿（ROS yaw） |
