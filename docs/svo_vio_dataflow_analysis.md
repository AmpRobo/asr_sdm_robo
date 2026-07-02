# SVO 系统数据链路与改造路线图

> 目标：在 `asr_sdm_video_inertial_odometry` 包内，把当前**松耦合 IMU-prior** 的 SVO 升级为**紧耦合 VIO**，参考 VINS-Mono 的数据链路。

---

## 1. 当前 SVO 数据链路（基线）

```
┌────────────────────────────────────────────────────────────────────────────┐
│                        Camera / ROSBag                                     │
│   /sensing/camera/realsense/color/image_raw   (D435i bag 直发)            │
│   /sensing/camera/realsense/imu               (D435i bag 直发)            │
│   /camera/image_raw + /imu/data               (其他 bag / 设备)           │
└────────────────────────────┬───────────────────────────────────────────────┘
                             │
   ┌─────────────────────────┴─────────────────────────────────┐
   ▼                                                           ▼
┌──────────────────────┐                          ┌────────────────────────┐
│  camera_node (可省)  │                          │   imu_node (可省)      │
│  话题重映射          │                          │   /imu/data →          │
│  /camera/*  →        │                          │   /sensing/imu/imu_raw │
│  /sensing/camera/    │                          │   /sensing/imu/        │
│  realsense/color/*   │                          │   imu_filtered (直通)  │
└──────────┬───────────┘                          └──────────┬─────────────┘
           │                                                 │
           └────────────────────┬────────────────────────────┘
                                ▼
┌───────────────────────────────────────────────────────────────────────────┐
│                     VoNode（svo_ros/vo_node.cpp）                          │
│                                                                           │
│  Thread 1..4: ROS MultiThreadedExecutor（4 线程）                          │
│    ├─ imuCb() ────► ImuHandler::addImuMeasurement()                        │
│    │                (SVO 内部 IMU 缓存，rotation prior 模式)               │
│    ├─ imgCb() ────► latest_img_ + img_cv_.notify_one()                    │
│    │                (节流 15 FPS，丢弃乱序)                                │
│    └─ remoteKeyCb()                                                       │
│                                                                           │
│  Thread 5: SVO processing loop（VoNode::run()）                            │
│    └─ wait on img_cv_ ─► vo_->addImage(img, ts)                            │
│         └─ FrameHandlerMono::addImage():                                   │
│              1. SparseImgAlign (IMU rotation prior, λ=0.5)                 │
│              2. Reprojector (map 重投影匹配)                                │
│              3. PoseOptimizer (g2o, 可选 IMU prior λ=0)                    │
│              4. needNewKf() → DepthFilter                                  │
│              5. (可选) optimizeStructure() / LOBA (loba_num_iter=3)        │
│                                                                           │
│  发布（visualizer）:                                                       │
│    /localization/video_inertial_odom/pose       PoseWithCovarianceStamped  │
│    /localization/video_inertial_odom/trajectory nav_msgs/Path              │
│    /localization/video_inertial_odom/points    Marker (3D 点云)            │
│    /localization/video_inertial_odom/keyframes Marker (相机视锥)           │
│    /localization/video_inertial_odom/info      Info (SLAM 状态)           │
│    /localization/video_inertial_odom/dense_input DenseInput                │
└───────────────────────────────────────────────────────────────────────────┘
```

### 1.1 关键观察

| 项目 | 当前状态 | 影响 |
|------|---------|------|
| **IMU 融合方式** | 松耦合（rotation prior）| IMU 只约束旋转初值，不参与优化量 |
| **优化框架** | 手写 g2o（SparseImgAlign + PoseOptimizer + LOBA） | 紧耦合 IMU factor **没有** |
| **IMU 预积分** | SVO 原生 `PreintegratedImuMeasurement`（仅 9 维状态）| 没有 15 维 VINS 风格预积分 |
| **滑窗管理** | SVO map 管理（`maxNKfs=180`）| 没有 VINS 风格的"窗口 10 帧 + 边缘化"|
| **回环检测** | 无（`runlc: False`）| 无 DBoW2 |
| **`svo_vio_backend`** | 纯 C++ 库，**未被任何节点调用** | 写好的紧耦合代码是"死代码" |
| **节点** | 单节点 `vo`（SVO+IMU）| 单一管线，没有 VINS 那种"特征+估计器+位姿图"拆分 |
| **线程数** | 5 (4 ROS + 1 SVO) | 单处理线程优化 |

---

## 2. VINS 数据链路（参考目标）

```
┌───────────────────────────────────────────────────────────────────────────┐
│  Camera/Bag                                                                │
│   /sensing/camera/realsense/color/image_raw                                 │
│   /sensing/camera/realsense/imu                                             │
└──────┬────────────────────────────────────┬─────────────────────────────────┘
       │                                    │
       ▼                                    ▼
┌──────────────────┐                ┌──────────────────┐
│ feature_tracker  │                │  vins_estimator  │
│ (单线程)         │                │  (2 线程)        │
│                  │   /feature     │                  │
│  KLT 光流 +      ├────────────────►  主线程 spin:     │
│  RANSAC 提特征   │  /feature_img  │   imu 入队      │
│                  │                │   feature 入队   │
│  发布:           │                │                  │
│   feature        │                │   measurement_   │
│   feature_img    │                │   process 线程:  │
│   restart        │                │   - IMU 预积分   │
└──────────────────┘                │   - 滑窗优化     │
                                    │   - Ceres 求解   │
                                    │                  │
                                    │  发布:           │
                                    │   odometry       │
                                    │   keyframe_pose  │
                                    │   keyframe_point │
                                    └────────┬─────────┘
                                             │
                                             ▼
                                    ┌──────────────────┐
                                    │   pose_graph     │
                                    │   (4 线程)       │
                                    │  - 主线程 spin   │
                                    │  - process 线程  │
                                    │  - keyboard 线程 │
                                    │  - optimize 线程 │
                                    │  - DBoW2 回环    │
                                    │  - 4DoF 优化     │
                                    └──────────────────┘
```

**VINS 核心特点**：
- **特征/估计器分离**：特征提取与优化解耦
- **紧耦合优化**：Ceres + 15 维 IMU factor + 视觉重投影 + 边缘化
- **三模块并行**：feature_tracker / vins_estimator / pose_graph 三个独立节点
- **回环检测**：DBoW2 + 4DoF 位姿图优化

---

## 3. SVO 改造为紧耦合 VIO 的路线图

### 3.1 改造原则

- **保持 SVO 前端优势**：直接法 + 半稠密地图 + 高速率
- **复用现成的 `svo_vio_backend`**：已有的 `VioBackend / SlidingWindow / VinsOptimizer / IMUPreintegrator / VinsFactors`
- **不抛弃 SVO 的 IMU 旋转 prior**：可作为初值/安全网
- **参考 VINS 三模块架构**（feature / estimator / pose_graph）

### 3.2 目标架构（终态）

```
┌──────────────────────────────────────────────────────────────────────────┐
│ Camera / Bag                                                              │
│   /sensing/camera/realsense/color/image_raw                               │
│   /sensing/camera/realsense/imu                                           │
└──────┬──────────────────────────────────────┬──────────────────────────────┘
       │                                      │
       ▼                                      ▼
┌────────────────────────┐            ┌────────────────────────┐
│  svo_feature_tracker   │            │   svo_vio_node          │
│  (新建节点)            │            │   (改造现有 vo_node)    │
│  - KLT 光流            │  /svo/      │                        │
│  - RANSAC              ├───────────►│  Main thread:          │
│  - 发布特征点 + 帧     │  features  │   imuCb (入队)         │
│  - (可暂时用 SVO 直    │            │   featureCb (入队)     │
│     接法前端)          │            │   remoteKeyCb          │
│                        │            │                        │
│  暂时不实现，复用 SVO  │            │  VIO process 线程:     │
│  前端：让 SVO 自身     │            │   - getMeasurements    │
│  addImage() 后提取特征 │            │   - VioBackend::       │
│  点云                  │            │       addKeyFrame()    │
│                        │            │   - VioBackend::       │
│                        │            │       optimize()       │
│                        │            │   - publish()          │
└────────────────────────┘            └──────────┬─────────────┘
                                                  │
                                                  ▼
                                        ┌────────────────────────┐
                                        │   svo_pose_graph       │
                                        │   (新建节点)            │
                                        │   - DBoW2 回环         │
                                        │   - 4DoF 优化          │
                                        │   - 漂移校正            │
                                        └────────────────────────┘
```

### 3.3 分阶段实施计划

#### **Phase 1：把 `svo_vio_backend` 接入 SVO 主流程（最关键）**

目标：SVO 每帧处理完后，把"位姿 + 关键帧"喂给 `VioBackend`，由后端做紧耦合 IMU 优化，**前端不退化**（SVO 仍跑自己的 pipeline，VioBackend 只是追加在后面做 refinement）。

具体改动：

1. **`svo_ros/src/vo_node.cpp`**：在 `VoNode` 中加入 `svo_vio_backend::VioBackend` 成员
   - 构造时初始化：`setGravity / setIMUExtrinsics / setIMUNoise`
   - IMU callback 中：**同时**调用 `vo_->imu_handler_->addImuMeasurement()` 和 `vio_backend_->addIMUMeasurement()`
   - `run()` 主循环中：每收到一帧 → `vo_->addImage()` → 如果是关键帧 → `vio_backend_->addKeyFrame()`
   - 关键帧触发优化：`vio_backend_->optimize()`（节流到 10 Hz，避免卡前端）
   - 优化后读取位姿：`vio_backend_->getCurrentPose()` → 作为 VIO 估计位姿发布

2. **`svo_vio_backend` 补全空实现**：
   - `buildOptimizationProblem()`：把所有滑窗帧的位姿/速度/零偏 + IMU 预积分 + 视觉重投影残差加入 Ceres Problem
   - `updateStatesFromOptimization()`：从 Ceres 求解结果回写到 `VinsSlidingWindow`
   - `marginalize()`：滑窗满时把最老帧做 Schur complement 边缘化

3. **新建 `svo_vio_node` 包**（也可直接在 `svo_ros` 内部加）：
   - 复用 `vo_node.cpp` 的所有逻辑 + VioBackend 集成
   - 新增 ROS 2 话题 `/svo_vio/odometry`（紧耦合优化后位姿）、`/svo_vio/keyframe_pose`、`/svo_vio/keyframe_point`、`/svo_vio/imu_propagate`（高频 IMU 预测）
   - 与 VINS 保持 API 一致，便于后续接 pose_graph

4. **新建 launch 文件**：`svo_vio.launch.py`
   - 启动 `vo` 节点（开启 `use_imu:=true` + VioBackend 启用开关）
   - 不再需要单独的 feature_tracker 节点
   - 与 D435i bag 兼容

#### **Phase 2：发布统一话题接口（对齐 VINS）**

让 SVO VIO 节点发布 VINS 同款话题（`/vins_estimator/odometry` 等），便于：
- 用 RViz 复用 VINS 配置直接可视化
- 后续 `pose_graph` 可以无缝接入
- 下游模块（如 `asr_sdm_slam` 决策）统一订阅

#### **Phase 3：单独抽出 `svo_pose_graph`（可选）**

新建独立节点，订阅 `/svo_vio/keyframe_pose + /svo_vio/keyframe_point`，做 DBoW2 回环 + 4DoF 优化。

参考 VINS：`pose_graph` 已经存在于 `asr_sdm_video_inertial_navigation_systems/pose_graph`，**可以考虑直接复用**这个节点，只要 SVO 节点发布的话题名对齐。

#### **Phase 4：可选扩展**

- 双目 SVO（`FrameHandlerStereo`）对接 VIO
- 在线外参标定
- 回环后全局优化（VINS 是位姿图，SVO 可以重置前端 map 适配新位姿）

### 3.4 关键文件改动清单

| 文件 | 改动 |
|------|------|
| `svo_ros/src/vo_node.cpp` | 集成 `VioBackend`，新增 `vio_backend_` 成员，IMU 双写，关键帧触发优化 |
| `svo_ros/CMakeLists.txt` | 加 `find_package(svo_vio_backend)`，link 库 |
| `svo_ros/package.xml` | 加 `svo_vio_backend` 依赖 |
| `svo_vio_backend/src/vio_backend.cpp` | 补全 `buildOptimizationProblem / updateStatesFromOptimization / marginalize` |
| `svo_vio_backend/include/svo_vio_backend/vio_backend.h` | 添加 4DoF pose graph 接口（如果走 Phase 3） |
| `svo_ros/launch/svo_vio.launch.py` | 新建启动文件 |
| `svo_ros/param/svo_vio_rig3.yaml` | 新建 VIO 参数文件（IMU noise, gravity, ric/tic） |

### 3.5 数据流改造前后对比

| 阶段 | 数据流 |
|------|-------|
| **改前** | 图像 → SVO frontend → g2o（pose only）→ 发布 |
| **Phase 1 改后** | 图像 → SVO frontend → SVO g2o → **同时** VioBackend 紧耦合 Ceres → 紧耦合位姿发布 |
| **Phase 2 改后** | + 话题对齐 VINS（统一接口）|
| **Phase 3 改后** | + pose_graph 回环 + 4DoF 优化 |

### 3.6 风险与对策

| 风险 | 影响 | 对策 |
|------|------|------|
| `VioBackend` 优化与 SVO 前端"打架" | 双线程数据竞争 | 优化节流 10 Hz，加 `m_backend` 互斥锁 |
| Ceres 优化慢于图像帧率 | 前端卡顿 | `max_solver_time: 0.04s`，`max_num_iterations: 8` |
| 边缘化破坏 SVO 关键帧图 | 前端跟踪失败 | VioBackend 只对滑窗内的最近 N 帧优化，不动 SVO map |
| 坐标系冲突（SVO world vs IMU world）| 估计发散 | 用 `gravity-aligned roll/pitch` 重力对齐（已在 `vo_node` 中使用）|
| D435i IMU 噪声参数不准 | 优化不收敛 | 用 VINS 同款 D435i BMI055 实测值（已写进 VINS 配置）|

---

## 4. 当前可以立即开始的最小可行改造

如果想**最快看到效果**，建议这样推进：

1. **Step 1（1-2 天）**：在 `vo_node.cpp` 中实例化 `VioBackend`，把 IMU 数据双写进 VioBackend。打印 `getCurrentPose()`，确认数据通路通了。
2. **Step 2（1 周）**：在 `VioBackend::buildOptimizationProblem()` 中把滑窗内 IMU 预积分 + 视觉重投影残差接进 Ceres。能在 RViz 中看到 VIO 紧耦合位姿。
3. **Step 3（1-2 周）**：补全 `marginalize()`，让滑窗稳定。
4. **Step 4（可选）**：对接 `pose_graph` 回环。

---

## 5. 与 `docs/vins_dataflow_analysis.md` 对照

| 维度 | VINS | SVO（当前）| SVO 改造后 |
|------|------|----------|-----------|
| 特征点 | KLT + RANSAC | 直接法（SparseImgAlign）| 保持直接法前端（优势）|
| 优化框架 | Ceres | g2o（前端）| **Ceres**（VioBackend）|
| IMU 融合 | 15 维紧耦合 | 9 维 rotation prior | **15 维紧耦合** |
| 滑窗 | 固定 10 帧 + 边缘化 | 180 帧 map | **10 帧滑窗 + 边缘化** |
| 回环 | DBoW2 + 4DoF | 无 | 复用 VINS pose_graph |
| 节点拆分 | 3 节点 | 1 节点 | 2 节点（vo + pose_graph） |
| 话题 | 标准 VINS | 自定义 svo/* | 对齐 VINS 话题 |
