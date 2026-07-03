# VINS-Mono ROS2 程序架构总结（含近期 sparse / td / geometric-gate 改造）

**适用范围**：`src/asr_sdm_universe/perception/asr_sdm_video_inertial_navigation_systems/`
**对应分支**：`feat/sparse-rotation-geometric-gate`（含此前 `feat/sparse-align-td-loopverify` 的部分被回滚）
**修改日期**：2026-06

---

## 1. 模块整体架构

```
asr_sdm_video_inertial_navigation_systems/
├── camera_model           // 鱼眼/针孔相机模型 + 标定解析
├── feature_tracker        // 前端：特征跟踪（SVO-style 稀疏改造点）
├── vins_estimator         // 后端：VIO 紧耦合 + 回环检测入口
├── pose_graph             // 4-DOF pose graph + 回环验证
├── benchmark_publisher    // 评测对齐发布
├── ar_demo                // AR 可视化 demo
├── config_pkg             // YAML / support_files（launch 引用）
└── rpg_svo_pro_open       // SVO 参考实现
```

数据流：

```
[Image + IMU]
   │
   ▼
camera_model  ──►  feature_tracker  ──►  vins_estimator  ──►  pose_graph  ──►  /vins_estimator/odometry, /pose_graph/...
   (标定/去畸变)        (KLT + sparse)         (VIO BA)            (回环 + 4DOF graph)
```

---

## 2. 各子模块详细组成

### 2.1 `camera_model`
- 相机模型库（pinhole / MEI 等）
- 标定文件解析，Ceres 标定工具

### 2.2 `feature_tracker`（前端）
**核心源文件**：
- `feature_tracker.{cpp,h}`：KLT 主流程
- `parameters.{cpp,h}`：参数加载
- `half_sample.h`：SVO 的 4-level half-sampled 金字塔
- `tic_toc.h`：计时

**新增模块**（SVO-style 稀疏改造）：
- `sparse_img_align.{cpp,h}`：semi-direct photometric alignment（Gauss-Newton + 4-level half pyramid），估计帧间 R/t，输出 `final_chi2`
- `imu_preintegrate.{cpp,h}`：极简 IMU 帧间 R 预积分，提供 `R_prev_cur_` 作为 sparse align 的初值
- `td_pre_calibrator/{td_pre_calibrator.h,td_pre_calibrator.cpp}`：camera-IMU 时间偏移预标定

### 2.3 `vins_estimator`（后端 VIO）
**目录结构**：
- `src/`：核心 estimator 源码
- `src/loop_geometric_verifier/`：geometric gate 模块（独立模块，对候选回环做旋转一致性检查；本轮在 `pose_graph` 中以新实现替代了这里的版本）
- `cmake/`：Ceres / OpenCV 等 find 脚本
- `launch/`：
  - `euroc.launch.py`：EuRoC 数据集
  - `vins.launch.py`：通用启动文件（取代旧的 `d435i_combined.launch.py`），默认读 `config/d435i/d435i.yaml`，支持 `config_yaml` / `image_topic` / `imu_topic` / `output_root` / `enable_sparse` 五个参数；可同时在不同 `ROS_DOMAIN_ID` 下跑两条链路做 ORIGINAL vs SPARSE1 对比

### 2.4 `pose_graph`（回环 + 4DOF graph）
**源文件**：
- `pose_graph.{cpp,h}` + `pose_graph_node.cpp`：回环检测入口（DBoW2）、4DOF graph optimization
- `keyframe.{cpp,h}`：关键帧管理
- `loop_geometric_verifier/`：旧版 geometric gate（已在 remote 上被 revert）
- `parameters.h` / `utility/` / `ThirdParty/`（DBoW2）

---

## 3. 我们此前的修改演进（commit 时间线）

```
feat(svo): integrate asr_sdm_svo reference pipeline      ← SVO 参考实现接入
docs(vins): README SVO-style sparse front-end section
feat(vins): SVO-style sparse-prior KLT front-end         ← 前端稀疏化（方向 A）
fix(vins): pose_graph support_file fallback + estimator IMU order
docs(vins): README - explain that first build needs the full chain
docs: add D2 improvement plan (D435i main, VINS baseline preserved)
chore: fixed some warnings
feat(vins): D2 W1 — publish sparse_align R over new SparseRot msg
Merge branch 'main' into feat/sparse-klt-vins
feat(vins): D2 — sparse_align + td_pre_calib with parallel pipeline verification
feat(vins): add loop geometric verifier to vins_estimator  ← 后被回滚
[revert] Revert "feat(vins): add loop geometric verifier to vins_estimator"
feat(pose_graph): add sparse rotation geometric gate for loop verification  ← 当前分支
```

---

## 4. 当前架构（核心改造点）

### 4.1 前端 SVO-style 稀疏改造（feature_tracker）
**核心思想**：用 photometric alignment 估计帧间 R，根据对齐质量 `final_chi2` **自适应缩小 KLT 搜索窗**：

```
prev_img 4-level half pyramid (复用)
   + cur_img pyramid
   + cur_pts (上一帧像素)
   + IMU R_prev_cur_ (初值)
   + fx/fy/cx/cy
        │
        ▼
  sparse_img_align  (Gauss-Newton, patch=2, max_level=2, max_iter=4, chi2_thresh=50)
        │
        ▼
   final_chi2  ──►  自适应选 KLT 搜索窗
   chi2 < 5   → 5×5,  1 level
   chi2 < 15  → 9×9,  1 level
   chi2 < 50  → 15×15, 2 levels
   sparse fail → fallback 21×21, 3 levels（原版）
```

**关键不变量**：
- KLT 起点仍是 `cur_pts`（不变）
- 仅缩小 `winSize` 和 `maxLevel`
- VINS 后端收到的 sub-pixel 特征点与原版一致 → **BA cost 行为不变**
- IMU 帧间 R 在 VINS 内部的主导地位保留

### 4.2 单链路运行 + 双链路对比

`ros2 launch vins_estimator vins.launch.py` 默认跑一条链路，读
`config_pkg/config/d435i/d435i.yaml`；把 `enable_sparse:=1` 打开则启用
SVO-style sparse align + td pre-calib；`enable_sparse:=0` 则是原版 VINS
baseline。

两条链路对比时不再用单一 launch 文件，而是用 `ROS_DOMAIN_ID` 隔离
（在两个终端里分别起同一份 launch）：

| 链路 | 启动命令关键参数 | feature_tracker 行为 | vins_estimator | pose_graph |
|------|----------------------|----------------|------------|------------|
| **ORIGINAL** | `enable_sparse:=0`，`ROS_DOMAIN_ID=0` | KLT 21×21, 3-level（baseline） | 原版 VINS | 原版 |
| **SPARSE1**  | `enable_sparse:=1`，`ROS_DOMAIN_ID=1` | sparse align + KLT 自适应窗 | 同原版 | 同原版 |

两条链路各自的输出目录由各自的 `output_root` 决定（默认
`$HOME/output/vins/`），便于 csv 对比。

### 4.3 回环验证：sparse rotation geometric gate（当前分支新增）
**位置**：`pose_graph/src/`

**动机**：DBoW2 返回的回环候选可能因视角变化导致误检，**纯重投影误差门**不够鲁棒。

**方案**：在重投影误差门**之前**，先做一道 **sparse 旋转角一致性检查**：
- 取当前帧与回环候选帧的 sparse 旋转 `R_sparse_curr`、`R_sparse_loop`
- 估算帧间相对旋转角度
- 与候选相对位姿中的旋转角比对，超过阈值则拒绝

**重要不变量**：
- 只在**回环候选路径**生效，不进 BA
- 重投影误差门仍保留，gate 是**前置筛选**
- BA 迭代次数 / `total_time` 不受影响（已用日志验证）

### 4.4 消息总线（当前发布）
- `SparseRot`（vins_estimator）：把 sparse_align R 暴露给外层，为后续"把 sparse R 当 VINS 帧间 R 先验"打基础（方向 B）

---

## 5. td_pre_calibrator（camera-IMU 时间偏移预标定）

**位置**：`feature_tracker/src/td_pre_calibrator/`

**动机**：VINS-Mono BA 把 `td` 当未知量在线学习，但当 sync 不准时，投影因子存在系统偏差 → BA 可能要几百次迭代，且容易卡在错误局部最小值。

> 这个模块在 **BA 跑之前**先给出一个**预标定 td**，让 BA 起点更接近真值。

### 5.1 模型

```
t_cam_physical = t_cam_stamp + td       (待求的 td)
```

前端 sparse_align 实际上跑两遍：
- 第一遍：用 IMU 先验（`R_prev_cur_`）warm-start → 得到 R_prior
- 第二遍：把 lambda_rot 设为 0，**完全纯视觉** → 得到 R_vision（参考质量）

当两遍结果不一致时，不一致程度 ≈ f(局部角速度 ‖ω‖, 未知 td)。

### 5.2 求解目标

求解 1-D 优化问题：

```
minimize_td   || log( R_imu(td) · R_vision^T ) ||_2

其中  R_imu(td) = ∫ exp(ω·dt) on [t_img_prev + td, t_img_cur + td]
       R_vision  = sparse_align 在 lambda_rot=0 时的纯视觉输出
```

### 5.3 算法流程

1. **粗网格搜索**（grid search）
   - 搜索范围：`td ∈ [-20ms, +20ms]`（覆盖 D435i 实测 unsync drift ±5ms）
   - 网格密度：41 个点（≈20 steps/ms）

2. **精细化搜索**（ternary search）
   - 在粗搜索最优点 ±0.5ms 窗口内做 8 次三分搜索
   - 单峰谷 → 收敛快

3. **数值积分**：piecewise-constant gyro
   ```
   R = Π_k exp(ω_k · Δt_k)
   ```
   每段用 axis-angle 公式 `R = I + sin(θ)K + (1-cos(θ))K²`。

### 5.4 关键参数

| 参数 | 默认 | 含义 |
|------|------|------|
| `td_min` / `td_max` | ±20ms | 搜索范围 |
| `n_samples` | 41 | 网格密度 |
| `min_angle_deg` | 0.05° | 帧间旋转太小则丢弃（不具信息量）|
| `min_dt` | 1ms | 帧间最小时间间隔 |
| `max_iter` | 8 | 精细化迭代次数 |
| `refine_window` | ±0.5ms | 精细化窗口 |

### 5.5 接口

```cpp
TdPreCalibrator c;
c.beginFrame(t_prev, t_cur);             // 一帧开始
c.setVisionRotation(R_vision);           // 纯视觉 R（lambda_rot=0）
c.addGyro(t, wx, wy, wz);                // 窗口内所有 gyro 样本
double td = c.solve();                   // 秒，失败返回 NaN
double td_mean = c.meanTd();             // 滚动平均
```

### 5.6 在双链路中的位置

```
                      ┌──────────────────────────┐
   cur_img, prev_img  │  feature_tracker (SP1)   │
   + cur_pts          │                          │
   + IMU stream ─────►│  ① sparse_align (IMU 先验)│
                      │     → R_prior            │
                      │                          │
                      │  ② sparse_align (λ=0)    │
                      │     → R_vision           │
                      │                          │
                      │  ③ td_pre_calibrator     │
                      │     solve(t_prev,t_cur,  │
                      │            R_vision,gyro) │
                      │     → td                 │
                      │                          │
                      │  ④ 按 chi2 选 KLT 搜索窗 │
                      └────────────┬─────────────┘
                                   │ /feature + td
                                   ▼
                      ┌──────────────────────────┐
                      │  vins_estimator (SP1)    │
                      │  BA 用 td_init = td_预标定 │
                      │  → 起点更准，迭代更少     │
                      └──────────────────────────┘
```

### 5.7 线程与无副作用保证

- 纯 CPU，无 ROS，无锁
- 每个 pipeline（ORIGINAL / SPARSE1）独立一个实例
- **不进入 VIO 主优化路径**，仅提供一个起始 `td` 给 BA

### 5.8 与 sparse geometric gate 的关系

| 模块 | 位置 | 作用阶段 | 不变量 |
|------|------|---------|--------|
| **td_pre_calibrator** | `feature_tracker` | BA 启动**前** | 不进 BA 主循环，仅提供初始 td |
| **sparse rotation geometric gate** | `pose_graph` | 回环候选**前置过滤** | 不进 BA，仅拒绝可疑回环 |
| **sparse_align 自适应 KLT 窗** | `feature_tracker` | 每帧 KLT 之前 | 后端 BA cost 行为不变 |

三个模块分别守在数据流的三个不同关口，**都不进 BA 主优化**，互不耦合——这就是它们能独立 commit / 独立回滚 / 独立验证的原因。

---

## 6. 改造全景图（含 td）

```
                        ROS bag (D435i)
                              │
              ┌───────────────┴───────────────┐
              ▼                               ▼
   ORIGINAL pipeline                   SPARSE1 pipeline
              │                               │
        ┌─────┴──────┐               ┌────────┴────────┐
        │ KLT 21×21 │               │ td_pre_calibrator│  ← 离线估算 td
        │ 3-level   │               │   (grid+ternary)│
        └─────┬──────┘               └────────┬────────┘
              │                               ▼
              │                       sparse_align (λ=IMU)
              │                               │
              │                               ▼
              │                       sparse_align (λ=0)  → R_vision
              │                               │
              │                               ▼
              │                       chi2 自适应 KLT 窗
              │                       5×5 / 9×9 / 15×15 / 21×21
              │                               │
              └──────────────┬────────────────┘
                             ▼
                    vins_estimator
                  BA 用 td_init = 预标定 td
                             │
                             ▼
                       pose_graph
              ┌──────────────────────────────┐
              │  sparse rotation gate (NEW)  │  ← 拒绝可疑回环
              │           ↓                  │
              │  reprojection error gate     │
              │           ↓                  │
              │  4-DOF graph optimization    │
              └──────────────────────────────┘
                             │
                             ▼
                  /vins_estimator/odometry
                  /pose_graph/path
```

**一句话总结**：

- **td** 把 BA 的"在线学习 td"前置成"启动前预标定"
- **sparse geometric gate** 把回环候选的第一道过滤从纯几何扩展到"几何 + 旋转一致性"
- **sparse_align + 自适应 KLT** 把前端的搜索范围砍掉一半、金字塔深度砍掉 1/3

三个改动都**不进 BA 主优化**，但都能显著降低 BA 压力、缩短收敛迭代、并提升前端鲁棒性。
