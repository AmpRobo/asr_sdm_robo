# VINS-MONO-ROS2
## ROS2 version of VINS-MONO
**New: Code has been adapted for Ubuntu 24.04. See the ros2_jazzy branch for details.**
# 1. Introduction
This repository implements the ROS2 version of VINS-MONO, mainly including the following packages:
* **camera_model**
* **feature_tracker**
* **vins_estimator**
* **pose_graph**
* **benchmark_pubilsher**
* **ar_demo**
* **config_pkg**

**NOTE**: Since the **_get_package_share_directory_** command in ROS2 launch files can only locate packages in the _install_ directory instead of the _src_ directory like ROS1, we create a package called **_config_pkg_** to store the _config/_ and _support_files/_ folders from VINS-MONO.
 
![mh01](https://github.com/dongbo19/VINS-MONO-ROS2/blob/main/config_pkg/config/gif/vins_ros2_mh01.gif)
![mh02](https://github.com/dongbo19/VINS-MONO-ROS2/blob/main/config_pkg/config/gif/vins_ros2_mh02.gif)
# 2. Prerequisites
* System  
  * Ubuntu 20.04  
  * ROS2 foxy
* Libraries
  * OpenCV 4.2.0
  * [Ceres Solver](http://ceres-solver.org/installation.html) 1.14.0
  * Eigen 3.3.7
# 3. Build VINS-MONO-ROS2
Clone the repository and colcon build:  
```
cd $(PATH_TO_YOUR_ROS2_WS)/src
git clone https://github.com/dongbo19/VINS-MONO-ROS2.git
cd ..
colcon build
```
# 4. VINS-MONO-ROS2 on EuRoC datasets
## 4.1. ROS1 bag to ROS2 bag
Download [EuRoC datasets](https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets). However, the datasets are in ROS1 format. To run the code in ROS2, we need to first convert these datasets to ROS2 format. We can use [rosbags](https://pypi.org/project/rosbags/) for this purpose, which can convert ROS built-in messages between ROS1 and ROS2.  
## 4.2. Visual-inertial odometry and loop closure
All configuration files are in the package, **_config_pkg_**, so in launch files, the path to the EuRoC configuration files is found using **_get_package_share_directory('config_pkg')_**.  
Open three terminals, launch the feature_tracker, vins_estimator, rviz2, and ros2 bag. Take the MH01 for example
```
ros2 launch feature_tracker vins_feature_tracker.launch.py              # for feature tracking and rviz2
ros2 launch vins_estimator euroc.launch.py                              # for backend optimization and loop closure
ros2 bag play $(PATH_TO_YOUR_DATASET)/MH_01_easy                        # for ros2 bag
```
![mh05](https://github.com/dongbo19/VINS-MONO-ROS2/blob/main/config_pkg/config/gif/vins_ros2_mh05.gif)
![v101](https://github.com/dongbo19/VINS-MONO-ROS2/blob/main/config_pkg/config/gif/vins_ros2_v101.gif)
## 4.3. Visualize ground truch
First, take the MH01 for example, modifying the **'sequence_name'** in the launch file: 
**_benchmark_publisher/launch/benchmark_publisher.launch.py_**
```
sequence_name_arg = DeclareLaunchArgument(
    'sequence_name',
    default_value='MH_01_easy',
    description='Sequence name for the benchmark'
)
sequence_name = LaunchConfiguration('sequence_name')
```
**PS: After modifying the launch file, don't forget to run **_colcon build_** for this package again.**  
Then, open four terminals, launch the feature_tracker, vins_estimator, benchmark_mark, rviz2, and ros2 bag.
```
ros2 launch feature_tracker vins_feature_tracker.launch.py            # for feature tracking and rviz2
ros2 launch vins_estimator euroc.launch.py                            # for backend optimization and loop closure
ros2 launch benchmark_publisher benchmark_publisher.launch.py         # for benchmark
ros2 bag play $(PATH_TO_YOUR_DATASET)/MH_01_easy                      # for ros2 bag
```
![mh01_benchmark](https://github.com/dongbo19/VINS-MONO-ROS2/blob/main/config_pkg/config/gif/vins_ros2_benchmark_mh01.gif)
![mh02_benchmark](https://github.com/dongbo19/VINS-MONO-ROS2/blob/main/config_pkg/config/gif/vins_ros2_benchmark_mh02.gif)
## 4.4. AR Demo
Download the [bag file](https://www.dropbox.com/scl/fi/q18lot4bfs1fqrctclz7b/ar_box.bag?rlkey=16yrxnwnt2fcutwwzwhlevd1n&e=1&dl=0).  
Then open two terminals  
```
ros2 launch ar_demo 3dm_bag.launch.py               # for featuer tracking, backend optimization, ar demo and rviz2.
ros2 bag play $(PATH_TO_YOUR_DATASET)/ar_box        # for ros2 bag
```
![ar_demo](https://github.com/dongbo19/VINS-MONO-ROS2/blob/main/config_pkg/config/gif/vins_ros2_ar_demo.gif)
# 5. Run your own datasets
If you need to run your own collected datasets, please add your configuration files to the _config_pkg/config_ directory, and then modify the **_config_path_** in the launch files mentioned above to find your configuration file:  
```
config_path = PathJoinSubstitution([
    config_pkg_path,
    'config/$(YOUR_YAML_FILE)'
])
```
**PS: After modifying the launch files or config files, don't forget to run **_colcon build_** for those packages again.**  
# 6. Acknowledgements
We use ros1 version of [VINS MONO](https://github.com/HKUST-Aerial-Robotics/VINS-Mono),  [ceres solver](http://ceres-solver.org/installation.html) for non-linear optimization, [DBoW2](https://github.com/dorian3d/DBoW2) for loop detection, and a generic [camera model](https://github.com/hengli/camodocal). Also, we referred to parts of the implementations from [VINS-FUSION-ROS2](https://github.com/zinuok/VINS-Fusion-ROS2) and [vins-mono-ros2](https://github.com/hitzzq/vins-mono-ros2).

# 7. Licence
The source code is released under [GPLv3](https://www.gnu.org/licenses/) license.

# 8 . D435i Demo
cd /home/lxy/asr_sdm_robo && colcon build --packages-up-to camera_model feature_tracker vins_estimator pose_graph benchmark_publisher ar_demo config_pkg


colcon build --packages-select asr_sdm_video_inertial_navigation_systems
ros2 launch vins_estimator d435i_combined.launch.py
ros2 bag play datasheet/d435if_20260530_080612_resized

# 9. SVO-style 稀疏前端改造（Sparse-prior KLT）

> 在 `feature_tracker` 前端引入 SVO 的稀疏图像对齐（semi-direct photometric alignment）
> 思想，让 KLT 用更小的搜索窗/金字塔就跑得动，从而**降低前端的算力消耗并提高
> 在快速运动 / 低纹理场景下的鲁棒性**。后端 VINS-Mono 估计器**完全未改动**——
> 改造成果只在 `feature_tracker` 内部可见。

## 9.1 目标

- 用 photometric alignment（Gauss-Newton + 4-level half-sampled pyramid）
  估计**帧间旋转 R**，并根据其拟合质量**自适应地缩小 KLT 搜索窗**。
- KLT 起点位置**不变**（仍用上一帧的 `cur_pts`），仅缩小 `winSize` 和 `maxLevel`。
- 完全保留 IMU 的帧间 R 先验在 VINS estimator 内部的主导地位——本改造只
  改前端跟踪成本，**不影响 VINS 的 IMU/Vision 紧耦合**。
- 后端 VINS estimator 收到的特征点仍然是 KLT sub-pixel 输出，BA cost 行为
  与原始 VINS 一致。

## 9.2 改造流程

原版（`feature_tracker.cpp`）每帧 KLT 用 21×21 搜索窗、3-level 金字塔：

```cpp
cv::calcOpticalFlowPyrLK(cur_img, forw_img, cur_pts, forw_pts,
                          status, err, cv::Size(21, 21), 3);
```

改造后：

```
1. sparse align 跑在前
   - 输入：prev_img 4-level pyramid (复用上一帧的) + cur_img pyramid +
          cur_pts (前帧像素) + IMU 帧间 R (R_prev_cur_) + fx/fy/cx/cy
   - 输出：R_k_kminus1, t_k_kminus1, final_chi2
   - 复用 prev_pyr_：跨帧只 buildHalfSamplePyramid 一次 (cur 那张)
   - 默认参数：patch_size=2, max_level=2, max_iter=4, chi2_thresh=50
2. 根据 final_chi2 决定 KLT 搜索窗
   - chi2 < 5   → 5×5, 1 level
   - chi2 < 15  → 9×9, 1 level
   - chi2 < 50  → 15×15, 2 levels
   - sparse fail → fallback 到 21×21, 3 levels（原版）
3. KLT 起点仍是 cur_pts，搜索窗变小 → sub-pixel 输出和原版一致
```

## 9.3 改动文件

| 文件 | 改动 |
|---|---|
| `feature_tracker/src/feature_tracker.h` | 新增 `saved_prev_pyr_`、`R_prev_cur_`、IMU preintegrator、KLT/sparse 统计字段 |
| `feature_tracker/src/feature_tracker.cpp` | 调换顺序：sparse align 在 KLT 之前；复用 prev pyramid；按 chi2 选 win/level；周期性 INFO 统计 |
| `feature_tracker/src/sparse_img_align.*` | 新增（半直接 photometric alignment，与 SVO 一致） |
| `feature_tracker/src/imu_preint.*` | 新增（极简 IMU 帧间 R 预积分） |
| `vins_estimator/launch/d435i_combined.launch.py` | 新增 `enable_sparse1:=1` 参数：起两条 VINS 链路（ORIGINAL 和 SPARSE1），共享同一个 bag input |

## 9.4 启动方法

最简：一键起两条链路（ORIGINAL vs SPARSE1），共用 bag。

```bash
cd /home/lxy/asr_sdm_robo
source /opt/ros/jazzy/setup.bash
source install/setup.bash

# 1) build —— 首次构建需要把 VINS 完整链都编一遍。
# 单独 build feature_tracker 是不够的，d435i_combined.launch.py
# 会通过 ament_index 找 config_pkg/vins_estimator/pose_graph 的
# install 目录（不能用 src/），缺一个就报
# "package 'config_pkg' not found" 然后 launch 整个不启。
#
# a) 增量：只重建本次改了源码的包（已经 build 过全链时用这个）
colcon build --packages-select feature_tracker

# b) 干净环境 / 第一次构建：把 VINS 完整链全部 build
colcon build --packages-up-to camera_model feature_tracker \
                                 vins_estimator pose_graph \
                                 benchmark_publisher ar_demo config_pkg

# 2) launch（同时跑 ORIGINAL + SPARSE1，对比用）
ros2 launch src/asr_sdm_universe/perception/asr_sdm_video_inertial_navigation_systems/vins_estimator/launch/d435i_combined.launch.py enable_sparse1:=1

# 3) 另起一个终端跑 bag
ros2 bag play datasheet/d435if_20260530_080612_resized/d435if_20260530_080612_resized_0.mcap --rate 1.0
```

输出分别落在 `/home/lxy/output/original/` 和 `/home/lxy/output/sparse1/`，便于对比轨迹 csv。

只跑 ORIGINAL（关闭 SPARSE1）：

```bash
ros2 launch src/asr_sdm_universe/perception/asr_sdm_video_inertial_navigation_systems/vins_estimator/launch/d435i_combined.launch.py enable_sparse1:=0
```

只跑 SPARSE1（ORIGINAL 不起，节省资源）：

```bash
ros2 launch src/asr_sdm_universe/perception/asr_sdm_video_inertial_navigation_systems/vins_estimator/launch/d435i_combined.launch.py enable_sparse1:=1 skip_original:=1
```

## 9.5 效果（95s D435i bag，1x 速率）

| 指标 | ORIGINAL | SPARSE1 | 变化 |
|---|---|---|---|
| **KLT mean cost** | 1.09ms | **0.89ms** | **-18%** |
| **KLT 搜索窗** | 21×21 | 14.4×14.4 | **-53%** |
| **KLT 金字塔** | 3.0 level | 1.91 level | **-36%** |
| sparse pipeline cost | – | 0.97ms | +0.97ms |
| sparse 成功率 | – | **100% (2920/2920)** | |
| sparse mean_chi2 | – | 17.77 | |
| sparse mean_nmeas | – | 537 像素/帧 | |
| **VINS 轨迹** | baseline | 路径长度差 0.13%、端到端位移差 0.4% | 一致 |
| **VINS init** | 1 次成功 | 1 次成功 | |
| VINS reboot / big bias | 0 | 0 | |

**总账**（慢速纹理丰富场景下）：KLT 节省 0.20ms，sparse 自身 +0.97ms，**总 +0.77ms/帧（慢 70%）**。
但在**快速运动**或**低纹理**场景下，sparse 提供的 R 先验会让 KLT 起点更接近真值、
不再需要 21 像素大窗搜索，**速度提升和鲁棒性收益才会体现**（这部分用 0.5x/2x bag
速率可以进一步验证）。

**总结**：本改造在**保持 VINS-Mono 后端完整一致**的前提下，把前端 KLT 的搜索范围
砍掉一半、深度砍掉 1/3；sparse 自身额外 ~1ms/帧的代价换来的是更稳的 KLT 起点
预测，是为后续在快速运动场景下使用更激进 KLT 窗、或者把 sparse R publish 给
VINS 当帧间 R 先验（方向 B）打基础的中间步骤。