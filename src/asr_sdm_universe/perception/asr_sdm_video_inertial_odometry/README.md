# Installation

## Dependency
```sh
sudo apt-get -y install ros-$ROS_DISTRO-sophus libgoogle-glog-dev
```

## Build
```sh
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --symlink-install --parallel-workers 8
```

> **Note:** SVO is a compute-intensive visual odometry algorithm. Always use `Release` build for performance. Debug builds will run significantly slower.

# Run Instructions

### Run using ROS2 Jazzy



### Visual Odometry demo (rosbag + RViz2)
This section shows how to run the SVO visual odometry node with the included rosbag and visualize results in RViz2.

Prerequisites
`rm -rf build install log`
`source /opt/ros/$ROS_DISTRO/setup.bash`
- Workspace has been built: `colcon build --packages-up-to svo_ros --cmake-args -DCMAKE_BUILD_TYPE=Release`
- Source the workspace: `source install/setup.bash`
- The demo bag exists at: `datasheet/airground_rig_s3_ros2/airground_rig_s3_ros2.db3`
- RViz2 config file: `src/asr_sdm_universe/perception/asr_sdm_video_inertial_odometry/asr_sdm_svo/svo_ros/rviz_config.rviz`

Recommended: use three terminals and source the workspace in each
```sh
source install/setup.bash
```

Terminal 1 — start all nodes (camera + IMU + VIO) in one launch
```sh
# Camera node: subscribes bag /camera/image_raw, publishes /sensing/camera/realsense/color/*
ros2 launch svo_ros camera.launch.py

# IMU node: subscribes bag /imu/data, publishes raw + filtered
ros2 launch svo_ros imu.launch.py imu_topic_in:=/imu/data

# VIO node: subscribes camera + IMU filtered
ros2 launch svo_ros test_rig3.launch.py cam_topic:=/sensing/camera/realsense/color/image_raw imu_topic:=/sensing/imu/imu_filtered fast_type:=12

# Or use the combined launch (includes camera + IMU + VIO + RViz)
ros2 launch svo_ros test_rig3.launch.py rviz:=true fast_type:=12

# 仅纯视觉（不使用 IMU）
ros2 launch svo_ros d435i_vio.launch.py use_imu:=false fast_type:=12

# VIO 方案（IMU prior λ）
ros2 launch svo_ros vio_rig3.launch.py
# 方案A（推荐）：仅 SparseImgAlign 用 IMU prior
ros2 launch svo_ros test_rig3.launch.py use_imu:=true \
  img_align_prior_lambda_rot:=0.5 pose_optim_prior_lambda_rot:=0.0 fast_type:=12

# VIO 方案B：D435i 原始数据链路严格对齐（直接订阅 bag 原生话题）
ros2 launch svo_ros d435i_vio.launch.py use_imu:=true

# EuRoC
ros2 launch svo_ros test_euroc.launch.py
ros2 launch svo_ros vio_euroc.launch.py
```

## VIO Mode (Recommended: use vio_rig3.launch.py)

The `vio_rig3.launch.py` is the VIO launch file that mirrors the reference `euroc_vio_mono.launch` from rpg/svo_pro_open. It loads the complete VIO parameter set from `param/vio_rig3.yaml` which includes all EuRoC-tuned algorithm parameters, IMU calibration, IMU prior weights, local bundle adjustment, and zero-motion detection.

### VIO vs VO Mode

| Feature | VO (`test_rig3.launch.py`) | VIO (`vio_rig3.launch.py`) |
|---|---|---|
| IMU prior in SparseImgAlign | Optional (`img_align_prior_lambda_rot`) | Enabled by default (0.5) |
| IMU prior in PoseOptimizer | Disabled by default (0.0) | Enabled by default (0.5) |
| Local Bundle Adjustment | Disabled (`loba_num_iter=0`) | Enabled (`loba_num_iter=3`) |
| Backend pose optimization iterations | Default (10) | Default (10) |
| Core keyframes for BA | Default (3) | Default (3) |
| Pose reproj threshold | 2.0 px | 2.0 px |
| Automatic re-initialization | Not configurable | Enabled (from `vio_rig3.yaml`) |

### Quick Start

```sh
# Terminal 1: VIO (includes camera + IMU + VIO + RViz)
ros2 launch svo_ros vio_rig3.launch.py rviz:=true fast_type:=12
ros2 launch svo_ros d435i_vio.launch.py use_imu:=true
# Terminal 2: Play bag
ros2 bag play datasheet/airground_rig_s3_ros2
ros2 bag play datasheet/d435if_20260530_080612_resized 
```

### VIO Launch Options

```sh
# VIO A (recommended): SparseImgAlign + PoseOptimizer both use IMU prior
ros2 launch svo_ros vio_rig3.launch.py \
  img_align_prior_lambda_rot:=0.5 pose_optim_prior_lambda_rot:=0.5 fast_type:=12

# VIO B (conservative): Only SparseImgAlign uses IMU prior
ros2 launch svo_ros vio_rig3.launch.py \
  img_align_prior_lambda_rot:=0.5 pose_optim_prior_lambda_rot:=0.0 fast_type:=12

# VIO C (aggressive): Strong IMU weight in both stages
ros2 launch svo_ros vio_rig3.launch.py \
  img_align_prior_lambda_rot:=1.0 pose_optim_prior_lambda_rot:=1.0 fast_type:=12

# Pure visual odometry (no IMU)
ros2 launch svo_ros vio_rig3.launch.py use_imu:=false fast_type:=12

# VIO with IMU calibration (mode 2: use pre-calibrated IMU)
ros2 launch svo_ros vio_rig3.launch.py \
  imu_preprocessing_mode:=2 imu_calib_file:=$(pwd)/imu_calibration.yaml
```

### Key VIO Parameters

The following parameters can be tuned via command line or in `param/vio_rig3.yaml`:

**IMU Prior Weights (lambda):**
- `img_align_prior_lambda_rot` — Rotation prior weight in SparseImgAlign (direct photometric alignment). Range: 0.0 (pure visual) to 1.0 (strong IMU). Default: 0.5 (EuRoC tuned).
- `pose_optim_prior_lambda_rot` — Rotation prior weight in PoseOptimizer (Gauss-Newton over reprojected 3D-2D correspondences). Default: 0.5. Set to 0.0 to disable (pure visual BA).
- `img_align_prior_lambda_trans` — Translation prior weight. Default: 0.0 (pure visual for monocular cameras which lack metric scale).

**Zero-Motion Detection:**
- `zero_motion_accel_std_thresh` — Accelerometer standard deviation threshold (m/s^2). Below this threshold, the robot is considered stationary and IMU prior is skipped. Default: 0.05.

**Local Bundle Adjustment (g2o):**
- `loba_num_iter` — Number of Levenberg-Marquardt iterations per keyframe BA. Default: 3. Set to 0 to disable local BA (pure frontend VO).
- `core_n_kfs` — Number of closest keyframes included in local BA window. Default: 3.

**Pose Optimization:**
- `pose_optim_num_iter` — Number of iterations in the Gauss-Newton pose optimizer. Default: 10.
- `pose_optim_prior_lambda_trans` — Translation prior weight in PoseOptimizer. Default: 0.0.

### Architecture

The VIO pipeline adds these stages on top of the pure VO pipeline:

```
[Image + IMU] 
     |
     v
[Stage 1] SparseImgAlign (direct method)
     |  Uses IMU gyro to predict rotation between frames
     |  IMU prior: weighted information matrix regularization
     v
[Stage 2] Map Reprojection (feature matching)
     v
[Stage 3] PoseOptimizer (Gauss-Newton over 3D-2D)
     |  Optional IMU rotation prior (when lambda > 0)
     v
[Stage 4] Structure Optimization (3D point refinement)
     v
[Stage 5] Keyframe Decision
     |  New keyframe if camera moved > kfselect_mindist relative to depth
     v
[Stage 6] Local Bundle Adjustment (g2o, when loba_num_iter > 0)
     |  Optimizes pose of core_n_kfs closest keyframes
     |  Refines 3D point positions
     v
[Stage 7] Depth Filter Update (background thread)
     |  Probabilistic depth estimation for candidate points
     v
[Output] Pose + Trajectory + Map Points
```

### Parameter File Reference

`param/vio_rig3.yaml` — Complete VIO parameters (EuRoC-tuned, adapted for Rig3):
- Camera intrinsics (D405 pinhole model)
- IMU noise parameters (sigma_omega_c, sigma_acc_c)
- IMU prior lambda weights
- Local BA settings (loba_num_iter, core_n_kfs)
- Zero-motion detection thresholds
- Keyframe selection criteria

`param/calib/rig3_mono.yaml` — IMU extrinsics in EuRoC YAML format:
- T_B_C: Camera-to-IMU transform (identity for Rig3)
- IMU noise parameters
- Initial bias and velocity estimates

`param/imu_rig3_calib.yaml` — IMU calibration output file:
- Auto-generated when running offline calibration (`c` key in terminal)
- Contains estimated gyro_bias, acc_bias, scale
- Loaded back with `l` key or via `imu_calib_file` parameter

### Keyboard Controls (while running)

| Key | Action |
|-----|--------|
| `q` | Quit |
| `r` | Reset SVO (clear map, restart initialization) |
| `s` | Start SVO (resume from paused) |
| `c` | Run offline IMU calibration (requires `imu_preprocessing_mode:=1`) |
| `l` | Load IMU calibration from file (requires `imu_calib_file` param) |

Terminal 2 — play rosbag (topics match bag defaults)
```sh
ros2 launch svo_ros test_rig3.launch.py use_imu:=true \
  img_align_prior_lambda_rot:=0.0 pose_optim_prior_lambda_rot:=0.0 fast_type:=12
```

Terminal 3 — start RViz2 with the provided config and use simulated time
```sh
rviz2 -d src/asr_sdm_universe/perception/asr_sdm_video_inertial_odometry/asr_sdm_svo/svo_ros/rviz_config.rviz 
```

What you should see
- Nodes: `/asr_sdm_camera_realsense_d405`, `/asr_sdm_imu_hiwonder_10axis`, `/asr_sdm_video_inertial_odometry`, `/rosbag2_player`
- Topics: `/camera/image_raw`, `/camera/camera_info`, `/imu/data`, `/sensing/camera/realsense/color/image_raw`, `/sensing/camera/realsense/color/camera_info`, `/sensing/imu/imu_raw`, `/sensing/imu/imu_filtered`, `/localization/video_inertial_odom/image`, `/localization/video_inertial_odom/keyframes`, `/localization/video_inertial_odom/points`, `/localization/video_inertial_odom/odom`, `/localization/video_inertial_odom/trajectory`, `/tf`, `/clock`

Quick checks
```sh
# list nodes
ros2 node list

# list topics
ros2 topic list | sort

# Camera node info
ros2 node info /asr_sdm_camera_realsense_d405

# IMU node info (publishers)
ros2 node info /asr_sdm_imu_hiwonder_10axis

# check camera publishing
ros2 topic hz /sensing/camera/realsense/color/image_raw

# check IMU publishing
ros2 topic hz /sensing/imu/imu_raw
ros2 topic hz /sensing/imu/imu_filtered

# SVO node info (publishers/subscribers)
ros2 node info /asr_sdm_video_inertial_odometry

# check VIO publishing rates
ros2 topic hz /localization/video_inertial_odom/image
ros2 topic hz /localization/video_inertial_odom/odom
```

Stop processes
```sh
# stop camera node
pkill -f "ros2 launch svo_ros camera.launch.py"

# stop IMU node
pkill -f "ros2 launch svo_ros imu.launch.py"

# stop VIO launch
pkill -f "ros2 launch svo_ros test_rig3.launch.py"

# stop VIO launch (new VIO launch file)
pkill -f "ros2 launch svo_ros vio_rig3.launch.py"

# stop rosbag player
pkill -f "ros2 bag play"

# stop RViz2
pkill -f "rviz2 -d .*rviz_config.rviz"
```

Notes
- The camera node (`asr_sdm_camera_realsense_d405`) subscribes to bag topics (`/camera/image_raw`, `/camera/camera_info`) and republishes to the standardized sensing namespace (`/sensing/camera/realsense/color/*`). VIO node and downstream modules subscribe to the standardized topics.
- The IMU node (`asr_sdm_imu_hiwonder_10axis`) subscribes to `/imu/data` and republishes to `/sensing/imu/imu_raw` (raw pass-through) and `/sensing/imu/imu_filtered` (filtered output, ready for VIO integration).
- The VIO node subscribes to `/sensing/camera/realsense/color/image_raw` and `/sensing/imu/imu_filtered`. Since the combined launch starts both camera and IMU nodes automatically, the data pipeline is wired up correctly with no manual remapping needed.
- If you prefer a single combined launch that starts everything together, use `test_rig3.launch.py` directly (it includes camera + IMU + VIO + RViz).
- For the full VIO experience with local bundle adjustment and optimized IMU prior weights, use `vio_rig3.launch.py` (see the VIO Mode section above).
- The rosbag can be played with `--clock` for simulated time. RViz2 is started with `use_sim_time:=true` so the timeline follows the bag.
