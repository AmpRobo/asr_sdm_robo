#!/usr/bin/env python3
"""
ROS2 Launch file for SVO VIO with Intel RealSense D435i.

Adapted from vio_rig3.launch.py for the D435i rosbag recorded on 2026-05-30.
D435i bag publishes camera/IMU topics directly under the standard sensing namespace,
so the camera/IMU republisher nodes from vio_rig3.launch.py are not needed here.

Key differences from vio_rig3.launch.py:
  - No camera/imu republisher nodes (bag topics already in correct namespace)
  - Loads d435i_mono.yaml (D435i 640x480 intrinsics) instead of vio_rig3.yaml
  - Loads calib/d435i_mono.yaml (D435i IMU extrinsics)
  - IMU topic: /sensing/camera/realsense/imu (bag-published, not from IMU node)
  - Initial camera pose: horizontal (init_rx=0.0) for D435i pointing forward

Usage:
  # Terminal 1: VIO node + RViz
  ros2 launch svo_ros d435i_vio.launch.py

  # Terminal 2: Play bag
  ros2 bag play /path/to/d435if_20260530_080612_resized

  # Or with RViz
  ros2 launch svo_ros d435i_vio.launch.py rviz:=true

  # Pure VO (no IMU)
  ros2 launch svo_ros d435i_vio.launch.py use_imu:=false
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition


def generate_launch_description():
    svo_ros_dir = get_package_share_directory('svo_ros')

    # D435i parameter file (includes camera intrinsics, IMU params, VIO weights, T_cam_imu)
    d435i_yaml_path = os.path.join(svo_ros_dir, 'param', 'd435i_camera.yaml')

    # Topic arguments
    # D435i bag publishes directly to these topics — no republisher needed
    cam_topic_arg = DeclareLaunchArgument(
        'cam_topic',
        default_value='/sensing/camera/realsense/color/image_raw',
        description='Camera image topic (D435i bag publishes here directly)')

    imu_topic_arg = DeclareLaunchArgument(
        'imu_topic',
        default_value='/sensing/camera/realsense/imu',
        description='IMU topic (D435i bag publishes fused IMU here)')

    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='false',
        description='Launch RViz2 for visualization')

    rviz_sw_arg = DeclareLaunchArgument(
        'rviz_sw',
        default_value='true',
        description='Use software rendering (LIBGL_ALWAYS_SOFTWARE=1) for RViz')

    use_imu_arg = DeclareLaunchArgument(
        'use_imu',
        default_value='true',
        description='Enable IMU VIO (SparseImgAlign + PoseOptimizer prior)')

    # VIO-specific: IMU prior lambda for SparseImgAlign (frontend)
    img_align_prior_lambda_rot_arg = DeclareLaunchArgument(
        'img_align_prior_lambda_rot',
        default_value='0.5',
        description='IMU rotation prior weight in SparseImgAlign (0.5=rpg/vio_mono.yaml recommended)')

    img_align_prior_lambda_trans_arg = DeclareLaunchArgument(
        'img_align_prior_lambda_trans',
        default_value='0.0',
        description='IMU translation prior weight in SparseImgAlign (0=pure visual for mono)')

    # VIO-specific: IMU prior lambda for PoseOptimizer (backend BA stage)
    pose_optim_prior_lambda_rot_arg = DeclareLaunchArgument(
        'pose_optim_prior_lambda_rot',
        default_value='0.0',
        description='IMU rotation prior weight in PoseOptimizer (0=disabled, 0.5=recommended)')

    pose_optim_prior_lambda_trans_arg = DeclareLaunchArgument(
        'pose_optim_prior_lambda_trans',
        default_value='0.0',
        description='IMU translation prior weight in PoseOptimizer (0=pure visual)')

    # VIO-specific: zero-motion detection
    zero_motion_accel_std_thresh_arg = DeclareLaunchArgument(
        'zero_motion_accel_std_thresh',
        default_value='0.05',
        description='Zero-motion accel std thresh (m/s^2); below=stationary')

    # VIO-specific: IMU preprocessing mode
    imu_preprocessing_mode_arg = DeclareLaunchArgument(
        'imu_preprocessing_mode',
        default_value='1',
        description='IMU preprocessing: 0=none, 1=collect+calibrate, 2=use calibration')

    # Phase 2: VINS-compatible feature cloud (SVO -> vins_estimator)
    publish_vins_features_arg = DeclareLaunchArgument(
        'publish_vins_features',
        default_value='true',
        description='Emit sensor_msgs/PointCloud on vins_feature_topic for '
                    'vins_estimator to consume (replaces svo_vio_backend)')
    vins_feature_topic_arg = DeclareLaunchArgument(
        'vins_feature_topic',
        default_value='/feature_tracker/feature',
        description='Topic name for VINS-compatible feature cloud '
                    '(default matches what stock vins_estimator subscribes to)')
    enable_vins_estimator_arg = DeclareLaunchArgument(
        'enable_vins_estimator',
        default_value='true',
        description='Spawn stock vins_estimator node (Phase 2 VIO back-end)')
    enable_vio_backend_arg = DeclareLaunchArgument(
        'enable_vio_backend',
        default_value='false',
        description='Old in-house svo_vio_backend (Phase 1, kept for A/B testing)')

    # SVO VIO node
    # Loads d435i_camera.yaml which includes:
    #   - Camera intrinsics (Pinhole, 640x480, D435i RGB params)
    #   - IMU calibration params (D435i BMI055 noise specs)
    #   - Camera-IMU extrinsics T_cam_imu (extracted from bag /tf_static)
    #   - VIO algorithm params (IMU prior weights, zero-motion, etc.)
    #
    # NOTE: D435i bag publishes camera and IMU topics directly to the
    # standard sensing namespace. No camera/IMU republisher nodes are needed.
    svo_node = Node(
        package='svo_ros',
        executable='vo',
        name='asr_sdm_video_inertial_odometry',
        output='screen',
        parameters=[
            d435i_yaml_path,
            {
                # Topic remapping (bag already publishes to these topics)
                'cam_topic': LaunchConfiguration('cam_topic'),
                'imu_topic': LaunchConfiguration('imu_topic'),

                # NOTE: Do NOT enable use_sim_time for bags with absolute timestamps.
                # The bag uses real Unix timestamps (2026-05-30). Enabling --clock
                # with simulated time would use the bag clock, which should work
                # but requires careful synchronization. By default we use real time.

                # Initial camera orientation correction:
                # Bag IMU data: ay=-9.74 → IMU Y-axis ≈ downward.
                # With T_cam_imu ≈ identity, camera optical Y ≈ IMU Y ≈ down.
                # After getInitialAttitude(): camera optical Z ≈ world +Y (tilted upward).
                # To make camera optical Z = world +X (horizontal forward):
                #   R_cam_world needs Rx(-π/2) additional rotation.
                # Verification: Rx(-π/2) @ camera_Z_in_world ≈ world +X ✓
                #              Rx(-π/2) @ camera_Y_in_world ≈ world +Z (up) ✓
                #              Rx(-π/2) @ world +Z = camera_Y (down) ✓
                'init_rx': -1.5707963267948966,
                'init_ry': 0.0,
                'init_rz': 0.0,

                # VIO: IMU enable
                'use_imu': LaunchConfiguration('use_imu'),

                # VIO: SparseImgAlign prior weights (frontend)
                'img_align_prior_lambda_rot': LaunchConfiguration('img_align_prior_lambda_rot'),
                'img_align_prior_lambda_trans': LaunchConfiguration('img_align_prior_lambda_trans'),

                # VIO: PoseOptimizer prior weights (backend BA stage)
                'pose_optim_prior_lambda_rot': LaunchConfiguration('pose_optim_prior_lambda_rot'),
                'pose_optim_prior_lambda_trans': LaunchConfiguration('pose_optim_prior_lambda_trans'),

                # VIO: zero-motion detection
                'zero_motion_accel_std_thresh': LaunchConfiguration('zero_motion_accel_std_thresh'),

                # VIO: IMU preprocessing
                'imu_preprocessing_mode': LaunchConfiguration('imu_preprocessing_mode'),

                # VINS-compatible feature cloud (Phase 2: SVO→vins_estimator)
                'publish_vins_features': LaunchConfiguration('publish_vins_features'),
                'vins_feature_topic': LaunchConfiguration('vins_feature_topic'),

                # Disable the old in-house svo_vio_backend (replaced by vins_estimator)
                'enable_vio_backend': LaunchConfiguration('enable_vio_backend'),

                # Visualization
                'enable_visualization': True,
                'publish_markers': True,
                'publish_dense_input': False,
                'publish_every_nth_img': 1,
                'publish_every_nth_dense_input': 4,
                'publish_map_every_frame': False,
            },
        ],
    )

    # ====================================================================
    # Stock VINS estimator (Phase 2: replaces svo_vio_backend)
    # ====================================================================
    # The SVO node above emits a VINS-compatible feature cloud on
    # `vins_feature_topic` (default `/feature_tracker/feature`). This
    # stock `vins_estimator` consumes it and runs the well-tested
    # VINS sliding-window Ceres back-end.
    #
    # IMU topic is the same as SVO's. Camera intrinsics + IMU extrinsics
    # are read from `realsense_d435i_config.yaml` (VINS config) which is
    # already kept consistent with SVO's `d435i_camera.yaml`.
    config_pkg_path = os.path.join(
        os.path.dirname(get_package_share_directory('svo_ros')),
        '..', '..', '..', '..', 'asr_sdm_universe', 'perception',
        'asr_sdm_video_inertial_navigation_systems', 'config_pkg')
    # Fallback: derive from get_package_share_directory of config_pkg directly
    try:
        config_pkg_share = get_package_share_directory('config_pkg')
    except Exception:
        config_pkg_share = config_pkg_path
    vins_config_path = os.path.join(
        config_pkg_share, 'config', 'realsense', 'realsense_d435i_config.yaml')

    vins_estimator_node = Node(
        package='vins_estimator',
        executable='vins_estimator',
        name='vins_estimator',
        namespace='vins_estimator',
        output='screen',
        parameters=[{
            'config_file': vins_config_path,
            # SVO publishes to default `/feature_tracker/feature`; we
            # do NOT spawn the stock feature_tracker node (it would
            # compete with SVO for the image stream).
        }],
        condition=IfCondition(LaunchConfiguration('enable_vins_estimator')),
    )

    # RViz2 config path
    rviz_config_path = os.path.join(svo_ros_dir, 'rviz_config.rviz')

    # Software rendering env vars (for systems without GPU or VM)
    env_libgl = SetEnvironmentVariable(
        name='LIBGL_ALWAYS_SOFTWARE',
        value='1',
        condition=IfCondition(LaunchConfiguration('rviz_sw')))
    env_gallium = SetEnvironmentVariable(
        name='GALLIUM_DRIVER',
        value='llvmpipe',
        condition=IfCondition(LaunchConfiguration('rviz_sw')))
    env_gl_version = SetEnvironmentVariable(
        name='MESA_GL_VERSION_OVERRIDE',
        value='3.0',
        condition=IfCondition(LaunchConfiguration('rviz_sw')))
    env_qt_gl = SetEnvironmentVariable(
        name='QT_XCB_FORCE_SOFTWARE_OPENGL',
        value='1',
        condition=IfCondition(LaunchConfiguration('rviz_sw')))

    # RViz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        condition=IfCondition(LaunchConfiguration('rviz')),
        output='screen'
    )

    # Static TF: world -> camera
    # Matches the euroc_vio_mono.launch convention (world at first camera position).
    # For a horizontal D435i: initial pose is identity, world frame = camera init.
    world_to_cam_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world_to_cam_tf',
        arguments=[
            '0', '0', '0',   # x y z
            '0', '0', '0',  # roll pitch yaw
            'world',        # parent frame
            'camera',       # child frame (matches bag /camera/* topics)
        ],
        output='screen'
    )

    return LaunchDescription([
        # Launch arguments
        cam_topic_arg,
        imu_topic_arg,
        rviz_arg,
        rviz_sw_arg,
        use_imu_arg,
        img_align_prior_lambda_rot_arg,
        img_align_prior_lambda_trans_arg,
        pose_optim_prior_lambda_rot_arg,
        pose_optim_prior_lambda_trans_arg,
        zero_motion_accel_std_thresh_arg,
        imu_preprocessing_mode_arg,
        publish_vins_features_arg,
        vins_feature_topic_arg,
        enable_vins_estimator_arg,
        enable_vio_backend_arg,

        # Environment (RViz software rendering)
        env_libgl,
        env_gallium,
        env_gl_version,
        env_qt_gl,

        # Nodes: SVO front-end (publishes VINS features) + VINS back-end
        # Note: stock feature_tracker node is NOT spawned — SVO publishes
        # the VINS-compatible feature cloud itself.
        svo_node,
        vins_estimator_node,
        world_to_cam_tf,
        rviz_node,
    ])
