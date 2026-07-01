#!/usr/bin/env python3
"""
ROS2 Launch file for SVO + tight-coupled VIO backend (Phase 1) on D435i bag.

This launch file extends d435i_vio.launch.py by enabling the svo_vio_backend
library (a VINS-Mono-style sliding-window Ceres optimizer) layered on top
of the SVO frontend. The SVO frontend still runs its IMU rotation prior
(SparseImgAlign) and the VINS-style backend is appended to refine poses.

It publishes the tight-coupled body-in-world pose to:
    /svo_vio/odometry   (nav_msgs/Odometry)

Usage:
  # Terminal 1: VIO node (frontend + tight backend) + RViz
  ros2 launch svo_ros svo_vio_tight.launch.py

  # Terminal 2: play bag
  ros2 bag play /path/to/d435if_20260530_080612_resized
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

    # D435i parameter file (camera intrinsics + IMU params + frontend VIO weights)
    d435i_yaml_path = os.path.join(svo_ros_dir, 'param', 'd435i_camera.yaml')

    # Topic arguments
    cam_topic_arg = DeclareLaunchArgument(
        'cam_topic', default_value='/sensing/camera/realsense/color/image_raw',
        description='Camera image topic (D435i bag publishes here directly)')

    imu_topic_arg = DeclareLaunchArgument(
        'imu_topic', default_value='/sensing/camera/realsense/imu',
        description='IMU topic (D435i bag publishes fused IMU here)')

    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='false',
        description='Launch RViz2 for visualization')

    rviz_sw_arg = DeclareLaunchArgument(
        'rviz_sw', default_value='true',
        description='Use software rendering (LIBGL_ALWAYS_SOFTWARE=1) for RViz')

    use_imu_arg = DeclareLaunchArgument(
        'use_imu', default_value='true',
        description='Enable IMU VIO')

    # VIO backend (Phase 1: tight-coupled sliding-window Ceres)
    enable_vio_backend_arg = DeclareLaunchArgument(
        'enable_vio_backend', default_value='true',
        description='Enable tight-coupled VIO backend (svo_vio_backend)')

    vio_odom_topic_arg = DeclareLaunchArgument(
        'vio_odom_topic', default_value='/svo_vio/odometry',
        description='Topic for tight-coupled VIO body-in-world pose')

    vio_max_iterations_arg = DeclareLaunchArgument(
        'vio_max_iterations', default_value='8',
        description='VIO backend Ceres max iterations')

    vio_solver_time_limit_arg = DeclareLaunchArgument(
        'vio_solver_time_limit', default_value='0.04',
        description='VIO backend Ceres time limit (s)')

    vio_optimize_period_arg = DeclareLaunchArgument(
        'vio_optimize_period', default_value='0.1',
        description='VIO backend optimization period (s)')

    # D435i BMI055 IMU noise (used by VioBackend)
    vio_acc_n_arg = DeclareLaunchArgument('vio_acc_n', default_value='0.012',
        description='Accelerometer noise density (m/s^2/sqrt(Hz))')
    vio_gyr_n_arg = DeclareLaunchArgument('vio_gyr_n', default_value='0.003',
        description='Gyroscope noise density (rad/s/sqrt(Hz))')
    vio_acc_w_arg = DeclareLaunchArgument('vio_acc_w', default_value='0.0004',
        description='Accelerometer bias random walk')
    vio_gyr_w_arg = DeclareLaunchArgument('vio_gyr_w', default_value='3.0e-5',
        description='Gyroscope bias random walk')

    # IMU-Camera extrinsics for VioBackend (ric, tic).
    # Default: identity (SVO's T_cam_imu=0 convention). Override from bag /tf_static
    # if non-identity extrinsics are needed. Format: row-major 9-vector for vio_ric.
    vio_ric_arg = DeclareLaunchArgument(
        'vio_ric', default_value='[1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]',
        description='IMU-Camera rotation matrix (row-major 9-vector)')

    vio_tic_arg = DeclareLaunchArgument(
        'vio_tic', default_value='[0.0, 0.0, 0.0]',
        description='IMU-Camera translation (3-vector)')

    # SVO frontend VIO weights (SparseImgAlign + PoseOptimizer)
    img_align_prior_lambda_rot_arg = DeclareLaunchArgument(
        'img_align_prior_lambda_rot', default_value='0.5')
    pose_optim_prior_lambda_rot_arg = DeclareLaunchArgument(
        'pose_optim_prior_lambda_rot', default_value='0.5')
    zero_motion_accel_std_thresh_arg = DeclareLaunchArgument(
        'zero_motion_accel_std_thresh', default_value='0.05')
    imu_preprocessing_mode_arg = DeclareLaunchArgument(
        'imu_preprocessing_mode', default_value='0',
        description='IMU preprocessing: 0=none (use raw IMU directly)')

    svo_node = Node(
        package='svo_ros',
        executable='vo',
        name='asr_sdm_video_inertial_odometry',
        output='screen',
        parameters=[
            d435i_yaml_path,
            {
                'cam_topic': LaunchConfiguration('cam_topic'),
                'imu_topic': LaunchConfiguration('imu_topic'),

                # Initial camera orientation: align with horizontal D435i
                'init_rx': -1.5707963267948966,
                'init_ry': 0.0,
                'init_rz': 0.0,

                # SVO frontend IMU
                'use_imu': LaunchConfiguration('use_imu'),
                'img_align_prior_lambda_rot':
                    LaunchConfiguration('img_align_prior_lambda_rot'),
                'pose_optim_prior_lambda_rot':
                    LaunchConfiguration('pose_optim_prior_lambda_rot'),
                'zero_motion_accel_std_thresh':
                    LaunchConfiguration('zero_motion_accel_std_thresh'),
                'imu_preprocessing_mode':
                    LaunchConfiguration('imu_preprocessing_mode'),

                # Tight-coupled VIO backend
                'enable_vio_backend': LaunchConfiguration('enable_vio_backend'),
                'vio_odom_topic':     LaunchConfiguration('vio_odom_topic'),
                'vio_max_iterations':  LaunchConfiguration('vio_max_iterations'),
                'vio_solver_time_limit': LaunchConfiguration('vio_solver_time_limit'),
                'vio_optimize_period': LaunchConfiguration('vio_optimize_period'),
                'vio_acc_n':           LaunchConfiguration('vio_acc_n'),
                'vio_gyr_n':           LaunchConfiguration('vio_gyr_n'),
                'vio_acc_w':           LaunchConfiguration('vio_acc_w'),
                'vio_gyr_w':           LaunchConfiguration('vio_gyr_w'),
                'vio_ric':             LaunchConfiguration('vio_ric'),
                'vio_tic':             LaunchConfiguration('vio_tic'),

                # Visualization
                'enable_visualization': True,
                'publish_markers': True,
                'publish_dense_input': False,
            },
        ],
    )

    rviz_config_path = os.path.join(svo_ros_dir, 'rviz_config.rviz')

    # Software rendering env vars
    env_libgl = SetEnvironmentVariable(
        name='LIBGL_ALWAYS_SOFTWARE', value='1',
        condition=IfCondition(LaunchConfiguration('rviz_sw')))
    env_gallium = SetEnvironmentVariable(
        name='GALLIUM_DRIVER', value='llvmpipe',
        condition=IfCondition(LaunchConfiguration('rviz_sw')))
    env_gl_version = SetEnvironmentVariable(
        name='MESA_GL_VERSION_OVERRIDE', value='3.0',
        condition=IfCondition(LaunchConfiguration('rviz_sw')))
    env_qt_gl = SetEnvironmentVariable(
        name='QT_XCB_FORCE_SOFTWARE_OPENGL', value='1',
        condition=IfCondition(LaunchConfiguration('rviz_sw')))

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        condition=IfCondition(LaunchConfiguration('rviz')),
        output='screen'
    )

    # Static TF: world -> camera
    world_to_cam_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world_to_cam_tf',
        arguments=[
            '0', '0', '0',
            '0', '0', '0',
            'world', 'camera',
        ],
        output='screen'
    )

    return LaunchDescription([
        # Args
        cam_topic_arg, imu_topic_arg, rviz_arg, rviz_sw_arg, use_imu_arg,
        enable_vio_backend_arg, vio_odom_topic_arg,
        vio_max_iterations_arg, vio_solver_time_limit_arg, vio_optimize_period_arg,
        vio_acc_n_arg, vio_gyr_n_arg, vio_acc_w_arg, vio_gyr_w_arg,
        vio_ric_arg, vio_tic_arg,
        img_align_prior_lambda_rot_arg, pose_optim_prior_lambda_rot_arg,
        zero_motion_accel_std_thresh_arg, imu_preprocessing_mode_arg,

        # Env
        env_libgl, env_gallium, env_gl_version, env_qt_gl,

        # Nodes
        svo_node,
        world_to_cam_tf,
        rviz_node,
    ])
