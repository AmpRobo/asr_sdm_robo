import os
import sys

import yaml

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, LogInfo, OpaqueFunction, GroupAction,
)
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node, PushRosNamespace


def load_vins_config(vins_share, config_name='vins.yaml'):
    config_path = os.path.join(vins_share, 'config', config_name)
    with open(config_path, 'r', encoding='utf-8') as f:
        raw = yaml.safe_load(f)
    return raw['/**']['ros__parameters']


def _as_launch_bool(value):
    if isinstance(value, bool):
        return '1' if value else '0'
    if isinstance(value, str):
        return '1' if value.lower() in ('1', 'true', 'yes', 'on') else '0'
    return '1' if value else '0'


def resolve_config_path(config_pkg_path, path):
    if not path or path[0] == '/':
        return path
    return os.path.join(config_pkg_path, path)


def build_common_params(context, config_pkg_path, launch_config_cls, vins_config):
    params_file_arg = context.perform_substitution(launch_config_cls('params_file'))
    calib_file_arg = context.perform_substitution(launch_config_cls('calibration_file'))
    config_file_arg = context.perform_substitution(launch_config_cls('config_file'))
    enable_sparse_arg = context.perform_substitution(launch_config_cls('enable_sparse'))
    if enable_sparse_arg == '':
        enable_sparse = bool(vins_config.get('enable_sparse', False))
    else:
        enable_sparse = enable_sparse_arg == '1'

    config_file = config_file_arg or str(vins_config.get('config_file', ''))
    if config_file:
        common_params = [
            {
                'config_file': config_file,
                'config_pkg_share': config_pkg_path,
                'vins_folder': config_pkg_path,
            },
        ]
        return common_params, config_file, '(from config_file)', enable_sparse

    params_rel = params_file_arg or str(vins_config.get('params_file', ''))
    calib_rel = calib_file_arg or str(vins_config.get('calibration_file', ''))
    params_file = resolve_config_path(config_pkg_path, params_rel)
    calib_file = resolve_config_path(config_pkg_path, calib_rel)

    common_params = [
        params_file,
        {
            'config_pkg_share': config_pkg_path,
            'vins_folder': config_pkg_path,
            'camera_calibration_file': calib_file,
        },
    ]

    if enable_sparse:
        sparse_cfg = vins_config.get('sparse_align', {})
        if sparse_cfg:
            common_params.append(sparse_cfg)

    return common_params, params_file, calib_file, enable_sparse


def declare_vins_config_args(vins_config, launch_config_name='vins.yaml'):
    return [
        DeclareLaunchArgument(
            'vins_launch_config',
            default_value=launch_config_name,
            description=(
                'Launch-level config yaml under vins_estimator/config/ '
                '(e.g. vins.yaml, vins_d435i.yaml).'
            ),
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value='',
            description='ROS 2 runtime params yaml (relative to config_pkg unless absolute).',
        ),
        DeclareLaunchArgument(
            'calibration_file',
            default_value='',
            description='OpenCV camera calibration yaml (relative to config_pkg unless absolute).',
        ),
        DeclareLaunchArgument(
            'config_file',
            default_value=str(vins_config.get('config_file', '')),
            description=(
                'Legacy OpenCV config yaml. When set, overrides params_file and '
                'loads settings via cv::FileStorage.'
            ),
        ),
        DeclareLaunchArgument(
            'enable_sparse',
            default_value='',
            description=(
                '0/1 - override yaml and enable sparse alignment. Empty = use launch config yaml.'
            ),
        ),
        DeclareLaunchArgument(
            'enable_frame_correction',
            default_value='false',
            description='Apply rotation correction to odometry output for frame alignment.',
        ),
        DeclareLaunchArgument(
            'frame_correction_roll',
            default_value='0.0',
            description='Roll angle in degrees for frame correction.',
        ),
        DeclareLaunchArgument(
            'frame_correction_pitch',
            default_value='0.0',
            description='Pitch angle in degrees for frame correction.',
        ),
        DeclareLaunchArgument(
            'frame_correction_yaw',
            default_value='0.0',
            description='Yaw angle in degrees for frame correction.',
        ),
    ]


def _launch_setup(context, *args, **kwargs):
    vins_share = get_package_share_directory('vins_estimator')
    config_pkg_path = get_package_share_directory('config_pkg')
    launch_config_name = context.perform_substitution(
        LaunchConfiguration('vins_launch_config'))
    vins_config = load_vins_config(vins_share, launch_config_name)

    enable_frame_correction = context.perform_substitution(
        LaunchConfiguration('enable_frame_correction'))
    frame_correction_roll = context.perform_substitution(
        LaunchConfiguration('frame_correction_roll'))
    frame_correction_pitch = context.perform_substitution(
        LaunchConfiguration('frame_correction_pitch'))
    frame_correction_yaw = context.perform_substitution(
        LaunchConfiguration('frame_correction_yaw'))

    common_params, log_params, log_calib, enable_sparse = build_common_params(
        context, config_pkg_path, LaunchConfiguration, vins_config)

    return [
        LogInfo(msg=[f'[vins launch] launch config: {launch_config_name}']),
        LogInfo(msg=[f'[vins launch] params file: {log_params}']),
        LogInfo(msg=[f'[vins launch] calibration file: {log_calib}']),
        LogInfo(msg=[f'[vins launch] sparse alignment override: {"ON" if enable_sparse else "OFF"}']),
        LogInfo(msg=[f'[vins launch] frame correction: {"ON" if enable_frame_correction.lower() == "true" else "OFF"}']),
        _pipeline(
            config_pkg_path=config_pkg_path,
            common_params=common_params,
            vins_config=vins_config,
            enable_frame_correction=enable_frame_correction.lower() == "true",
            frame_correction_roll=float(frame_correction_roll or "0.0"),
            frame_correction_pitch=float(frame_correction_pitch or "0.0"),
            frame_correction_yaw=float(frame_correction_yaw or "0.0"),
        ),
    ]


def generate_launch_description():
    vins_share = get_package_share_directory('vins_estimator')
    default_launch_config = 'vins.yaml'
    vins_config = load_vins_config(vins_share, default_launch_config)

    return LaunchDescription(
        declare_vins_config_args(vins_config, default_launch_config)
        + [OpaqueFunction(function=_launch_setup)]
    )


def _pipeline(config_pkg_path, common_params, vins_config,
              enable_frame_correction=False,
              frame_correction_roll=0.0,
              frame_correction_pitch=0.0,
              frame_correction_yaw=0.0):
    """
    Return launch actions for the VINS pipeline.

    Pipeline: feature_tracker -> vins_estimator -> pose_graph.
    Visualization lives in logging_simulator (config/rviz.rviz).
    All nodes run under the namespace configured in config/vins.yaml.
    """
    ns = vins_config.get('namespace', 'localization/video_inertial_navigation_systems')
    pose_graph_cfg = vins_config.get('pose_graph', {})

    ft_remaps = [
        ('feature', f'/{ns}/feature'),
        ('feature_img', f'/{ns}/feature_img'),
        ('restart', f'/{ns}/restart'),
    ]

    ve_remaps = [
        (t, f'/{ns}/{t}') for t in [
            'camera_pose_visual', 'keyframe_point', 'path', 'odometry',
            'extrinsic', 'imu_propagate', 'keyframe_pose',
            'relo_relative_pose', 'key_poses', 'camera_pose',
            'point_cloud', 'history_cloud', 'relocalization_path',
        ]
    ]
    ve_sub_remaps = [
        ('/feature_tracker/feature', f'/{ns}/feature'),
        ('/feature_tracker/restart', f'/{ns}/restart'),
        ('/pose_graph/match_points', f'/{ns}/match_points'),
        ('/feature_tracker/sparse_rot', f'/{ns}/sparse_rot'),
        ('/feature_tracker/td_estimate', f'/{ns}/td_estimate'),
    ]
    ve_remaps = ve_sub_remaps + ve_remaps

    # Frame correction parameters for vins_estimator
    frame_correction_params = {}
    if enable_frame_correction:
        frame_correction_params = {
            'enable_frame_correction': True,
            'frame_correction_roll': frame_correction_roll,
            'frame_correction_pitch': frame_correction_pitch,
            'frame_correction_yaw': frame_correction_yaw,
        }

    pg_remaps = [
        (t, f'/{ns}/{t}') for t in [
            'match_image', 'camera_pose_visual', 'key_odometrys',
            'no_loop_path', 'match_points', 'pose_graph_path',
            'base_path', 'pose_graph',
        ]
    ]
    pg_sub_remaps = [
        ('/vins_estimator/imu_propagate', f'/{ns}/imu_propagate'),
        ('/vins_estimator/odometry', f'/{ns}/odometry'),
        ('/vins_estimator/keyframe_pose', f'/{ns}/keyframe_pose'),
        ('/vins_estimator/extrinsic', f'/{ns}/extrinsic'),
        ('/vins_estimator/keyframe_point', f'/{ns}/keyframe_point'),
        ('/vins_estimator/relo_relative_pose', f'/{ns}/relo_relative_pose'),
    ]
    pg_remaps = pg_sub_remaps + pg_remaps

    support_path = os.path.join(config_pkg_path, 'support_files')
    pose_graph_params = common_params + [{
        'support_file': support_path,
        'visualization_shift_x': pose_graph_cfg.get('visualization_shift_x', 0),
        'visualization_shift_y': pose_graph_cfg.get('visualization_shift_y', 0),
        'skip_cnt': pose_graph_cfg.get('skip_cnt', 0),
        'skip_dis': pose_graph_cfg.get('skip_dis', 0.0),
    }]

    ns_actions = [PushRosNamespace(part) for part in ns.split('/') if part]

    return GroupAction(ns_actions + [

        Node(
            package='feature_tracker',
            executable='feature_tracker_node',
            name='feature_tracker',
            output='screen',
            remappings=ft_remaps,
            parameters=common_params,
        ),

        Node(
            package='vins_estimator',
            executable='vins_estimator',
            name='vins_estimator',
            output='screen',
            remappings=ve_remaps,
            parameters=common_params + [frame_correction_params],
        ),

        Node(
            package='pose_graph',
            executable='pose_graph',
            name='pose_graph',
            output='screen',
            remappings=pg_remaps,
            parameters=pose_graph_params,
        ),
    ])
