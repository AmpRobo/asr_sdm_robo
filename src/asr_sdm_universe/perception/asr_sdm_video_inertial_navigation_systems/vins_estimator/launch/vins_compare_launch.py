"""Launch sparse_on and sparse_off pipelines simultaneously for comparison.

Two independent VINS pipelines, each listening on its own topic namespace:
  sparse_off: /sparse_off/cam0/image_raw, /sparse_off/imu0
  sparse_on:  /sparse_on/cam0/image_raw, /sparse_on/imu0

Bag play should be started separately with --remap to each namespace:
  ros2 bag play <bag> --remap /cam0/image_raw:=/sparse_off/cam0/image_raw /imu0:=/sparse_off/imu0
  ros2 bag play <bag> --remap /cam0/image_raw:=/sparse_on/cam0/image_raw  /imu0:=/sparse_on/imu0
"""

import os

from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import Node, PushRosNamespace
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def build_pipelines(vins_share, config_pkg_share):
    """Build both sparse_off and sparse_on pipeline actions."""
    pipelines = []

    for mode, launch_config in [("sparse_off", "vins.yaml"), ("sparse_on", "vins_spa.yaml")]:
        import yaml
        config_path = os.path.join(vins_share, 'config', launch_config)
        with open(config_path, 'r') as f:
            launch_cfg = yaml.safe_load(f)['/**']['ros__parameters']

        params_file = os.path.join(
            config_pkg_share, 'config/euroc/euroc_config.yaml'
        )
        calib_file = os.path.join(
            config_pkg_share, 'config/euroc/euroc_cam_calibration.yaml'
        )

        ns = mode

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

        support_path = os.path.join(config_pkg_share, 'support_files')
        pg_cfg = launch_cfg.get('pose_graph', {})

        # All params as a single flat dict
        params = {
            'config_pkg_share': config_pkg_share,
            'vins_folder': config_pkg_share,
            'camera_calibration_file': calib_file,
            'output_path': f'output_{ns}',
            'support_file': support_path,
            'visualization_shift_x': pg_cfg.get('visualization_shift_x', 0),
            'visualization_shift_y': pg_cfg.get('visualization_shift_y', 0),
            'skip_cnt': pg_cfg.get('skip_cnt', 0),
            'skip_dis': pg_cfg.get('skip_dis', 0.0),
        }
        # Add sparse_align params if enabled
        if launch_cfg.get('enable_sparse'):
            params.update(launch_cfg.get('sparse_align', {}))

        pipelines += [
            Node(
                package='feature_tracker',
                executable='feature_tracker_node',
                name='feature_tracker',
                output='log',
                remappings=ft_remaps,
                parameters=[params_file, params],
            ),
            Node(
                package='vins_estimator',
                executable='vins_estimator',
                name='vins_estimator',
                output='log',
                remappings=ve_remaps,
                parameters=[params_file, params],
            ),
            Node(
                package='pose_graph',
                executable='pose_graph',
                name='pose_graph',
                output='log',
                remappings=pg_remaps,
                parameters=[params_file, params],
            ),
        ]

    return pipelines


def launch_setup(context, *args, **kwargs):
    config_pkg_share = get_package_share_directory('config_pkg')
    vins_share = get_package_share_directory('vins_estimator')

    rviz_share = get_package_share_directory('rviz2')
    rviz_cfg = os.path.join(rviz_share, 'rviz', 'cfg', 'vins.rviz')
    if not os.path.exists(rviz_cfg):
        rviz_cfg = os.path.join(rviz_share, 'rviz', 'vins.rviz')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_cfg],
        output='log',
    )

    return build_pipelines(vins_share, config_pkg_share) + [rviz_node]


def generate_launch_description():
    return LaunchDescription(
        [OpaqueFunction(function=launch_setup)]
    )
