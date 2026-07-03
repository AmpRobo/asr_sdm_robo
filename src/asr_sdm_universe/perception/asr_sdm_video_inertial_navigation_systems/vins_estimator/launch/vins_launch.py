from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, LogInfo, OpaqueFunction, GroupAction,
)
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node, PushRosNamespace
import os


# ---------------------------------------------------------------------------
# Sparse image alignment section (SVO-style photometric Gauss-Newton)
# Injected into config yaml when enable_sparse=1, stripped when enable_sparse=0.
# ---------------------------------------------------------------------------
SPARSE_SECTION_MARKER = '\n# Sparse image alignment'

SPARSE_SECTION_BODY = (
    SPARSE_SECTION_MARKER + ' (SVO-style semi-direct refinement in the front-end)\n'
    'use_sparse_align: 1\n'
    'sparse_align_patch_size: 2\n'
    'sparse_align_max_level: 2\n'
    'sparse_align_min_level: 0\n'
    'sparse_align_max_iter: 4\n'
    'sparse_align_lambda_rot: 0.5\n'
    'sparse_align_lambda_trans: 0.0\n'
    'sparse_align_chi2_thresh: 50.0\n'
    'sparse_align_min_features: 30\n'
    'sparse_align_min_iter_for_ok: 1\n'
    'use_td_pre_calib: 1\n'
)


def _materialize_config(context, *args, **kwargs):
    """Read base config, optionally append sparse section, write materialized copy."""
    base_path = context.perform_substitution(LaunchConfiguration('config_file'))

    with open(base_path, 'r') as f:
        src_text = f.read()

    if SPARSE_SECTION_MARKER in src_text:
        base_text = src_text.split(SPARSE_SECTION_MARKER)[0].rstrip() + '\n'
    else:
        base_text = src_text

    enable_sparse = context.perform_substitution(LaunchConfiguration('enable_sparse'))
    if enable_sparse == '1':
        final_text = base_text.rstrip() + '\n' + SPARSE_SECTION_BODY
    else:
        final_text = base_text

    config_pkg_path = get_package_share_directory('config_pkg')
    output_dir = os.path.join(config_pkg_path, 'config')
    os.makedirs(output_dir, exist_ok=True)

    base_name = os.path.splitext(os.path.basename(base_path))[0]
    output_path = os.path.join(output_dir, f'{base_name}_materialized.yaml')

    with open(output_path, 'w') as f:
        f.write(final_text)

    # Store in context so LaunchConfiguration('__materialized_config') resolves
    context.launch_configurations['__materialized_config'] = output_path

    return [
        LogInfo(msg=[f'[vins launch] materialized config: {output_path}']),
        LogInfo(msg=[f'[vins launch] sparse alignment: {"ON" if enable_sparse == "1" else "OFF"}']),
    ]


def generate_launch_description():
    config_pkg_path = get_package_share_directory('config_pkg')

    return LaunchDescription([

        # ---- parameters -----------------------------------------------------
        DeclareLaunchArgument(
            'config_file',
            default_value=os.path.join(config_pkg_path, 'config/euroc/euroc_config.yaml'),
            description=(
                'Base VINS config yaml. Edit imu_topic / image_topic and '
                'camera calibration here to match your dataset '
                '(EuRoC, RealSense D435i, custom rosbag / mcap).'
            ),
        ),

        DeclareLaunchArgument(
            'enable_sparse',
            default_value='0',
            description='0/1 - enable SVO-style sparse photometric alignment.',
        ),

        # ---- materialize config at launch time ------------------------------
        OpaqueFunction(function=_materialize_config),

        # ---- pipeline -------------------------------------------------------
        _pipeline(config_pkg_path=config_pkg_path),
    ])


def _pipeline(config_pkg_path):
    """
    Return launch actions for the VINS pipeline.

    Pipeline: feature_tracker -> vins_estimator -> pose_graph + rviz2
    All nodes run under /sparse1 (matched to the default rviz config).
    """
    ns = 'sparse1'
    materialized_cfg = LaunchConfiguration('__materialized_config')

    support_path = os.path.join(config_pkg_path, 'support_files')
    rviz_cfg = os.path.join(config_pkg_path, 'config', 'vins_euroc_rviz.rviz')

    # feature_tracker relative publishers -> remap under /vins_estimator/
    ft_remaps = [
        ('feature', f'/{ns}/feature'),
        ('feature_img', f'/{ns}/feature_img'),
        ('restart', f'/{ns}/restart'),
    ]
    # feature_tracker subscribes to image/imu via absolute topics from yaml
    # (e.g. /imu0, /cam0/image_raw) — no subscribe remaps needed.

    # vins_estimator relative publishers
    ve_remaps = [
        (t, f'/{ns}/{t}') for t in [
            'camera_pose_visual', 'keyframe_point', 'path', 'odometry',
            'extrinsic', 'imu_propagate', 'keyframe_pose',
            'relo_relative_pose', 'key_poses', 'camera_pose',
            'point_cloud', 'history_cloud', 'relocalization_path',
        ]
    ]
    # vins_estimator's hard-coded subscriptions to feature_tracker / pose_graph
    ve_sub_remaps = [
        ('/feature_tracker/feature', f'/{ns}/feature'),
        ('/feature_tracker/restart', f'/{ns}/restart'),
        ('/pose_graph/match_points', f'/{ns}/match_points'),
        ('/feature_tracker/sparse_rot', f'/{ns}/sparse_rot'),   # D2.1
        ('/feature_tracker/td_estimate', f'/{ns}/td_estimate'), # D2.2
    ]
    ve_remaps = ve_sub_remaps + ve_remaps

    # pose_graph relative publishers
    pg_remaps = [
        (t, f'/{ns}/{t}') for t in [
            'match_image', 'camera_pose_visual', 'key_odometrys',
            'no_loop_path', 'match_points', 'pose_graph_path',
            'base_path', 'pose_graph',
        ]
    ]
    # pose_graph's hard-coded subscriptions to vins_estimator topics
    pg_sub_remaps = [
        ('/vins_estimator/imu_propagate', f'/{ns}/imu_propagate'),
        ('/vins_estimator/odometry', f'/{ns}/odometry'),
        ('/vins_estimator/keyframe_pose', f'/{ns}/keyframe_pose'),
        ('/vins_estimator/extrinsic', f'/{ns}/extrinsic'),
        ('/vins_estimator/keyframe_point', f'/{ns}/keyframe_point'),
        ('/vins_estimator/relo_relative_pose', f'/{ns}/relo_relative_pose'),
    ]
    pg_remaps = pg_sub_remaps + pg_remaps

    return GroupAction([
        PushRosNamespace(ns),

        # ---- feature_tracker (front-end: KLT + optional sparse align) -----
        Node(
            package='feature_tracker',
            executable='feature_tracker_node',
            name='feature_tracker',
            output='screen',
            remappings=ft_remaps,
            parameters=[{
                'config_file': materialized_cfg,
                'vins_folder': config_pkg_path,
            }],
        ),

        # ---- vins_estimator (back-end: MSCKF + BA) -------------------------
        Node(
            package='vins_estimator',
            executable='vins_estimator',
            name='vins_estimator',
            output='screen',
            remappings=ve_remaps,
            parameters=[{
                'config_file': materialized_cfg,
                'vins_folder': config_pkg_path,
            }],
        ),

        # ---- pose_graph (loop closure + visualization) ---------------------
        Node(
            package='pose_graph',
            executable='pose_graph',
            name='pose_graph',
            output='screen',
            remappings=pg_remaps,
            parameters=[{
                'config_file': materialized_cfg,
                'support_file': support_path,
                'visualization_shift_x': 0,
                'visualization_shift_y': 0,
                'skip_cnt': 0,
                'skip_dis': 0.0,
            }],
        ),

        # ---- rviz2 ---------------------------------------------------------
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_cfg],
        ),
    ])
