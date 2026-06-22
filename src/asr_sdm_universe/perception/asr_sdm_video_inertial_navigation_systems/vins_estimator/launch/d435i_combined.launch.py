from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction, GroupAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node, PushRosNamespace
import os


# ---------------------------------------------------------------------------
# Materialize two config files at launch-time under /tmp
#   - realsense_d435i_original.yaml : the upstream yaml without the sparse section
#   - realsense_d435i_sparse1.yaml  : the same yaml plus the sparse section
# Both pipelines share the SAME source yaml (so only one config_pkg file needs
# to be tracked in git) but read it through /tmp so the runtime behaviour is
# fully controlled by the launch arguments.
# ---------------------------------------------------------------------------
SPARSE_SECTION_MARKER = '\n# Sparse image alignment'

SPARSE_SECTION_BODY = (
    SPARSE_SECTION_MARKER + ' (SVO-style semi-direct refinement in the front-end)\n'
    '# Runs a photometric Gauss-Newton over 4x4 patches on a 4-level image\n'
    '# pyramid; replaces the KLT output when alignment converges and falls back\n'
    '# to KLT otherwise. Requires IMU messages to provide a rotation prior.\n'
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
)


def _materialize_config(context, *args, **kwargs):
    base_path = context.perform_substitution(
        PathJoinSubstitution([
            get_package_share_directory('config_pkg'),
            'config/realsense/realsense_d435i_config.yaml'
        ])
    )
    with open(base_path, 'r') as f:
        src_text = f.read()

    if SPARSE_SECTION_MARKER in src_text:
        original_text = src_text.split(SPARSE_SECTION_MARKER)[0].rstrip() + '\n'
    else:
        original_text = src_text

    sparse1_text = src_text.rstrip() + '\n' + SPARSE_SECTION_BODY

    # Per-pipeline output_path so the two pipelines don't write to the same CSV
    original_text = original_text.replace(
        'output_path: "/home/lxy/output/"',
        'output_path: "/home/lxy/output/original/"', 1)
    original_text = original_text.replace(
        'pose_graph_save_path: "/home/lxy/output/pose_graph/"',
        'pose_graph_save_path: "/home/lxy/output/pose_graph/original/"', 1)
    sparse1_text = sparse1_text.replace(
        'output_path: "/home/lxy/output/"',
        'output_path: "/home/lxy/output/sparse1/"', 1)
    sparse1_text = sparse1_text.replace(
        'pose_graph_save_path: "/home/lxy/output/pose_graph/"',
        'pose_graph_save_path: "/home/lxy/output/pose_graph/sparse1/"', 1)

    os.makedirs('/tmp/vins_d435i_configs', exist_ok=True)
    original_cfg = '/tmp/vins_d435i_configs/realsense_d435i_original.yaml'
    sparse1_cfg = '/tmp/vins_d435i_configs/realsense_d435i_sparse1.yaml'
    with open(original_cfg, 'w') as f:
        f.write(original_text)
    with open(sparse1_cfg, 'w') as f:
        f.write(sparse1_text)

    # Stash for downstream OpaqueFunctions
    context.launch_configurations['__vins_cfg_original'] = original_cfg
    context.launch_configurations['__vins_cfg_sparse1'] = sparse1_cfg
    return [
        LogInfo(msg=[f'[vins d435i launch] materialized {original_cfg}']),
        LogInfo(msg=[f'[vins d435i launch] materialized {sparse1_cfg}']),
    ]


def _launch_pipelines(context, *args, **kwargs):
    enable_original = context.perform_substitution(LaunchConfiguration('enable_original'))
    enable_sparse1 = context.perform_substitution(LaunchConfiguration('enable_sparse1'))

    config_pkg_path = get_package_share_directory('config_pkg')
    cfg_original = context.launch_configurations.get(
        '__vins_cfg_original', '/tmp/vins_d435i_configs/realsense_d435i_original.yaml')
    cfg_sparse1 = context.launch_configurations.get(
        '__vins_cfg_sparse1', '/tmp/vins_d435i_configs/realsense_d435i_sparse1.yaml')

    actions = []

    def _pipeline(label, ns, config_path, output_subdir):
        base_params = {
            'config_file': config_path,
            'vins_folder': config_pkg_path,
            'config_pkg_share': config_pkg_path,
        }
        ve_params = dict(base_params)
        ve_params.pop('config_pkg_share', None)
        pg_params = {
            'config_file': config_path,
            'support_file': os.path.join(config_pkg_path, 'support_files'),
            'visualization_shift_x': 0,
            'visualization_shift_y': 0,
            'skip_cnt': 0,
            'skip_dis': 0.0,
        }
        # The C++ nodes create publishers with relative topic names but
        # rclcpp::Node::make_shared(name) does not pick up the __ns remap
        # automatically, so the topics end up in the root namespace.
        # We push the namespace via PushRosNamespace (which updates the
        # __ns at the process level too) AND remap each relative topic
        # explicitly to the fully-qualified name under <ns>/.
        #
        # The vins_estimator and pose_graph nodes subscribe to a few
        # *absolute* topic names (e.g. "/feature_tracker/feature",
        # "/pose_graph/match_points"); remap those to the namespaced
        # versions so the subscription can find the publishers of
        # *this* pipeline (and not the publishers of the other
        # pipeline, if both are running at the same time).
        ft_remaps = [
            ('feature', f'/{ns}/feature'),
            ('feature_img', f'/{ns}/feature_img'),
            ('restart', f'/{ns}/restart'),
        ]
        # feature_tracker subscribes to image/imu with absolute names from
        # the yaml (e.g. /sensing/camera/realsense/color/image_raw) so
        # no subscribe remaps are required.
        ve_remaps = [
            (t, f'/{ns}/{t}') for t in [
                'camera_pose_visual', 'keyframe_point', 'path', 'odometry',
                'extrinsic', 'imu_propagate', 'keyframe_pose',
                'relo_relative_pose', 'key_poses', 'camera_pose',
                'point_cloud', 'history_cloud', 'relocalization_path',
            ]
        ]
        # vins_estimator's hard-coded subscriptions to feature_tracker /
        # pose_graph topics
        ve_sub_remaps = [
            ('/feature_tracker/feature', f'/{ns}/feature'),
            ('/feature_tracker/restart', f'/{ns}/restart'),
            ('/pose_graph/match_points', f'/{ns}/match_points'),
            ('/feature_tracker/sparse_rot', f'/{ns}/sparse_rot'),  # D2.1
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
        return [
            LogInfo(msg=[
                f'[vins d435i launch] starting pipeline {label}:',
                f' ns=/{ns}',
                f' config={config_path}',
            ]),
            GroupAction([
                PushRosNamespace(ns),
                Node(
                    package='feature_tracker',
                    executable='feature_tracker_node',
                    name='feature_tracker',
                    output='screen',
                    parameters=[base_params],
                    remappings=ft_remaps,
                ),
                Node(
                    package='vins_estimator',
                    executable='vins_estimator',
                    name='vins_estimator',
                    output='screen',
                    parameters=[ve_params],
                    remappings=ve_remaps,
                ),
                Node(
                    package='pose_graph',
                    executable='pose_graph',
                    name='pose_graph',
                    output='screen',
                    parameters=[pg_params],
                    remappings=pg_remaps,
                ),
            ]),
        ]

    if enable_original == '1':
        actions.extend(_pipeline(
            'ORIGINAL (sparse=0)',
            'original',
            cfg_original,
            'original',
        ))

    if enable_sparse1 == '1':
        actions.extend(_pipeline(
            'SPARSE1 (sparse=1 改造)',
            'sparse1',
            cfg_sparse1,
            'sparse1',
        ))
    if enable_original != '1' and enable_sparse1 != '1':
        actions.append(LogInfo(msg=[
            '[vins d435i launch] both pipelines disabled, nothing to do.'
        ]))

    return actions


def generate_launch_description():
    config_pkg_path = get_package_share_directory('config_pkg')
    rviz_config_path = PathJoinSubstitution([
        config_pkg_path,
        'config/vins_euroc_rviz.rviz'
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'enable_original',
            default_value='1',
            description='0/1 - start the original VINS pipeline (sparse=0).'
        ),
        DeclareLaunchArgument(
            'enable_sparse1',
            default_value='0',
            description='0/1 - start the sparse1改造 VINS pipeline (sparse=1).'
        ),
        LogInfo(msg=['[vins d435i launch] Materializing runtime config files']),
        OpaqueFunction(function=_materialize_config),
        OpaqueFunction(function=_launch_pipelines),
        LogInfo(msg=['[vins d435i launch] Starting rviz2']),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_path],
            output='screen',
        ),
    ])
