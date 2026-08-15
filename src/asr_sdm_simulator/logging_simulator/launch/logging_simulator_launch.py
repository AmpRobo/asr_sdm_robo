#!/usr/bin/env python3
"""Launch the logging simulator stack.

Core job: load the asr_sdm robot model, follow VINS odometry, show it in RViz.

    ros2 launch logging_simulator logging_simulator_launch.py

odom_visualization listens to:

    /localization/video_inertial_navigation_systems/odometry

and publishes world->base (stamped with now) so the asr_sdm RobotModel is
visible in RViz. Until VINS publishes, it holds the configured initial pose.

Node parameters and topic names live in config/logging_simulator.yaml.
robot_model:=asr_sdm includes asr_sdm/launch/asr_sdm_description.launch.py.

Optional stacks:

    control:=enable    asr_sdm_control_manager (default off)
    teleop:=enable     asr_sdm_teleop (default off)
    planning:=enable   asr_sdm_planning_manager (default off)
"""

from __future__ import annotations

import os
from typing import Any

import yaml
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


DEFAULT_VINS_ODOM = '/localization/video_inertial_navigation_systems/odometry'


def _load_yaml(path: str) -> dict[str, Any]:
    with open(path, 'r', encoding='utf-8') as f:
        return yaml.safe_load(f) or {}


def _if_bool(flag: bool) -> IfCondition:
    return IfCondition('true' if flag else 'false')


def _vins_odom_topic(cfg: dict[str, Any]) -> str:
    return cfg.get('topics', {}).get('vins_odom', DEFAULT_VINS_ODOM)


def _make_odom_visualization_node(cfg: dict[str, Any]) -> Node:
    features = cfg.get('features', {})
    viz = cfg.get('odom_visualization', {})
    color = viz.get('color', {})
    kc = cfg.get('kinematic_controller', {})
    return Node(
        package='odom_visualization',
        executable='odom_visualization',
        name='odom_visualization_ukf',
        output='screen',
        parameters=[{
            'color.a': float(color.get('a', 0.8)),
            'color.r': float(color.get('r', 1.0)),
            'color.g': float(color.get('g', 0.0)),
            'color.b': float(color.get('b', 0.0)),
            'covariance_scale': float(viz.get('covariance_scale', 100.0)),
            'odom_topics': [_vins_odom_topic(cfg)],
            # world->base for the asr_sdm RobotModel; stamps TF with now.
            'tf45': bool(features.get('use_asr_sdm_model', True)),
            'stamp_tf_with_now': True,
            'initial_x': float(kc.get('initial_x', -5.0)),
            'initial_y': float(kc.get('initial_y', 0.0)),
            'initial_z': float(kc.get('initial_z', 0.0)),
            'initial_yaw': float(kc.get('initial_yaw', 0.0)),
            'initial_pitch': float(kc.get('initial_pitch', 0.0)),
            'initial_roll': float(kc.get('initial_roll', 0.0)),
            'tf_yaw': float(viz.get('tf_yaw', 0.0)),
            'tf_pitch': float(viz.get('tf_pitch', 0.0)),
            'tf_roll': float(viz.get('tf_roll', 0.0)),
            'tf_publish_rate': 50.0,
        }],
    )


def _make_joint_state_publisher(cfg: dict[str, Any], control: str) -> list[Node]:
    """Zero joint states when the kinematic controller is not running."""
    if control == 'enable':
        return []
    topics = cfg.get('topics', {})
    return [Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'rate': 30.0}],
        remappings=[
            ('joint_states', topics.get('joint_states', '/control/joint_states')),
        ],
        output='screen',
    )]


def _make_robot_model_action(robot_model: str, config_path: str) -> IncludeLaunchDescription:
    """Bring up the robot model from the selected robot package."""
    try:
        robot_model_share = get_package_share_directory(robot_model)
    except PackageNotFoundError as error:
        raise RuntimeError(
            f"robot_model:={robot_model} is not an installed package") from error

    description_launch = os.path.join(
        robot_model_share, 'launch', f'{robot_model}_description.launch.py')
    if not os.path.isfile(description_launch):
        raise RuntimeError(
            f"robot_model:={robot_model} does not provide {description_launch}")

    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(description_launch),
        launch_arguments=[('config_file', config_path)],
    )


def _make_optional_include(
    argument: str,
    value: str,
    package: str,
    launch_arguments: dict[str, str] | None = None,
) -> list[IncludeLaunchDescription]:
    """Include <package>/launch/<package>.launch.py when the argument is 'enable'."""
    if value != 'enable':
        return []

    try:
        package_share = get_package_share_directory(package)
    except PackageNotFoundError as error:
        raise RuntimeError(f'{argument}:=enable needs the {package} package') from error

    include_kwargs: dict[str, Any] = {}
    if launch_arguments:
        include_kwargs['launch_arguments'] = list(launch_arguments.items())

    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(package_share, 'launch', f'{package}.launch.py')),
        **include_kwargs,
    )]


def _make_rviz_node(cfg: dict[str, Any], rviz_config: str) -> Node:
    features = cfg.get('features', {})
    return Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        condition=_if_bool(bool(features.get('use_rviz', True))),
        arguments=['-d', rviz_config],
    )


def launch_setup(context) -> list[Any]:
    robot_model = LaunchConfiguration('robot_model').perform(context)
    control = LaunchConfiguration('control').perform(context)
    teleop = LaunchConfiguration('teleop').perform(context)
    planning = LaunchConfiguration('planning').perform(context)

    pkg_share = get_package_share_directory('logging_simulator')
    config_path = os.path.join(pkg_share, 'config', 'logging_simulator.yaml')
    rviz_config = os.path.join(pkg_share, 'config', 'rviz.rviz')

    cfg = _load_yaml(config_path)

    return [
        _make_odom_visualization_node(cfg),
        _make_robot_model_action(robot_model, config_path),
        *_make_joint_state_publisher(cfg, control),
        *_make_optional_include(
            'control', control, 'asr_sdm_control_manager', {'config_file': config_path}),
        *_make_optional_include('teleop', teleop, 'asr_sdm_teleop'),
        *_make_optional_include(
            'planning', planning, 'asr_sdm_planning_manager',
            {'odom_topic': _vins_odom_topic(cfg)},
        ),
        _make_rviz_node(cfg, rviz_config),
    ]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_model',
            default_value='asr_sdm',
            description='Robot model package name; must provide launch/<name>_description.launch.py',
        ),
        DeclareLaunchArgument(
            'control',
            default_value='disable',
            choices=['enable', 'disable'],
            description='Start asr_sdm_control_manager kinematic controller (default off)',
        ),
        DeclareLaunchArgument(
            'teleop',
            default_value='disable',
            choices=['enable', 'disable'],
            description='Start asr_sdm_teleop gamepad teleop chain',
        ),
        DeclareLaunchArgument(
            'planning',
            default_value='disable',
            choices=['enable', 'disable'],
            description='Start asr_sdm_planning_manager planning chain',
        ),
        OpaqueFunction(function=launch_setup),
    ])
