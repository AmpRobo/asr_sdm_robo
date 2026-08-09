#!/usr/bin/env python3
"""Launch the planning simulator stack.

Node parameters and topic names live in config/planning_simulator.yaml.

The robot model and its controller come from the package named by the robot_model
argument, which defaults to asr_sdm:

    ros2 launch planning_simulator planning_simulator_launch.py robot_model:=asr_sdm

That package must ship launch/<robot_model>_description.launch.py, which is included here
and reads the same config file.

The robot model pose in RViz comes from odom_visualization, which turns one
odometry topic into the world->base transform. Pick the source with:

    odom_source:=auto      whichever source published most recently (default)
    odom_source:=control   /control/asr_sdm/odom, the kinematic controller
    odom_source:=vins      /localization/video_inertial_navigation_systems/odometry

odom_visualization subscribes to the selected topics and only it broadcasts that
transform, so the sources never fight over TF. With auto, run either stack alone
and the model follows it; run both and the pose alternates between the estimates.

Optional stacks, each included from <package>/launch/<package>.launch.py:

    control:=enable    asr_sdm_control_manager, the kinematic controller (default on)
    teleop:=enable     asr_sdm_teleop, the gamepad chain (default off)
    planning:=enable   asr_sdm_planning_manager, topological replanning (default off)

    ros2 launch planning_simulator planning_simulator_launch.py teleop:=enable
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


DEFAULT_SIM_ODOM = '/control/asr_sdm/odom'
DEFAULT_VINS_ODOM = '/localization/video_inertial_navigation_systems/odometry'


def _load_yaml(path: str) -> dict[str, Any]:
    with open(path, 'r', encoding='utf-8') as f:
        return yaml.safe_load(f) or {}


def _if_bool(flag: bool) -> IfCondition:
    return IfCondition('true' if flag else 'false')


def _flatten_params(prefix: str, value: Any) -> dict[str, Any]:
    """Flatten nested dicts into ROS dotted parameter names."""
    if not isinstance(value, dict):
        return {prefix: value}
    out: dict[str, Any] = {}
    for key, child in value.items():
        dotted = f'{prefix}.{key}' if prefix else str(key)
        out.update(_flatten_params(dotted, child))
    return out


def _make_map_generator_node(cfg: dict[str, Any], map_generator_config: str) -> Node:
    topics = cfg.get('topics', {})
    sim = cfg.get('simulator', {})
    features = cfg.get('features', {})
    return Node(
        package='asr_sdm_map_generator',
        executable='random_forest_test',
        name='random_map_sensing',
        output='screen',
        emulate_tty=True,
        condition=_if_bool(bool(features.get('use_asr_sdm_map_generator', True))),
        parameters=[
            map_generator_config,
            {
                'init_state_x': float(sim.get('init_state_x', -5.0)),
                'init_state_y': float(sim.get('init_state_y', 0.0)),
                'sensing.enable_click_map': bool(features.get('enable_click_map', True)),
                'topics.add_static_obstacle': topics.get('add_static_obstacle', '/simulator/planning_simulator/add_static_obstacle'),
            },
        ],
        remappings=[
            ('odometry', topics.get('visual_slam_odom', '/visual_slam/odom')),
        ],
    )


def _make_planning_simulator_node(cfg: dict[str, Any]) -> Node:
    topics = cfg.get('topics', {})
    sim = cfg.get('simulator', {})
    rate = cfg.get('rate', {})
    return Node(
        package='planning_simulator',
        executable='planning_simulator',
        name='planning_simulator',
        output='screen',
        parameters=[{
            'rate.odom': float(rate.get('odom', 100.0)),
            'simulator.init_state_x': float(sim.get('init_state_x', -5.0)),
            'simulator.init_state_y': float(sim.get('init_state_y', 0.0)),
            'simulator.init_state_z': float(sim.get('init_state_z', 3.0)),
        }],
        remappings=[
            ('odom', topics.get('visual_slam_odom', '/visual_slam/odom')),
            ('cmd', topics.get('so3_cmd', 'so3_cmd')),
            ('imu', topics.get('imu', 'sim/imu')),
            ('force_disturbance', topics.get('force_disturbance', 'force_disturbance')),
            ('moment_disturbance', topics.get('moment_disturbance', 'moment_disturbance')),
            ('initialpose', topics.get('initialpose', '/control/initial_pose')),
        ],
    )


def _make_so3_control_node(cfg: dict[str, Any]) -> Node:
    topics = cfg.get('topics', {})
    features = cfg.get('features', {})
    so3 = cfg.get('so3_control', {})
    params = {
        'mass': float(so3.get('mass', 0.98)),
        'use_angle_corrections': bool(so3.get('use_angle_corrections', False)),
        'use_external_yaw': bool(so3.get('use_external_yaw', False)),
    }
    params.update(_flatten_params('gains', so3.get('gains', {})))
    params.update(_flatten_params('corrections', so3.get('corrections', {})))
    return Node(
        package='so3_control',
        executable='so3_control_node',
        name='so3_control',
        output='screen',
        condition=_if_bool(bool(features.get('use_so3_control', False))),
        parameters=[params],
        remappings=[
            ('odom', topics.get('state_ukf_odom', '/state_ukf/odom')),
            ('position_cmd', topics.get('position_cmd', 'position_cmd')),
            ('motors', topics.get('motors', 'motors')),
            ('corrections', topics.get('corrections', 'corrections')),
            ('so3_cmd', topics.get('so3_cmd', 'so3_cmd')),
            ('imu', topics.get('imu', 'sim/imu')),
        ],
    )


def _make_disturbance_node(cfg: dict[str, Any]) -> Node:
    topics = cfg.get('topics', {})
    return Node(
        package='so3_disturbance_generator',
        executable='so3_disturbance_generator',
        name='so3_disturbance_generator',
        output='screen',
        remappings=[
            ('odom', topics.get('visual_slam_odom', '/visual_slam/odom')),
            ('noisy_odom', topics.get('state_ukf_odom', '/state_ukf/odom')),
            ('correction', topics.get('visual_slam_correction', '/visual_slam/correction')),
            ('force_disturbance', topics.get('force_disturbance', 'force_disturbance')),
            ('moment_disturbance', topics.get('moment_disturbance', 'moment_disturbance')),
        ],
    )


def _odom_source_topics(cfg: dict[str, Any], odom_source: str) -> list[str]:
    """Odometry inputs that become the world->base transform of the robot model."""
    topics = cfg.get('topics', {})
    sim_odom = topics.get('sim_odom', DEFAULT_SIM_ODOM)
    vins_odom = topics.get('vins_odom', DEFAULT_VINS_ODOM)
    if odom_source == 'control':
        return [sim_odom]
    if odom_source == 'vins':
        return [vins_odom]
    return [sim_odom, vins_odom]


def _make_odom_visualization_node(cfg: dict[str, Any], odom_source: str) -> Node:
    features = cfg.get('features', {})
    viz = cfg.get('odom_visualization', {})
    color = viz.get('color', {})
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
            'odom_topics': _odom_source_topics(cfg, odom_source),
            # Publish world->base TF so the asr_sdm model can follow odometry.
            'tf45': bool(features.get('use_asr_sdm_model', True)),
        }],
    )


def _make_robot_model_action(robot_model: str, config_path: str) -> IncludeLaunchDescription:
    """Bring up the robot model and its controller from the selected robot package.

    robot_model names a package that ships launch/<robot_model>_description.launch.py.
    The included launch reads the same config file, so the robot_model,
    kinematic_controller and topics sections below stay authoritative here.
    """
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
        launch_arguments={'config_file': config_path}.items(),
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

    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(package_share, 'launch', f'{package}.launch.py')),
        launch_arguments=(launch_arguments or {}).items(),
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
    odom_source = LaunchConfiguration('odom_source').perform(context)
    control = LaunchConfiguration('control').perform(context)
    teleop = LaunchConfiguration('teleop').perform(context)
    planning = LaunchConfiguration('planning').perform(context)

    pkg_share = get_package_share_directory('planning_simulator')
    config_path = os.path.join(pkg_share, 'config', 'planning_simulator.yaml')
    rviz_config = os.path.join(pkg_share, 'config', 'rviz.rviz')
    map_generator_config = os.path.join(
        get_package_share_directory('asr_sdm_map_generator'),
        'config', 'asr_sdm_map_generator.yaml')

    cfg = _load_yaml(config_path)

    return [
        _make_map_generator_node(cfg, map_generator_config),
        _make_planning_simulator_node(cfg),
        _make_so3_control_node(cfg),
        _make_disturbance_node(cfg),
        _make_odom_visualization_node(cfg, odom_source),
        _make_robot_model_action(robot_model, config_path),
        *_make_optional_include(
            'control', control, 'asr_sdm_control_manager', {'config_file': config_path}),
        *_make_optional_include('teleop', teleop, 'asr_sdm_teleop'),
        *_make_optional_include(
            'planning', planning, 'asr_sdm_planning_manager',
            {'odom_topic': cfg.get('topics', {}).get('visual_slam_odom', '/visual_slam/odom')},
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
            'odom_source',
            default_value='auto',
            choices=['auto', 'control', 'vins'],
            description='Odometry that drives the robot model pose: auto follows whichever '
                        'source published most recently, control pins controller odom, '
                        'vins pins VINS localization odometry',
        ),
        DeclareLaunchArgument(
            'control',
            default_value='enable',
            choices=['enable', 'disable'],
            description='Start asr_sdm_control_manager kinematic controller (model will not move if disabled)',
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
