#!/usr/bin/env python3
"""Launch the planning simulator stack.

Node parameters and topic names live in config/planning_simulator.yaml.
"""

from __future__ import annotations

import math
import os
from typing import Any

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


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
            ('initialpose', topics.get('initialpose', '/initialpose')),
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


def _make_odom_visualization_node(cfg: dict[str, Any]) -> Node:
    topics = cfg.get('topics', {})
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
            # Publish world->base TF so the asr_sdm model can follow odometry.
            'tf45': bool(features.get('use_asr_sdm_model', True)),
        }],
        remappings=[
            ('odom', topics.get('odom', '/asr_sdm/odom')),
        ],
    )


def _make_robot_model_actions(cfg: dict[str, Any], asr_sdm_share: str) -> list[Any]:
    features = cfg.get('features', {})
    model_cfg = cfg.get('robot_model', {})
    enabled = _if_bool(bool(features.get('use_asr_sdm_model', True)))

    model_xacro = os.path.join(
        asr_sdm_share, model_cfg.get('model_xacro', 'urdf/asr_sdm_wrapper.urdf.xacro'))
    description_launch = os.path.join(
        asr_sdm_share, model_cfg.get('description_launch', 'launch/description.launch.py'))

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(description_launch),
            condition=enabled,
            launch_arguments={
                'model': model_xacro,
                'use_sim_time': str(bool(model_cfg.get('use_sim_time', False))).lower(),
            }.items(),
        ),
        Node(
            package='asr_sdm_controller',
            executable='realtime_front_unit_controller_3d',
            name='asr_sdm_kinematic_controller',
            output='screen',
            condition=enabled,
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_asr_sdm',
            condition=enabled,
            arguments=[
                '--frame-id', model_cfg.get('parent_frame', 'base'),
                '--child-frame-id', model_cfg.get('child_frame', 'screwdrive_segment_0'),
                '--yaw', str(float(model_cfg.get('yaw_rad', math.pi))),
                '--pitch', str(float(model_cfg.get('pitch_rad', math.pi / 2.0))),
            ],
        ),
    ]


def _make_kinematic_teleop_actions(cfg: dict[str, Any]) -> list[Node]:
    features = cfg.get('features', {})
    teleop = cfg.get('teleop', {})
    return [
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
            condition=_if_bool(bool(features.get('use_joy', True))),
        ),
        Node(
            package='asr_sdm_teleop',
            executable='asr_sdm_teleop_node',
            name='asr_sdm_teleop',
            output='screen',
            condition=_if_bool(bool(features.get('use_teleop', True))),
            parameters=[teleop],
        ),
    ]


def _make_rviz_node(cfg: dict[str, Any], rviz_config: str) -> Node:
    features = cfg.get('features', {})
    return Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        condition=_if_bool(bool(features.get('use_rviz', True))),
        arguments=['-d', rviz_config],
    )


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory('planning_simulator')
    config_path = os.path.join(pkg_share, 'config', 'planning_simulator.yaml')
    rviz_config = os.path.join(pkg_share, 'config', 'rviz.rviz')
    map_generator_config = os.path.join(
        get_package_share_directory('asr_sdm_map_generator'),
        'config', 'asr_sdm_map_generator.yaml')
    asr_sdm_share = get_package_share_directory('asr_sdm')

    cfg = _load_yaml(config_path)

    actions: list[Any] = [
        _make_map_generator_node(cfg, map_generator_config),
        _make_planning_simulator_node(cfg),
        _make_so3_control_node(cfg),
        _make_disturbance_node(cfg),
        _make_odom_visualization_node(cfg),
        *_make_robot_model_actions(cfg, asr_sdm_share),
        *_make_kinematic_teleop_actions(cfg.get('teleop', {})),
        _make_rviz_node(cfg, rviz_config),
    ]
    return LaunchDescription(actions)
