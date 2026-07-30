#!/usr/bin/env python3
"""Launch the controller-driven ASR-SDM kinematic simulation.

Kinematic parameters live in the ``kinematic`` section of
config/planning_simulator.yaml.
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


def _make_kinematic_actions(
    cfg: dict[str, Any], asr_sdm_share: str, map_generator_config: str,
    rviz_config: str,
) -> list[Any]:
    features = cfg.get('features', {})
    initial_pose = cfg.get('initial_pose', {})
    topics = cfg.get('topics', {})
    frames = cfg.get('frames', {})
    controller = cfg.get('controller', {})
    joint_mapping = cfg.get('joint_mapping', {})
    teleop = cfg.get('teleop', {})

    model_xacro = os.path.join(asr_sdm_share, 'urdf', 'asr_sdm_wrapper.urdf.xacro')
    description_launch = os.path.join(asr_sdm_share, 'launch', 'description.launch.py')
    controller_params = {
        **controller,
        'cmd_vel_topic': topics.get('cmd_vel', '/asr_sdm/cmd_vel'),
        'controller_state_topic': topics.get(
            'controller_state', '/asr_sdm/controller_state_3d'),
        'initialpose_topic': topics.get('initialpose', '/initialpose'),
        'odom_topic': topics.get('odom', '/asr_sdm/odom'),
        'joint_state_topic': topics.get('joint_states', '/joint_states'),
        'world_frame': frames.get('world', 'world'),
        'controller_base_frame': frames.get(
            'controller_base', 'asr_sdm_controller_base'),
        'root_frame': frames.get('root', 'screwdrive_segment_0'),
        'joint_source_indices': joint_mapping.get(
            'source_indices', [0, 1, 2, 3, 4, 5]),
        'joint_signs': joint_mapping.get('signs', [1.0] * 6),
        'joint_offsets_rad': joint_mapping.get('offsets_rad', [0.0] * 6),
        'clip_joint_positions': bool(joint_mapping.get('clip_positions', True)),
        'joint_position_limit_rad': float(
            joint_mapping.get('position_limit_rad', math.pi / 2.0)),
        'initial_x': float(initial_pose.get('x', -5.0)),
        'initial_y': float(initial_pose.get('y', 0.0)),
        'initial_z': float(initial_pose.get('z', 0.0)),
        'initial_yaw': float(initial_pose.get('yaw', 0.0)),
    }

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(description_launch),
            launch_arguments={
                'model': model_xacro,
                'use_sim_time': 'false',
            }.items(),
        ),
        Node(
            package='asr_sdm_controller',
            executable='realtime_front_unit_controller_3d',
            name='asr_sdm_kinematic_controller',
            output='screen',
            parameters=[controller_params],
        ),
        Node(
            package='asr_sdm_map_generator',
            executable='random_forest_test',
            name='random_map_sensing',
            output='screen',
            emulate_tty=True,
            condition=_if_bool(bool(features.get('use_map', True))),
            parameters=[
                map_generator_config,
                {
                    'init_state_x': float(initial_pose.get('x', -5.0)),
                    'init_state_y': float(initial_pose.get('y', 0.0)),
                    'sensing.enable_click_map': bool(
                        features.get('enable_click_map', True)),
                    'topics.add_static_obstacle': topics.get(
                        'add_static_obstacle',
                        '/simulator/planning_simulator/add_static_obstacle'),
                },
            ],
            remappings=[('odometry', topics.get('odom', '/asr_sdm/odom'))],
        ),
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
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz',
            output='screen',
            condition=_if_bool(bool(features.get('use_rviz', True))),
            arguments=['-d', rviz_config],
        ),
    ]


def generate_launch_description() -> LaunchDescription:
    package_share = get_package_share_directory('planning_simulator')
    asr_sdm_share = get_package_share_directory('asr_sdm')
    map_generator_share = get_package_share_directory('asr_sdm_map_generator')
    config = _load_yaml(os.path.join(package_share, 'config', 'planning_simulator.yaml'))
    map_generator_config = os.path.join(
        map_generator_share, 'config', 'asr_sdm_map_generator.yaml')
    rviz_config = os.path.join(package_share, 'config', 'rviz.rviz')

    return LaunchDescription(_make_kinematic_actions(
        config.get('kinematic', {}), asr_sdm_share, map_generator_config, rviz_config))
