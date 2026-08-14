#!/usr/bin/env python3
"""Bring up the asr_sdm robot description.

Loads the URDF/Xacro and starts robot_state_publisher. If the URDF root
differs from the odometry base frame, also publishes a static transform
between them.

Nothing here publishes joint states, so on its own the model holds its zero pose.
asr_sdm_control_manager/launch/asr_sdm_control_manager.launch.py runs the
kinematic controller that animates it.

Parameters come from config/robot_model.yaml. Pass config_file:=<path> to use a
file with the same schema, which is how planning_simulator reuses this launch.
"""

from __future__ import annotations

import os
from pathlib import Path
from typing import Any

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

PACKAGE_NAME = 'asr_sdm'


def _load_yaml(path: str) -> dict[str, Any]:
    with open(path, 'r', encoding='utf-8') as f:
        return yaml.safe_load(f) or {}


def _if_bool(flag: bool) -> IfCondition:
    return IfCondition('true' if flag else 'false')


def _load_robot_description(model_path: Path) -> str:
    if model_path.suffix == '.xacro':
        import xacro

        return xacro.process_file(str(model_path)).toxml()
    return model_path.read_text(encoding='utf-8')


def _make_description_actions(cfg: dict[str, Any], package_share: str) -> list[Any]:
    features = cfg.get('features', {})
    model_cfg = cfg.get('robot_model', {})
    topics = cfg.get('topics', {})
    enabled = _if_bool(bool(features.get('use_asr_sdm_model', True)))

    model_xacro = os.path.join(
        package_share, model_cfg.get('model_xacro', 'urdf/asr_sdm_wrapper.urdf.xacro'))
    robot_description = _load_robot_description(Path(model_xacro))
    use_sim_time = bool(model_cfg.get('use_sim_time', False))
    joint_states_topic = topics.get('joint_states', '/control/joint_states')
    parent_frame = model_cfg.get('parent_frame', 'base')
    child_frame = model_cfg.get('child_frame', 'base')

    actions = [
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            condition=enabled,
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': use_sim_time,
            }],
            remappings=[
                ('joint_states', joint_states_topic),
            ],
            output='screen',
        ),
    ]
    # Skip identity base->base: odom already publishes world->base onto the URDF root.
    if parent_frame != child_frame:
        actions.append(
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='base_to_asr_sdm',
                condition=enabled,
                arguments=[
                    '--frame-id', parent_frame,
                    '--child-frame-id', child_frame,
                ],
            ),
        )
    return actions


def launch_setup(context) -> list[Any]:
    config_path = LaunchConfiguration('config_file').perform(context)
    package_share = get_package_share_directory(PACKAGE_NAME)

    cfg = _load_yaml(config_path)

    return _make_description_actions(cfg, package_share)


def generate_launch_description() -> LaunchDescription:
    default_config = PathJoinSubstitution([
        FindPackageShare(PACKAGE_NAME),
        'config',
        'robot_model.yaml',
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value=default_config,
            description='机器人模型参数 YAML 的绝对路径',
        ),
        OpaqueFunction(function=launch_setup),
    ])
