#!/usr/bin/env python3
"""Bring up the asr_sdm robot model.

Starts robot_state_publisher through description.launch.py plus the static
transform that orients the model under its base frame.

Nothing here publishes joint states, so on its own the model holds its zero pose.
asr_sdm_control_manager/launch/asr_sdm_control_manager.launch.py runs the
kinematic controller that animates it.

Parameters come from config/robot_model.yaml. Pass config_file:=<path> to use a
file with the same schema, which is how planning_simulator reuses asr_sdm_model.launch.py.
"""

from __future__ import annotations

import math
import os
from typing import Any

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


PACKAGE_NAME = 'asr_sdm'


def _load_yaml(path: str) -> dict[str, Any]:
    with open(path, 'r', encoding='utf-8') as f:
        return yaml.safe_load(f) or {}


def _if_bool(flag: bool) -> IfCondition:
    return IfCondition('true' if flag else 'false')


def _make_description_actions(cfg: dict[str, Any], package_share: str) -> list[Any]:
    features = cfg.get('features', {})
    model_cfg = cfg.get('robot_model', {})
    topics = cfg.get('topics', {})
    enabled = _if_bool(bool(features.get('use_asr_sdm_model', True)))

    model_xacro = os.path.join(
        package_share, model_cfg.get('model_xacro', 'urdf/asr_sdm_wrapper.urdf.xacro'))
    description_launch = os.path.join(
        package_share, model_cfg.get('description_launch', 'launch/description.launch.py'))

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(description_launch),
            condition=enabled,
            launch_arguments={
                'model': model_xacro,
                'use_sim_time': str(bool(model_cfg.get('use_sim_time', False))).lower(),
                'joint_states_topic': topics.get('joint_states', '/control/joint_states'),
            }.items(),
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
