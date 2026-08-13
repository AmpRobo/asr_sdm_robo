#!/usr/bin/env python3
"""
Launch the asr_sdm kinematic controller.

asr_sdm_control_manager turns cmd_vel into joint states and odometry,
which is what animates the robot model published by asr_sdm_model.launch.py.

Parameters come from config/asr_sdm_control_manager.yaml. Pass config_file:=<path>
to use a file with the same schema, which is how planning_simulator reuses this
launch.
"""

from __future__ import annotations

from typing import Any

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import yaml


PACKAGE_NAME = 'asr_sdm_control_manager'


def _load_yaml(path: str) -> dict[str, Any]:
    with open(path, 'r', encoding='utf-8') as f:
        return yaml.safe_load(f) or {}


def _if_bool(flag: bool) -> IfCondition:
    return IfCondition('true' if flag else 'false')


def _make_kinematic_controller_node(cfg: dict[str, Any]) -> Node:
    topics = cfg.get('topics', {})
    features = cfg.get('features', {})
    params = dict(cfg.get('kinematic_controller', {}))
    params.update({
        'cmd_vel_topic': topics.get('cmd_vel', '/control/asr_sdm/cmd_vel'),
        'controller_state_topic': topics.get(
            'controller_state', '/control/asr_sdm/controller_state_3d'),
        'control_cmd_topic': topics.get('control_cmd', '/control/asr_sdm/control_cmd_3d'),
        'initialpose_topic': topics.get('initialpose', '/control/initial_pose'),
        # planning_simulator calls the same topic sim_odom in its own config.
        'odom_topic': topics.get('sim_odom', topics.get('odom', '/control/asr_sdm/odom')),
        'joint_state_topic': topics.get('joint_states', '/control/joint_states'),
    })
    return Node(
        package=PACKAGE_NAME,
        executable='asr_sdm_control_manager',
        name='asr_sdm_kinematic_controller',
        output='screen',
        condition=_if_bool(bool(features.get('use_kinematic_controller', True))),
        parameters=[params],
    )


def launch_setup(context) -> list[Any]:
    config_path = LaunchConfiguration('config_file').perform(context)

    cfg = _load_yaml(config_path)

    return [
        _make_kinematic_controller_node(cfg),
    ]


def generate_launch_description() -> LaunchDescription:
    default_config = PathJoinSubstitution([
        FindPackageShare(PACKAGE_NAME),
        'config',
        'asr_sdm_control_manager.yaml',
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value=default_config,
            description='控制器参数 YAML 的绝对路径',
        ),
        OpaqueFunction(function=launch_setup),
    ])
