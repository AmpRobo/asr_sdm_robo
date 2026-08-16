#!/usr/bin/env python3
"""Launch the joystick chain: the joy driver and asr_sdm_teleop_node.

asr_sdm_teleop_node turns /joy into the robot_cmd topic that the asr_sdm kinematic
controller consumes.

Node parameters live in config/asr_sdm_teleop.yaml.
"""

from __future__ import annotations

import os
from typing import Any

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch_ros.actions import Node


PACKAGE_NAME = 'asr_sdm_teleop'


def _load_yaml(path: str) -> dict[str, Any]:
    with open(path, 'r', encoding='utf-8') as f:
        return yaml.safe_load(f) or {}


def _if_bool(flag: bool) -> IfCondition:
    return IfCondition('true' if flag else 'false')


def _make_teleop_actions(cfg: dict[str, Any]) -> list[Node]:
    features = cfg.get('features', {})
    return [
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
            condition=_if_bool(bool(features.get('use_joy', True))),
            parameters=[cfg.get('joy', {})],
        ),
        Node(
            package=PACKAGE_NAME,
            executable='asr_sdm_teleop_node',
            name='asr_sdm_teleop',
            output='screen',
            condition=_if_bool(bool(features.get('use_teleop', True))),
            parameters=[cfg.get('teleop', {})],
        ),
    ]


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory(PACKAGE_NAME)
    config_path = os.path.join(pkg_share, 'config', 'asr_sdm_teleop.yaml')

    cfg = _load_yaml(config_path)

    actions: list[Any] = [
        *_make_teleop_actions(cfg),
    ]
    return LaunchDescription(actions)
