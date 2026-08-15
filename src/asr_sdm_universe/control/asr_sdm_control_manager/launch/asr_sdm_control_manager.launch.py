#!/usr/bin/env python3
"""Launch the asr_sdm kinematic controller.

asr_sdm_control_manager turns cmd_vel into joint states and odometry.
Node parameters live in config/asr_sdm_control_manager.yaml.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("asr_sdm_control_manager"),
        "config", "asr_sdm_control_manager.yaml")

    asr_sdm_control_manager = Node(
        package="asr_sdm_control_manager",
        executable="asr_sdm_control_manager",
        name="asr_sdm_kinematic_controller",
        output="screen",
        parameters=[config],
    )

    return LaunchDescription([
        asr_sdm_control_manager,
    ])
