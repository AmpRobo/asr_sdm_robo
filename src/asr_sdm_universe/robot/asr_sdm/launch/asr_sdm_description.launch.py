#!/usr/bin/env python3
"""Bring up the asr_sdm robot description.

Loads the URDF/Xacro and starts robot_state_publisher.
Nothing here publishes joint states, so on its own the model holds its zero pose.
asr_sdm_control_manager animates it.
"""

import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("asr_sdm")
    model_xacro = os.path.join(pkg_share, "urdf", "asr_sdm_wrapper.urdf.xacro")
    robot_description = xacro.process_file(model_xacro).toxml()

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{
            "robot_description": robot_description,
            "use_sim_time": False,
        }],
        remappings=[
            ("joint_states", "/control/joint_states"),
        ],
    )

    return LaunchDescription([
        robot_state_publisher,
    ])
