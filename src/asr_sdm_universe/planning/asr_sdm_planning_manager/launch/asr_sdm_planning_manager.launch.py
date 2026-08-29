#!/usr/bin/env python3
"""ROS 2 launch file for the topological replanning demo.

Port of topo_replan.launch / topo_algorithm.xml. Starts planning_manager_node (in
topological replanning mode) and traj_server. The external ROS 1 simulator stack
is not launched here; provide your own odometry / sensor sources.

Node parameters live in config/topo_replan.yaml.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("asr_sdm_planning_manager"),
        "config", "topo_replan.yaml")

    odom_topic_arg = DeclareLaunchArgument(
        "odom_topic", default_value="/visual_slam/odom",
        description="Topic of your odometry such as VIO or LIO")
    cmd_topic_arg = DeclareLaunchArgument(
        "cmd_topic", default_value="/control/asr_sdm/robot_cmd",
        description="Trajectory command topic consumed by asr_sdm_control_manager")
    odom_topic = LaunchConfiguration("odom_topic")
    cmd_topic = LaunchConfiguration("cmd_topic")

    planning_manager_node = Node(
        package="asr_sdm_planning_manager",
        executable="planning_manager_node",
        name="planning_manager_node",
        output="screen",
        parameters=[config],
        remappings=[
            ("odom", odom_topic),
            ("/esdf_map/odom", odom_topic),
            ("/esdf_map/cloud", "/pcl_render_node/cloud"),
            ("/esdf_map/pose", "/pcl_render_node/camera_pose"),
            ("/esdf_map/depth", "/pcl_render_node/depth"),
        ],
    )

    traj_server = Node(
        package="asr_sdm_planning_manager",
        executable="traj_server",
        name="traj_server",
        output="screen",
        parameters=[config],
        remappings=[
            ("/position_cmd", cmd_topic),
            ("odom", odom_topic),
        ],
    )

    return LaunchDescription([
        odom_topic_arg,
        cmd_topic_arg,
        planning_manager_node,
        traj_server,
    ])
