#!/usr/bin/env python3
"""
ROS2 Launch file for SVO Visual Odometry - Live camera configuration.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml


def generate_launch_description():
    # Get the package share directory
    svo_ros_dir = get_package_share_directory('svo_ros')

    # Load camera + VO parameters from my_camera.yaml
    camera_yaml_path = os.path.join(svo_ros_dir, 'param', 'my_camera.yaml')
    with open(camera_yaml_path, 'r') as f:
        node_parameters = yaml.safe_load(f)

    # Combine with launch args
    node_parameters = {
        # Camera topic to subscribe to (override via launch argument)
        'cam_topic': LaunchConfiguration('cam_topic'),
        **node_parameters,
    }

    # Declare launch arguments
    cam_topic_arg = DeclareLaunchArgument(
        'cam_topic',
        default_value='/camera/image_raw',
        description='Camera topic to subscribe to'
    )

    svo_node = Node(
        package='svo_ros',
        executable='vo',
        name='svo',
        output='screen',
        parameters=[node_parameters],
    )

    return LaunchDescription([
        cam_topic_arg,
        svo_node,
    ])


