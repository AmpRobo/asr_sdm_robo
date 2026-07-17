import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_share = get_package_share_directory('asr_sdm_esdf_map')
    config = os.path.join(package_share, 'config', 'esdf_map_config.yaml')
    rviz = os.path.join(package_share, 'test', 'esdf_map_test.rviz')

    enable_depth_odom = LaunchConfiguration('enable_depth_odom')
    enable_pointcloud_odom = LaunchConfiguration('enable_pointcloud_odom')

    return LaunchDescription([
        DeclareLaunchArgument(
            'enable_depth_odom',
            default_value='false',
            description='Enable the synchronized Depth + Odometry input chain.',
        ),
        DeclareLaunchArgument(
            'enable_pointcloud_odom',
            default_value='false',
            description='Enable the PointCloud + Odometry input chain.',
        ),
        Node(
            package='asr_sdm_esdf_map',
            executable='esdf_map_test',
            name='esdf_map',
            output='screen',
            parameters=[
                config,
                {
                    'esdf_map.enable_depth_odom': ParameterValue(
                        enable_depth_odom, value_type=bool),
                    'esdf_map.enable_pointcloud_odom': ParameterValue(
                        enable_pointcloud_odom, value_type=bool),
                },
            ],
        ),
        # RViz reports "Fixed Frame: No tf data" when no sensor stack is
        # running. This identity transform makes the world frame explicit in
        # preload-only tests without changing any map coordinates.
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='esdf_map_static_tf',
            output='screen',
            arguments=[
                '--x', '0', '--y', '0', '--z', '0',
                '--yaw', '0', '--pitch', '0', '--roll', '0',
                '--frame-id', 'world',
                '--child-frame-id', 'esdf_map_visualization',
            ],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz],
        ),
    ])
