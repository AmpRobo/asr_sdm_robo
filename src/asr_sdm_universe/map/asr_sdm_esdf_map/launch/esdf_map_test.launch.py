import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('asr_sdm_esdf_map')
    config = os.path.join(package_share, 'config', 'esdf_map_config.yaml')
    rviz = os.path.join(package_share, 'test', 'esdf_map_test.rviz')

    return LaunchDescription([
        Node(
            package='asr_sdm_esdf_map',
            executable='esdf_map_test',
            name='esdf_map',
            output='screen',
            # The YAML file is the single source of truth for every map and
            # test-node parameter. Do not add a second parameter dictionary
            # here, because later entries would override values from YAML.
            parameters=[config],
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
