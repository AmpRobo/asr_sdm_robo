"""Launch the simulated map generator.

    ros2 launch asr_sdm_map_generator asr_sdm_map_generator_launch.py map_type:=maze2d
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

MAP_TYPES = ["random_forest", "perlin3d", "post2d", "maze2d", "maze3d"]


def generate_launch_description():
    pkg_share = get_package_share_directory("asr_sdm_map_generator")
    config = os.path.join(pkg_share, "config", "asr_sdm_map_generator.yaml")

    map_type_arg = DeclareLaunchArgument(
        "map_type",
        default_value="random_forest",
        choices=MAP_TYPES,
        description="Map generator; per-type settings live in config/asr_sdm_map_generator.yaml")

    random_forest_node = Node(
        package="asr_sdm_map_generator",
        executable="random_forest_test",
        name="random_map_sensing",
        output="screen",
        emulate_tty=True,
        parameters=[config, {"map.type": LaunchConfiguration("map_type")}],
    )

    return LaunchDescription([map_type_arg, random_forest_node])
