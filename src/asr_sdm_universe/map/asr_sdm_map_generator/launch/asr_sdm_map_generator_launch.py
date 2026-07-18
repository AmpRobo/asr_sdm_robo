from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory("asr_sdm_map_generator")
    config = os.path.join(pkg_share, "config", "asr_sdm_map_generator.yaml")

    random_forest_node = Node(
        package="asr_sdm_map_generator",
        executable="random_forest_test",
        name="random_map_sensing",
        output="screen",
        emulate_tty=True,
        parameters=[config],
    )

    return LaunchDescription([random_forest_node])
