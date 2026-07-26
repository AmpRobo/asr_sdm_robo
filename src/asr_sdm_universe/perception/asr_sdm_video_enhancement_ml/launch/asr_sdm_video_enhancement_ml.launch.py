import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory("asr_sdm_video_enhancement_ml")
    default_model = os.path.join(package_share, "models", "uieb_model.onnx")

    return LaunchDescription(
        [
            DeclareLaunchArgument("model_path", default_value=default_model),
            DeclareLaunchArgument("input_topic", default_value="/sensing/camera/camera0/image_raw/compressed"),
            DeclareLaunchArgument("output_topic", default_value="/perception/video_enhancement/image_enhanced/compressed"),
            DeclareLaunchArgument("normalize_output", default_value="true"),
            Node(
                package="asr_sdm_video_enhancement_ml",
                executable="asr_sdm_video_enhancement_ml_node",
                name="asr_sdm_video_enhancement_ml_node",
                output="screen",
                parameters=[
                    {
                        "model_path": LaunchConfiguration("model_path"),
                        "input_topic": LaunchConfiguration("input_topic"),
                        "output_topic": LaunchConfiguration("output_topic"),
                        "normalize_output": LaunchConfiguration("normalize_output"),
                    }
                ],
            ),
        ]
    )
