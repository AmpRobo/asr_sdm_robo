import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory("asr_sdm_video_enhancement_ml")
    default_model = os.path.join(package_share, "models", "five_aplus_epoch97.onnx")
    onnxruntime_root = os.environ.get("ONNXRUNTIME_ROOT")
    if not onnxruntime_root:
        raise RuntimeError(
            "ONNXRUNTIME_ROOT must be set to the ONNX Runtime installation root."
        )

    onnxruntime_lib = os.path.join(onnxruntime_root, "lib")
    if not os.path.isdir(onnxruntime_lib):
        raise RuntimeError(
            f"ONNX Runtime library directory does not exist: {onnxruntime_lib}"
        )

    existing_ld_library_path = os.environ.get("LD_LIBRARY_PATH", "")
    ld_library_path = (
        onnxruntime_lib
        if not existing_ld_library_path
        else onnxruntime_lib + ":" + existing_ld_library_path
    )

    return LaunchDescription(
        [
            SetEnvironmentVariable("LD_LIBRARY_PATH", ld_library_path),
            DeclareLaunchArgument("model_path", default_value=default_model),
            DeclareLaunchArgument("input_topic", default_value="/camera/camera/color/image_raw"),
            DeclareLaunchArgument("output_topic", default_value="/asr_sdm_video_enhancement_ml/image"),
            DeclareLaunchArgument("num_threads", default_value="0"),
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
                        "num_threads": LaunchConfiguration("num_threads"),
                        "normalize_output": LaunchConfiguration("normalize_output"),
                    }
                ],
            ),
        ]
    )
