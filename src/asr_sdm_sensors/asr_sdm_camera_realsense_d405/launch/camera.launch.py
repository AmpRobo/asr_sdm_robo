import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


_EMPTY_VALUES = ('', "''", '""')


def _launch_setup(context, *args, **kwargs):
    package_share_dir = get_package_share_directory('asr_sdm_camera_realsense_d405')
    realsense_share_dir = get_package_share_directory('realsense2_camera')

    config_file_arg = LaunchConfiguration('config_file').perform(context).strip()

    if config_file_arg not in _EMPTY_VALUES:
        config_file = config_file_arg
    else:
        config_file = os.path.join(package_share_dir, 'config', 'd435if.yaml')

    if not os.path.exists(config_file):
        raise RuntimeError(
            f'RealSense config file not found: {config_file}. '
            f'Pass config_file:=/absolute/path/to/file.yaml'
        )

    official_launch_file = os.path.join(realsense_share_dir, 'launch', 'rs_launch.py')

    return [
        LogInfo(msg=f'Using RealSense config file: {config_file}'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(official_launch_file),
            launch_arguments={
                'config_file': config_file,
            }.items(),
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value='',
            description=(
                'Optional absolute path to a RealSense flat YAML config file. '
                'If not set, config/d435if.yaml is used.'
            ),
        ),
        OpaqueFunction(function=_launch_setup),
    ])