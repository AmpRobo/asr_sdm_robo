from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


PACKAGE_NAME = 'asr_sdm'


def launch_setup(context):
    model_path = Path(LaunchConfiguration('model').perform(context))

    if model_path.suffix == '.xacro':
        import xacro

        robot_description = xacro.process_file(str(model_path)).toxml()
    else:
        robot_description = model_path.read_text(encoding='utf-8')

    return [Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
        output='screen',
    )]


def generate_launch_description():
    default_model = PathJoinSubstitution([
        FindPackageShare(PACKAGE_NAME),
        'urdf',
        'generated',
        'asr_sdm_segments_4.urdf',
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'model',
            default_value=default_model,
            description='机器人 URDF 或 Xacro 文件的绝对路径',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='是否使用仿真时钟',
        ),
        OpaqueFunction(function=launch_setup),
    ])
