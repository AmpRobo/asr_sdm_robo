from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


PACKAGE_NAME = 'asr_sdm'


def generate_launch_description():
    model = LaunchConfiguration('model')
    use_sim_time = LaunchConfiguration('use_sim_time')

    default_model = PathJoinSubstitution([
        FindPackageShare(PACKAGE_NAME),
        'urdf',
        'asr_sdm_wrapper.urdf.xacro',
    ])

    robot_description = ParameterValue(
        Command(['xacro ', model]),
        value_type=str,
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time,
        }],
        output='screen',
    )

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
        robot_state_publisher,
    ])
