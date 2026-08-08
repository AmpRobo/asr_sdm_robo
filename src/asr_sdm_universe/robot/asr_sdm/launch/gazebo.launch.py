from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


PACKAGE_NAME = 'asr_sdm'


def _load_robot_description(model_path: Path) -> str:
    if model_path.suffix == '.xacro':
        import xacro

        return xacro.process_file(str(model_path)).toxml()
    return model_path.read_text(encoding='utf-8')


def description_setup(context):
    model_path = Path(LaunchConfiguration('model').perform(context))
    robot_description = _load_robot_description(model_path)

    return [Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        condition=UnlessCondition(LaunchConfiguration('use_rviz')),
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
        remappings=[
            ('joint_states', LaunchConfiguration('joint_states_topic')),
        ],
        output='screen',
    )]


def generate_launch_description():
    model = LaunchConfiguration('model')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')
    use_custom_world = LaunchConfiguration('use_custom_world')
    custom_world = LaunchConfiguration('custom_world')
    gazebo_world = LaunchConfiguration('gazebo_world')

    package_share = FindPackageShare(PACKAGE_NAME)
    default_model = PathJoinSubstitution([
        package_share,
        'urdf',
        'generated',
        'asr_sdm_segments_4.urdf',
    ])
    custom_world_path = PathJoinSubstitution([
        package_share,
        'worlds',
        custom_world,
    ])

    ros_gz_sim_launch = PathJoinSubstitution([
        FindPackageShare('ros_gz_sim'),
        'launch',
        'gz_sim.launch.py',
    ])

    package_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(ros_gz_sim_launch),
        condition=IfCondition(use_custom_world),
        launch_arguments={
            'gz_args': ['-r ', custom_world_path, ' --verbose'],
        }.items(),
    )

    external_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(ros_gz_sim_launch),
        condition=UnlessCondition(use_custom_world),
        launch_arguments={
            'gz_args': ['-r ', gazebo_world, ' --verbose'],
        }.items(),
    )

    display = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                package_share,
                'launch',
                'display.launch.py',
            ])
        ),
        condition=IfCondition(use_rviz),
        launch_arguments={
            'model': model,
            'use_sim_time': use_sim_time,
            'gui': 'false',
        }.items(),
    )

    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', PACKAGE_NAME,
            '-x', '1.2',
            '-z', '1.0',
            '-Y', '3.4',
            '-topic', '/robot_description',
        ],
        output='screen',
    )

    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    # Gazebo 和 sdformat 都需要从安装空间的 share 目录解析 package:// URI。
    share_parent = str(Path(get_package_share_directory(PACKAGE_NAME)).parent)

    return LaunchDescription([
        DeclareLaunchArgument(
            'model',
            default_value=default_model,
            description='机器人 URDF 或 Xacro 文件的绝对路径',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='是否使用 Gazebo 仿真时钟',
        ),
        DeclareLaunchArgument(
            'joint_states_topic',
            default_value='/control/joint_states',
            description='robot_state_publisher 订阅的关节状态 topic',
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='false',
            description='是否同时启动 RViz 模型浏览器',
        ),
        DeclareLaunchArgument(
            'use_custom_world',
            default_value='true',
            description='是否加载本包 worlds 目录中的 world',
        ),
        DeclareLaunchArgument(
            'custom_world',
            default_value='empty.sdf',
            description='本包 worlds 目录中的 SDF 文件名',
        ),
        DeclareLaunchArgument(
            'gazebo_world',
            default_value='empty.sdf',
            description='ros_gz_sim 可直接解析的 world 名称或路径',
        ),
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', share_parent),
        AppendEnvironmentVariable('SDF_PATH', share_parent),
        OpaqueFunction(function=description_setup),
        display,
        package_world,
        external_world,
        spawn,
        clock_bridge,
    ])
