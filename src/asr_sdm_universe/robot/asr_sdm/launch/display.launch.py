from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


PACKAGE_NAME = 'asr_sdm'


def generate_launch_description():
    model = LaunchConfiguration('model')
    use_sim_time = LaunchConfiguration('use_sim_time')
    gui = LaunchConfiguration('gui')
    rvizconfig = LaunchConfiguration('rvizconfig')

    package_share = FindPackageShare(PACKAGE_NAME)
    default_model = PathJoinSubstitution([
        package_share,
        'urdf',
        'asr_sdm_wrapper.urdf.xacro',
    ])
    default_rviz_config = PathJoinSubstitution([
        package_share,
        'rviz',
        'robot_description.rviz',
    ])

    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                package_share,
                'launch',
                'description.launch.py',
            ])
        ),
        launch_arguments={
            'model': model,
            'use_sim_time': use_sim_time,
        }.items(),
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        condition=UnlessCondition(gui),
        parameters=[{'use_sim_time': use_sim_time}],
    )

    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        condition=IfCondition(gui),
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # 该静态变换仅用于独立模型浏览，不会在 URDF 中固定机器人基座。
    world_to_robot = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world_to_asr_sdm',
        arguments=[
            '--frame-id', 'world',
            '--child-frame-id', 'screwdrive_segment_0',
            '--pitch', '1.5707963267948966',
        ],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rvizconfig],
        parameters=[{'use_sim_time': use_sim_time}],
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
        DeclareLaunchArgument(
            'gui',
            default_value='true',
            description='是否启动 joint_state_publisher_gui',
        ),
        DeclareLaunchArgument(
            'rvizconfig',
            default_value=default_rviz_config,
            description='RViz 配置文件的绝对路径',
        ),
        joint_state_publisher,
        joint_state_publisher_gui,
        robot_state_publisher,
        world_to_robot,
        rviz,
    ])
