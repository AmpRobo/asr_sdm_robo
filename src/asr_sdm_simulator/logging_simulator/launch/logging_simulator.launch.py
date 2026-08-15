#!/usr/bin/env python3
"""Launch the logging simulator stack.

Node parameters live in config/logging_simulator.yaml.

    ros2 launch logging_simulator logging_simulator.launch.py
    ros2 launch logging_simulator logging_simulator.launch.py control:=enable
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _enabled(flag: LaunchConfiguration) -> IfCondition:
    return IfCondition(PythonExpression(["'", flag, "' == 'enable'"]))


def generate_launch_description():
    pkg_share = get_package_share_directory("logging_simulator")
    rviz_config = os.path.join(pkg_share, "config", "rviz.rviz")

    robot_model = LaunchConfiguration("robot_model")
    control = LaunchConfiguration("control")
    teleop = LaunchConfiguration("teleop")
    planning = LaunchConfiguration("planning")

    vins_odom = "/localization/video_inertial_navigation_systems/odometry"

    robot_model_arg = DeclareLaunchArgument(
        "robot_model",
        default_value="asr_sdm",
        description="Robot model package; must provide launch/<name>_description.launch.py")
    control_arg = DeclareLaunchArgument(
        "control",
        default_value="disable",
        choices=["enable", "disable"],
        description="Start asr_sdm_control_manager kinematic controller")
    teleop_arg = DeclareLaunchArgument(
        "teleop",
        default_value="disable",
        choices=["enable", "disable"],
        description="Start asr_sdm_teleop gamepad teleop chain")
    planning_arg = DeclareLaunchArgument(
        "planning",
        default_value="disable",
        choices=["enable", "disable"],
        description="Start asr_sdm_planning_manager planning chain")

    odom_visualization = Node(
        package="odom_visualization",
        executable="odom_visualization",
        name="odom_visualization_ukf",
        output="screen",
        parameters=[{
            "color.a": 0.8,
            "color.r": 1.0,
            "color.g": 0.0,
            "color.b": 0.0,
            "covariance_scale": 100.0,
            "odom_topics": [vins_odom],
            "tf45": True,
            "stamp_tf_with_now": True,
            "initial_x": -5.0,
            "initial_y": 0.0,
            "initial_z": 0.0,
            "initial_yaw": 0.0,
            "initial_pitch": 0.0,
            "initial_roll": 0.0,
            "tf_yaw": 3.141592653589793,
            "tf_pitch": -1.5707963267948966,
            "tf_roll": 0.0,
            "tf_publish_rate": 50.0,
        }],
    )

    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        output="screen",
        parameters=[{"rate": 30.0}],
        remappings=[("joint_states", "/control/joint_states")],
        condition=IfCondition(PythonExpression(["'", control, "' == 'disable'"])),
    )

    robot_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare(robot_model),
                "launch",
            ]),
            "/",
            robot_model,
            "_description.launch.py",
        ]),
    )

    control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("asr_sdm_control_manager"),
                "launch", "asr_sdm_control_manager.launch.py")),
        condition=_enabled(control),
    )
    teleop_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("asr_sdm_teleop"),
                "launch", "asr_sdm_teleop.launch.py")),
        condition=_enabled(teleop),
    )
    planning_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("asr_sdm_planning_manager"),
                "launch", "asr_sdm_planning_manager.launch.py")),
        launch_arguments={"odom_topic": vins_odom}.items(),
        condition=_enabled(planning),
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz",
        arguments=["-d", rviz_config],
    )

    return LaunchDescription([
        robot_model_arg,
        control_arg,
        teleop_arg,
        planning_arg,
        odom_visualization,
        joint_state_publisher,
        robot_description,
        control_launch,
        teleop_launch,
        planning_launch,
        rviz,
    ])
