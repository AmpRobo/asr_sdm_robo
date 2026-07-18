import math
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('planning_simulator')
    config_path = os.path.join(pkg_share, 'config', 'planning_simulator.yaml')
    rviz_config = os.path.join(pkg_share, 'config', 'rviz.rviz')
    map_generator_config = os.path.join(
        get_package_share_directory('asr_sdm_map_generator'),
        'config', 'asr_sdm_map_generator.yaml')
    asr_sdm_share = get_package_share_directory('asr_sdm')
    asr_sdm_model = os.path.join(asr_sdm_share, 'urdf', 'asr_sdm_wrapper.urdf.xacro')
    asr_sdm_description_launch = os.path.join(
        asr_sdm_share, 'launch', 'description.launch.py')

    with open(config_path, 'r', encoding='utf-8') as f:
        cfg = yaml.safe_load(f) or {}

    use_map_generator = bool(cfg.get('use_asr_sdm_map_generator', True))
    use_asr_sdm_model = bool(cfg.get('use_asr_sdm_model', True))
    rate_odom = float(cfg.get('rate', {}).get('odom', 100.0))
    sim_cfg = cfg.get('simulator', {})
    init_x = float(sim_cfg.get('init_state_x', -5.0))
    init_y = float(sim_cfg.get('init_state_y', 0.0))
    init_z = float(sim_cfg.get('init_state_z', 3.0))

    # Random forest map generator (publishes /asr_sdm_map_generator/global_cloud)
    random_map_sensing = Node(
        package='asr_sdm_map_generator',
        executable='random_forest_test',
        name='random_map_sensing',
        output='screen',
        emulate_tty=True,
        condition=IfCondition('true' if use_map_generator else 'false'),
        parameters=[
            map_generator_config,
            {
                'init_state_x': init_x,
                'init_state_y': init_y,
            },
        ],
        remappings=[
            ('odometry', '/visual_slam/odom'),
        ],
    )

    # Simulator
    planning_simulator = Node(
        package='planning_simulator',
        executable='planning_simulator',
        name='planning_simulator',
        output='screen',
        parameters=[{
            'rate.odom': rate_odom,
            'simulator.init_state_x': init_x,
            'simulator.init_state_y': init_y,
            'simulator.init_state_z': init_z,
        }],
        remappings=[
            ('odom', '/visual_slam/odom'),
            ('cmd', 'so3_cmd'),
            ('imu', 'sim/imu'),
            ('force_disturbance', 'force_disturbance'),
            ('moment_disturbance', 'moment_disturbance'),
            ('initialpose', '/initialpose'),
        ],
    )

    # Controller (ROS 1 nodelet -> rclcpp component executable)
    so3_control = Node(
        package='so3_control',
        executable='so3_control_node',
        name='so3_control',
        output='screen',
        parameters=[{
            'mass': 0.98,
            'use_angle_corrections': False,
            'use_external_yaw': False,
            # gains from config/gains_hummingbird.yaml, with launch overrides
            'gains.rot.x': 1.0,
            'gains.rot.y': 1.0,
            'gains.rot.z': 1.0,
            'gains.ang.x': 0.07,
            'gains.ang.y': 0.07,
            'gains.ang.z': 0.1,
            # corrections from config/corrections_hummingbird.yaml
            'corrections.z': 0.0,
            'corrections.r': 0.0,
            'corrections.p': 0.0,
        }],
        remappings=[
            ('odom', '/state_ukf/odom'),
            ('position_cmd', 'position_cmd'),
            ('motors', 'motors'),
            ('corrections', 'corrections'),
            ('so3_cmd', 'so3_cmd'),
            ('imu', 'sim/imu'),
        ],
    )

    so3_disturbance_generator = Node(
        package='so3_disturbance_generator',
        executable='so3_disturbance_generator',
        name='so3_disturbance_generator',
        output='screen',
        remappings=[
            ('odom', '/visual_slam/odom'),               # Ground Truth & Fake VSLAM Odom
            ('noisy_odom', '/state_ukf/odom'),           # Fake VINS Odom
            ('correction', '/visual_slam/correction'),   # Fake VSLAM Correction
            ('force_disturbance', 'force_disturbance'),
            ('moment_disturbance', 'moment_disturbance'),
        ],
    )

    # Visualization
    odom_visualization_ukf = Node(
        package='odom_visualization',
        executable='odom_visualization',
        name='odom_visualization_ukf',
        output='screen',
        parameters=[{
            'color.a': 0.8,
            'color.r': 1.0,
            'color.g': 0.0,
            'color.b': 0.0,
            'covariance_scale': 100.0,
            # Publish world->base TF so the asr_sdm model can follow odometry.
            'tf45': use_asr_sdm_model,
        }],
        remappings=[
            ('odom', '/visual_slam/odom'),
        ],
    )

    # asr_sdm robot model (robot_state_publisher + joint states + mesh TF offset)
    asr_sdm_robot_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(asr_sdm_description_launch),
        condition=IfCondition('true' if use_asr_sdm_model else 'false'),
        launch_arguments={
            'model': asr_sdm_model,
            'use_sim_time': 'false',
        }.items(),
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        condition=IfCondition('true' if use_asr_sdm_model else 'false'),
    )

    # Mesh is modeled along +Z; pitch +90 deg lays the body in the XY plane.
    # Yaw +180 deg aligns the visual heading with odom / 2D Pose Estimate.
    base_to_asr_sdm = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_asr_sdm',
        condition=IfCondition('true' if use_asr_sdm_model else 'false'),
        arguments=[
            '--frame-id', 'base',
            '--child-frame-id', 'screwdrive_segment_0',
            '--yaw', str(math.pi),
            '--pitch', str(math.pi / 2.0),
        ],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', rviz_config],
    )

    return LaunchDescription([
        random_map_sensing,
        planning_simulator,
        so3_control,
        so3_disturbance_generator,
        odom_visualization_ukf,
        asr_sdm_robot_description,
        joint_state_publisher,
        base_to_asr_sdm,
        rviz,
    ])
