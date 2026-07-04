from launch import LaunchDescription
from launch.actions import LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import importlib.util
import os
import sys


def _load_vins_launch_module():
    vins_share = get_package_share_directory('vins_estimator')
    launch_path = os.path.join(vins_share, 'launch', 'vins_launch.py')
    spec = importlib.util.spec_from_file_location('vins_launch', launch_path)
    module = importlib.util.module_from_spec(spec)
    sys.modules['vins_launch'] = module
    spec.loader.exec_module(module)
    return module


def _launch_setup(context, *args, **kwargs):
    vins_launch = _load_vins_launch_module()
    vins_share = get_package_share_directory('vins_estimator')
    config_pkg_path = get_package_share_directory('config_pkg')
    launch_config_name = context.perform_substitution(
        LaunchConfiguration('vins_launch_config'))
    vins_config = vins_launch.load_vins_config(vins_share, launch_config_name)

    common_params, log_params, log_calib, enable_sparse = vins_launch.build_common_params(
        context, config_pkg_path, LaunchConfiguration, vins_config)

    rviz_config_path = os.path.join(
        config_pkg_path, vins_config.get('rviz_config', 'config/vins_euroc_rviz.rviz'))

    return [
        LogInfo(msg=[f'[feature tracker launch] launch config: {launch_config_name}']),
        LogInfo(msg=[f'[feature tracker launch] params file: {log_params}']),
        LogInfo(msg=[f'[feature tracker launch] calibration file: {log_calib}']),
        LogInfo(msg=[f'[feature tracker launch] sparse alignment override: {"ON" if enable_sparse else "OFF"}']),
        Node(
            package='feature_tracker',
            executable='feature_tracker_node',
            name='feature_tracker',
            namespace='feature_tracker',
            output='screen',
            parameters=common_params,
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_path],
            output='screen',
        ),
    ]


def generate_launch_description():
    vins_launch = _load_vins_launch_module()
    vins_share = get_package_share_directory('vins_estimator')
    default_launch_config = 'vins.yaml'
    vins_config = vins_launch.load_vins_config(vins_share, default_launch_config)

    return LaunchDescription(
        vins_launch.declare_vins_config_args(vins_config, default_launch_config)
        + [OpaqueFunction(function=_launch_setup)]
    )
