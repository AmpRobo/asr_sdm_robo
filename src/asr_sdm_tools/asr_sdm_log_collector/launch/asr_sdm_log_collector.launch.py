from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PACKAGE_NAME = 'asr_sdm_log_collector'

# Launch arguments that override a single parameter, keyed by the parameter the
# node declares. An empty value means "leave whatever the parameter file says",
# which is why these are resolved in an OpaqueFunction: a declarative dict would
# always be applied and would overwrite the file with the argument default.
OVERRIDES = {
    'log_directory': ('sink.log_directory', str),
    'log_filename': ('sink.log_filename', str),
    'socket_path': ('unix_socket.socket_path', str),
    'echo_to_console': ('sink.echo_to_console', bool),
}


def _launch_setup(context, *args, **kwargs) -> list:
    parameters: list = [LaunchConfiguration('config_file').perform(context)]

    overrides = {}
    for argument_name, (parameter_name, parameter_type) in OVERRIDES.items():
        value = LaunchConfiguration(argument_name).perform(context)
        if not value:
            continue
        if parameter_type is bool:
            overrides[parameter_name] = value.lower() in ('true', '1', 'yes')
        else:
            overrides[parameter_name] = value
    if overrides:
        parameters.append(overrides)

    return [
        Node(
            package=PACKAGE_NAME,
            executable='asr_sdm_log_collector_node',
            name='asr_sdm_log_collector',
            output='screen',
            emulate_tty=True,
            parameters=parameters,
        ),
    ]


def generate_launch_description() -> LaunchDescription:
    default_config = os.path.join(
        get_package_share_directory(PACKAGE_NAME),
        'config',
        'asr_sdm_log_collector.yaml',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value=default_config,
            description='Parameter file for the collector node.',
        ),
        DeclareLaunchArgument(
            'log_directory',
            default_value='',
            description='Override sink.log_directory. Accepts ~ and $VAR.',
        ),
        DeclareLaunchArgument(
            'log_filename',
            default_value='',
            description='Override sink.log_filename, the merged log file.',
        ),
        DeclareLaunchArgument(
            'socket_path',
            default_value='',
            description='Override unix_socket.socket_path. Useful on a '
                        'development machine, where /run is not writable.',
        ),
        DeclareLaunchArgument(
            'echo_to_console',
            default_value='',
            description='Override sink.echo_to_console to mirror every '
                        'collected record to the collector stdout.',
        ),
        OpaqueFunction(function=_launch_setup),
    ])
