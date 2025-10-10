"""Launch the ARIAC MVP smoke test pipeline."""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory('ariac_team')
    default_team_config = os.path.join(pkg_share, 'config', 'ariac_team_config.yaml')
    default_trial_config = os.path.join(pkg_share, 'config', 'trials', 'mvp_smoke.yaml')

    team_config = LaunchConfiguration('team_config')
    trial_config = LaunchConfiguration('trial_config')

    return LaunchDescription([
        DeclareLaunchArgument(
            'team_config',
            default_value=default_team_config,
            description='Path to the team configuration yaml',
        ),
        DeclareLaunchArgument(
            'trial_config',
            default_value=default_trial_config,
            description='Path to the trial configuration yaml',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('ariac_gz'), 'launch', 'ariac.launch.py')
            ),
            launch_arguments={
                'user_config': team_config,
                'trial_config': trial_config,
            }.items(),
        ),
        Node(
            package='ariac_team',
            executable='mvp_coordinator',
            output='screen',
        ),
    ])
