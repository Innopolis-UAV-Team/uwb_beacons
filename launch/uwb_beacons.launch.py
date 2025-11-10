#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """Generate launch description for precision landing."""

    # Get package directory
    pkg_dir = get_package_share_directory('uwb_beacons')
    config_file = os.path.join(pkg_dir, 'config', 'uwb_beacons_params.yaml')

    # Precision lander node
    precision_lander_node = Node(
        package='uwb_beacons',
        executable='uwb_localizer_node',
        name='precision_lander_node',
        parameters=[config_file],
        output='screen',
        emulate_tty=True
    )

    return LaunchDescription([
        precision_lander_node
    ])
