#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    # Declare launch arguments
    port_arg = DeclareLaunchArgument(
        'port',
        default_value='/dev/ttyUSB0',
        description='Serial port for UWB device'
    )
    
    baud_arg = DeclareLaunchArgument(
        'baud',
        default_value='115200',
        description='Baud rate for serial communication'
    )
    
    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='map',
        description='Frame ID for published poses'
    )
    
    calibration_arg = DeclareLaunchArgument(
        'calibration',
        default_value='linear',
        description='Calibration type (linear, quadratic, cubic, none)'
    )
    
    z_sign_arg = DeclareLaunchArgument(
        'z_sign',
        default_value='1',
        description='Z-axis sign for position calculation'
    )
    
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value='',
        description='Path to YAML configuration file'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )

    # Get the package directory
    pkg_share = FindPackageShare('uwb_localizer')
    
    # Default config file path
    default_config_file = PathJoinSubstitution([
        pkg_share, 'config', 'uwb_localizer_params.yaml'
    ])
    
    # Use provided config file or default
    config_file = LaunchConfiguration('config_file')
    
    # UWB Localizer node
    uwb_localizer_node = Node(
        package='uwb_localizer',
        executable='uwb_localizer_ros2.py',
        name='uwb_localizer',
        output='screen',
        parameters=[
            default_config_file,
        ],
        remappings=[
            ('uwb/pose', 'uwb/pose'),
            ('uwb/ranges', 'uwb/ranges'),
            ('uwb/debug', 'uwb/debug'),
        ]
    )

    return LaunchDescription([
        port_arg,
        baud_arg,
        frame_id_arg,
        calibration_arg,
        z_sign_arg,
        config_file_arg,
        use_sim_time_arg,
        uwb_localizer_node,
    ])
