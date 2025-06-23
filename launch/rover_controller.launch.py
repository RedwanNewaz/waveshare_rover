#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare launch arguments
    joy_config_arg = DeclareLaunchArgument(
        'joy_config',
        default_value='xbox',
        description='Joystick configuration (ps3, ps4, xbox, etc.)'
    )
    
    device_name_arg = DeclareLaunchArgument(
        'device_name',
        default_value='',
        description='Joy device name (empty for default)'
    )
    # Include teleop_twist_joy launch file with configurable joy config
    teleop_twist_joy_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('teleop_twist_joy'),
                'launch',
                'teleop-launch.py'
            ])
        ]),
        launch_arguments={
            'joy_config': LaunchConfiguration('joy_config')
        }.items()
    )
    
    # Joy node with configurable device
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        parameters=[{
            'device_name': LaunchConfiguration('device_name'),
            'autorepeat_rate': 20.0,
            'deadzone': 0.05
        }]
    )
    
    # Rover controller node
    rover_controller_node = Node(
        package='rover_controller',
        executable='rover_controller_node',
        name='rover_controller_node',
        output='screen'
    )
    
    return LaunchDescription([
        joy_config_arg,
        # device_name_arg,
        teleop_twist_joy_launch,
        # joy_node,
        rover_controller_node
    ])