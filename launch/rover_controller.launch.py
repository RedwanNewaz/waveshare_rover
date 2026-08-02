#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='waver',
        description='Namespace to launch the joystick, teleop, and rover controller nodes under.'
    )

    ip_address_arg = DeclareLaunchArgument(
        'ip_address',
        default_value='192.168.10.148',
        description='IP address the rover controller node sends HTTP drive commands to.'
    )

    namespace = LaunchConfiguration('namespace')
    ip_address = LaunchConfiguration('ip_address')

    # enable joystick controller
    joy_node = Node(
        name='joy_node',
        package="joy",
        executable="joy_node",
        namespace=namespace
    )

    teleop_node = Node(
        name = 'teleop_node',
        package = "teleop_twist_joy",
        executable="teleop_node",
        namespace=namespace
    )

    # Rover controller node
    rover_controller_node = Node(
        package='rover_controller',
        executable='rover_controller_node',
        name='rover_controller_node',
        namespace=namespace,
        output='screen',
        parameters=[{'ip_address': ip_address}]
    )

    return LaunchDescription([
        namespace_arg,
        ip_address_arg,
        joy_node,
        teleop_node,
        rover_controller_node
    ])