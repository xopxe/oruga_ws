from launch import LaunchDescription
from launch_ros.actions import Node

import os
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():

    pkg_project_bringup = get_package_share_directory('bringup')
    joy_params = PathJoinSubstitution(
        [pkg_project_bringup, "config", "joystick.yaml"]
    )

    # joy_node does not respect coalesce_interval_ms
    # https://github.com/ros-drivers/joystick_drivers/issues/284
    joy_node = Node(
        package='joy',
        executable='joy_node',
        output='screen',
        parameters=[joy_params],
        #arguments=['--ros-args', '--log-level', 'DEBUG'],
    )
    
    game_controller_node = Node(
        package='joy',
        executable='game_controller_node',
        output='screen',
        parameters=[joy_params],
    )
    
    teleop_node = Node(
        package='teleop_twist_joy', 
        executable='teleop_node',
        name = 'teleop_node',
        parameters=[joy_params],
        #remappings=[('/cmd_vel', '/oruga/cmd_vel')]
    )

    return LaunchDescription([
        #joy_node,
        game_controller_node,
        teleop_node,
    ])
    