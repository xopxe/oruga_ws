# Copyright 2020 ros2_control Development Team
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "gui",
            default_value="false",
            description="Start RViz2 automatically with this launch file.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "joints_gui",
            default_value="false",
            description="Start joint_state_publisher_gui automatically with this launch file.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_lidar',
            default_value="false",
            description='Enable the lidar sensor.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_camera',
            default_value="false",
            description='Enable the camera sensor.',
        )
    )
    # Initialize Arguments
    gui = LaunchConfiguration("gui")
    joints_gui = LaunchConfiguration("joints_gui")
    use_lidar = LaunchConfiguration("use_lidar")
    use_camera = LaunchConfiguration("use_camera")

    # Setup project paths
    pkg_project_bringup = get_package_share_directory('bringup')
    pkg_project_description = get_package_share_directory('description')


    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [pkg_project_description, 'models', 'oruga', "oruga.urdf.xacro"]
            ),
            " ",
            "use_lidar:=",use_lidar,
            " ",
            "use_camera:=",use_camera,        ]
    )

    rviz_config_file = PathJoinSubstitution(
        [pkg_project_bringup, 'config', "oruga.rviz"]
    )

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[
            {"robot_description": robot_description_content},
            {'frame_prefix': ''}
        ],
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(gui),
    )
       
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        condition=IfCondition(joints_gui),
    )
 
    lidar_node = Node(
        package='hls_lfcd_lds_driver',
        executable='hlds_laser_publisher',
        output='screen',
        parameters=[{
            'port': '/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0',
            'frame_id': 'lidar_link',
        }],
        condition=IfCondition(use_lidar),
    )
    
    camera_node = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        output='screen',
        namespace='camera',
        parameters=[{
            'image_size': [640,480],
            'time_per_frame': [1, 6],
            'frame_id': 'camera_link_optical',
        }],
        condition=IfCondition(use_camera),
    )

    nodes = [
        rviz_node,
        lidar_node,
        camera_node,
        robot_state_pub_node,
        joint_state_publisher_gui,
    ]

    return LaunchDescription(declared_arguments + nodes)

