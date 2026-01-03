"""Simulate a Tello drone"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    ns = 'tello'
    # launch-configurable service names (defaults use this node namespace)
    takeoff_service = LaunchConfiguration('takeoff_service', default=f'/{ns}/takeoff')
    land_service = LaunchConfiguration('land_service', default=f'/{ns}/land')
    tello_action_service = LaunchConfiguration('tello_action_service', default=f'/{ns}/tello_action')
    command_topic = LaunchConfiguration('command_topic', default=f'/{ns}/cmd_vel')
    filtered_pose_topic = LaunchConfiguration('filtered_pose_topic', default=f'/{ns}/filtered_pose')

    world_path = os.path.join(get_package_share_directory('tello_gazebo'), 'worlds', 'simple.world')
    urdf_path = os.path.join(get_package_share_directory('tello_description'), 'urdf', 'tello_1.urdf')

    return LaunchDescription([
        # allow overriding service names from the command line
        DeclareLaunchArgument('takeoff_service', default_value=f'/{ns}/takeoff', description='Takeoff service name'),
        DeclareLaunchArgument('land_service', default_value=f'/{ns}/land', description='Land service name'),
        DeclareLaunchArgument('tello_action_service', default_value=f'/{ns}/tello_action', description='Tello action service name'),
        DeclareLaunchArgument('command_topic', default_value=f'/{ns}/cmd_vel', description='Command (cmd_vel) topic'),
        DeclareLaunchArgument('filtered_pose_topic', default_value=f'/{ns}/filtered_pose', description='Filtered pose topic'),

        # Spawn tello.urdf
        Node(package='dji_tello_driver', executable='tello_node', output='screen'),

        Node(
            package='plotjuggler',
            executable='plotjuggler',
            output='screen',
            arguments=[
                '--plugin', 'Ros2',
            ]
        ),

        Node(
            package='tello_interface',
            executable='tello_interface_node',
            output='screen',
            arguments=['--gui'],
            parameters=[{
                'takeoff_service_name': takeoff_service,
                'land_service_name': land_service,
                'tello_action_service_name': tello_action_service,
                'command_topic': command_topic,
                'filtered_pose_topic': filtered_pose_topic,
            }],
        ),

        ExecuteProcess(
        cmd=[
            'rqt',
            '--perspective-file',
            os.path.join(get_package_share_directory('tello_interface'), 'plotjuggler', 'tello.perspective')
        ],
        output='screen'
        ),
    ])
