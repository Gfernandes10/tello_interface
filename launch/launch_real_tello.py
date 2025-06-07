"""Simulate a Tello drone"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess


def generate_launch_description():
    ns = 'drone1'
    world_path = os.path.join(get_package_share_directory('tello_gazebo'), 'worlds', 'simple.world')
    urdf_path = os.path.join(get_package_share_directory('tello_description'), 'urdf', 'tello_1.urdf')

    return LaunchDescription([

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
