import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription

from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='dso',
            node_executable='camera',
            output='screen'),
        Node(
            package='dso',
            node_executable='dso_ros',
            output='screen'),
    ])
