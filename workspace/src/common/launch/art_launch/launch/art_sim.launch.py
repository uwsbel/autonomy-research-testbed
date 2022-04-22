# ROS imports
from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    ld = LaunchDescription()

    # ------------
    # Launch Files
    # ------------

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('art_simulation_launch'),
                'launch',
                'art_simulation.launch.py'
            ])
        ])
    )
    ld.add_action(simulation)

    return ld
