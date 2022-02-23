import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python import get_package_share_directory


def generate_launch_description():
    launch_description = LaunchDescription()

    # ------------
    # Launch Files
    # ------------

    control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('art_launch'),
                'launch/control.launch.py'))
    )
    launch_description.add_action(control)

    path_planning = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('art_launch'),
                'launch/path_planning.launch.py'))
    )
    launch_description.add_action(path_planning)

    cone_detection = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('art_launch'),
                'launch/cone_detection.launch.py')),
    )
    launch_description.add_action(cone_detection)

    return launch_description
