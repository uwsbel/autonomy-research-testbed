import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python import get_package_share_directory


def generate_launch_description():

    control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('miniav_launch'),
                'launch/control.launch.py'))
    )

    path_planning = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('miniav_launch'),
                'launch/path_planning.launch.py'))
    )

    cone_detection = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('miniav_launch'),
                'launch/cone_detection.launch.py')),
    )

    return LaunchDescription([
        control,
        path_planning,
        cone_detection
    ])
