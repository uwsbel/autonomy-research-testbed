import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import TextSubstitution

from ament_index_python import get_package_share_directory


def generate_launch_description():

    stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('miniav_launch'),
                'launch/miniav_stack.launch.py')),
        launch_arguments={'use_sim_time': 'True'}.items()
    )

    #TODO:
    # sim_bridge = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(
    #             get_package_share_directory('miniav_launch'),
    #             'launch/sim_bridge.launch.py'))
    # )

    return LaunchDescription([
        stack
        #TODO: sim_bridge
    ])
