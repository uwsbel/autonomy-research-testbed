import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python import get_package_share_directory


def generate_launch_description():

    launch_description = LaunchDescription()

    # ------------
    # Launch Files
    # ------------

    stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('miniav_launch'),
                'launch/miniav_stack.launch.py'))
    )
    launch_description.add_action(stack)

    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('miniav_launch'),
                'launch/usb_cam.launch.py'))
    )
    launch_description.add_action(camera)

    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('miniav_launch'),
                'launch/arduino_driver.launch.py'))
    )
    launch_description.add_action(camera)

    return launch_description