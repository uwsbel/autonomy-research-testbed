import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python import get_package_share_directory


def generate_launch_description():

    DeclareLaunchArgument(
        "use_sim_time", default_value=False
    )

    stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('miniav_launch'),
                'launch/miniav_stack.launch.py')),
        launch_arguments={
            'use_sim_time': 'False'
        }.items()
    )

    #TODO:
    camera_driver = IncludeLaunchDescription(
         PythonLaunchDescriptionSource(
             os.path.join(
                 get_package_share_directory('miniav_launch'),
                 'launch/usb_cam.launch.py'))
    )

    #TODO:
    # arduino_driver = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(
    #             get_package_share_directory('miniav_launch'),
    #             'launch/arduino_driver.launch.py'))
    # )

    return LaunchDescription([
        stack
        #TODO: camera_driver
        #TODO: arduino_driver
    ])
