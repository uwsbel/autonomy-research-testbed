# Internal imports
from launch_utils import AddLaunchArgument, GetLaunchArgument

# ROS related imports
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    ld = LaunchDescription()

    # -----------------
    # Launch Parameters
    # -----------------

    use_flir = AddLaunchArgument("use_flir", True)

    # ---------------
    # Launch Includes
    # ---------------

    flir_camera_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('art_sensing_launch'),
                'launch',
                'flir_camera_driver.launch.py'
            ])
        ]),
        condition=IfCondition(use_flir),
    )
    ld.add_action(flir_camera_driver)

    usb_cam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('art_sensing_launch'),
                'launch',
                'usb_cam.launch.py'
            ])
        ]),
        condition=UnlessCondition(use_flir),
    )
    ld.add_action(usb_cam)

    return ld
