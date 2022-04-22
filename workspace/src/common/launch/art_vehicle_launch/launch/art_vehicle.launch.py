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

    use_arduino = AddLaunchArgument(ld, "use_arduino", "True")

    # ---------------
    # Launch Includes
    # ---------------

    arduino_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('art_vehicle_launch'),
                'launch',
                'arduino_driver.launch.py'
            ])
        ]),
        condition=IfCondition(use_arduino),
    )
    ld.add_action(arduino_driver)

    return ld
