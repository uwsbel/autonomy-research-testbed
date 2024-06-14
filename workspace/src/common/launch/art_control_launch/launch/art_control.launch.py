# ros imports
from launch import LaunchDescription

# internal imports
from launch_utils import IncludeLaunchDescriptionWithCondition


def generate_launch_description():
    ld = LaunchDescription()

    # ---------------
    # Launch Includes
    # ---------------

    IncludeLaunchDescriptionWithCondition(
        ld, "art_control_launch", "pid_lateral_controller"
    )

    return ld
