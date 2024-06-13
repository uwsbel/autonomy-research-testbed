# ros imports
from launch import LaunchDescription

# internal imports
from launch_utils import IncludeLaunchDescriptionWithCondition


def generate_launch_description():
    ld = LaunchDescription()

    AddLaunchArgument(ld, "namespace", "artcar_1")

    # ---------------
    # Launch Includes
    # ---------------

    IncludeLaunchDescriptionWithCondition(
        ld, "art_control_launch", "pid_lateral_controller",
        namespace=GetLaunchArgument("namespace")
    )

    return ld
