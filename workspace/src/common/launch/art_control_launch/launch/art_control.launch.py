# ros imports
from launch import LaunchDescription

# internal imports
from launch_utils import IncludeLaunchDescriptionWithCondition
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description():
    ld = LaunchDescription()
    task = LaunchConfiguration('task')

    # ---------------
    # Launch Includes
    # ---------------

    IncludeLaunchDescriptionWithCondition(
        ld, "art_control_launch", "convoy_leader"
    )

    #IncludeLaunchDescriptionWithCondition(
    #    ld, "art_control_launch", "pid_lateral_follower"
    #)

    #IncludeLaunchDescriptionWithCondition(
    #    ld, "art_control_launch", "pid_lateral_controller"
    #)

    return ld
