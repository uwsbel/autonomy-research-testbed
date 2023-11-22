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
        ld, "art_planning_launch", "centerline_path_planner"
    )

    return ld
