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
        ld, "centerline_objects_path_planner_launch", "centerline_objects_path_planner"
    )
    IncludeLaunchDescriptionWithCondition(
        ld, "waypoints_path_planner_launch", "waypoints_path_planner"
    )
    return ld
