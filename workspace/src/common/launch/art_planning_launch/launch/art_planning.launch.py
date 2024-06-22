# ros imports
from launch import LaunchDescription

# internal imports
from launch_utils import IncludeLaunchDescriptionWithCondition
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description():
    ld = LaunchDescription()


    # ---------------
    # Launch Includes
    # ---------------
    IncludeLaunchDescriptionWithCondition(
        ld, "art_planning_launch", "centerline_objects_path_planner"
    )
    IncludeLaunchDescriptionWithCondition(
        ld, "art_planning_launch", "waypoints_path_planner"
    )
    return ld
