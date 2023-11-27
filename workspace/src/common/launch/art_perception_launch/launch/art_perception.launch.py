# ros imports
from launch import LaunchDescription

# internal imports
from launch_utils import IncludeLaunchDescriptionWithCondition


def generate_launch_description():
    ld = LaunchDescription()

    # ---------------
    # Launch Includes
    # ---------------

    IncludeLaunchDescriptionWithCondition(ld, "art_perception_launch", "cone_detector")
    IncludeLaunchDescriptionWithCondition(ld, "art_perception_launch", "lidar_scan_2d")
    return ld
