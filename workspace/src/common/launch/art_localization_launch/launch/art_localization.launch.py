# ros imports
from launch import LaunchDescription

# internal imports
from launch_utils import IncludeLaunchDescriptionWithCondition


def generate_launch_description():
    ld = LaunchDescription()

    # ---------------
    # Launch Includes
    # ---------------

    # IncludeLaunchDescriptionWithCondition(ld, "art_localization_launch", "ekf_estimation")
    IncludeLaunchDescriptionWithCondition(
        ld, "art_localization_launch", "particle_filter_estimation"
    )
    # IncludeLaunchDescriptionWithCondition(ld, "art_localization_launch", "ground_truth")
    return ld
