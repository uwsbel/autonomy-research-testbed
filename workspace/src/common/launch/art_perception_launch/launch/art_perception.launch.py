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

    # IncludeLaunchDescriptionWithCondition(ld, "art_perception_launch", "cone_detector")
    IncludeLaunchDescriptionWithCondition(ld, "art_perception_launch", "cone_detector")

    return ld
