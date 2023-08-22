# ros imports
from launch import LaunchDescription

# internal imports
from launch_utils import IncludeLaunchDescriptionWithCondition


def generate_launch_description():

    ld = LaunchDescription()

    # ---------------
    # Launch Includes
    # ---------------

    IncludeLaunchDescriptionWithCondition(ld, "art_sensing_launch", "usb_cam")
    IncludeLaunchDescriptionWithCondition(ld, "art_sensing_launch", "xsens")

    return ld 
