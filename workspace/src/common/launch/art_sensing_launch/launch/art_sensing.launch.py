# ros imports
from launch import LaunchDescription

# internal imports
from launch_utils import IncludeLaunchDescriptionWithCondition
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description():
    ld = LaunchDescription()
    robot_ns = LaunchConfiguration('artcar_1')

    # ---------------
    # Launch Includes
    # ---------------

    #IncludeLaunchDescriptionWithCondition(ld, "art_sensing_launch", "usb_cam")

    #IncludeLaunchDescriptionWithCondition(ld, "art_sensing_launch", "xsens")

    IncludeLaunchDescriptionWithCondition(ld, "art_sensing_launch", "gps")
    IncludeLaunchDescriptionWithCondition(ld, "art_sensing_launch", "imu")


    return ld
