from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node


def generate_launch_description():

    DeclareLaunchArgument("control_mode", default_value=TextSubstitution(text="PID"))
    DeclareLaunchArgument("control_file", default_value=TextSubstitution(text="data/smallest_radius_right.csv"))
    DeclareLaunchArgument("steering_gain", default_value=TextSubstitution(text="1.6"))
    DeclareLaunchArgument("throttle_gain", default_value=TextSubstitution(text="0.08"))

    node = Node(
            package='control',
            namespace='',
            executable='pid',
            name='pid',
            parameters=[
                 {"control_mode": LaunchConfiguration("control_mode",default="PID")},
                 {"control_file": LaunchConfiguration("control_file",default="data/smallest_radius_right.csv")},
                 {"steering_gain": LaunchConfiguration("steering_gain",default=1.6)},
                 {"throttle_gain": LaunchConfiguration("throttle_gain",default=0.08)},
            ]
        )
    return  LaunchDescription([
        node
    ])