from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node


def generate_launch_description():
    launch_description = LaunchDescription()

    # ----------------
    # Launch Arguments
    # ----------------

    def AddLaunchArgument(arg, default):
        launch_description.add_action(
            DeclareLaunchArgument(
                arg,
                default_value=TextSubstitution(text=default)
            )
        )

    AddLaunchArgument("input/path", "/path_planning/path")
    AddLaunchArgument("input/vehicle_state", "/vehicle/state")
    AddLaunchArgument("output/vehicle_inputs", "/control/vehicle_inputs")

    AddLaunchArgument("control_mode", "PID")
    AddLaunchArgument("control_file", "data/smallest_radius_right.csv")
    AddLaunchArgument("steering_gain", "1.6")
    AddLaunchArgument("throttle_gain", "0.08")
    AddLaunchArgument("use_sim_time", "False")

    # -----
    # Nodes
    # -----

    node = Node(
            package='control',
            namespace='',
            executable='pid',
            name='pid',
            remappings=[
                ("~/input/path", LaunchConfiguration("input/path")),
                ("~/input/vehicle_state", LaunchConfiguration("input/vehicle_state")),
                ("~/output/vehicle_inputs", LaunchConfiguration("output/vehicle_inputs"))
            ],
            parameters=[
                {"use_sim_time": LaunchConfiguration("use_sim_time")},
                {"control_mode": LaunchConfiguration("control_mode",default="PID")},
                {"control_file": LaunchConfiguration("control_file",default="data/smallest_radius_right.csv")},
                {"steering_gain": LaunchConfiguration("steering_gain",default=1.6)},
                {"throttle_gain": LaunchConfiguration("throttle_gain",default=0.08)},
            ]
        )
    launch_description.add_action(node)

    return launch_description
