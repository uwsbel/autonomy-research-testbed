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

    AddLaunchArgument("use_sim_time", "False")

    # -----
    # Nodes
    # -----

    node = Node(
            package='arduino_driver',
            namespace='',
            executable='motor_driver',
            name='motor_driver',
            remappings=[
                ("~/output/vehicle_inputs", LaunchConfiguration("output/vehicle_inputs"))
            ],
            parameters=[
                {"use_sim_time": LaunchConfiguration("use_sim_time")}
            ]
        )
    launch_description.add_action(node)

    return launch_description
