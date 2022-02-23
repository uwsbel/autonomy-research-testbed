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

    AddLaunchArgument("input/image", "/sensing/front_facing_camera/raw")
    AddLaunchArgument("input/objects", "/perception/objects")
    AddLaunchArgument("output/path", "/path_planning/path")

    AddLaunchArgument("use_sim_time", "False")
    AddLaunchArgument("vis", "False")
    AddLaunchArgument("lookahead", ".75")

    # -----
    # Nodes
    # -----

    node = Node(
            package='path_planning',
            namespace='',
            executable='path_planning',
            name='path_planning',
            remappings=[
                ("~/input/objects", LaunchConfiguration("input/objects")),
                ("~/input/vehicle_state", LaunchConfiguration("input/vehicle_state")),
                ("~/output/path", LaunchConfiguration("output/path"))
            ],
            parameters=[
                {"use_sim_time": LaunchConfiguration("use_sim_time")},
                {"vis": LaunchConfiguration("vis")},
                {"lookahead": LaunchConfiguration("lookahead")}
            ]
        )
    launch_description.add_action(node)

    return launch_description
