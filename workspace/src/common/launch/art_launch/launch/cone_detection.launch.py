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
    AddLaunchArgument("input/vehicle_state", "/vehicle/state")
    AddLaunchArgument("output/objects", "/perception/objects")

    AddLaunchArgument("use_sim_time", "False")
    AddLaunchArgument("model", "data/model_refined")
    AddLaunchArgument("camera_calibration_file", "data/calibration.json")
    AddLaunchArgument("vis", "False")

    # -----
    # Nodes
    # -----

    node = Node(
        package='cone_detector',
        namespace='',
        executable='object_recognition',
        name='object_recognition',
        remappings=[
            ("~/input/image", LaunchConfiguration("input/image")),
            ("~/input/vehicle_state", LaunchConfiguration("input/vehicle_state")),
            ("~/output/objects", LaunchConfiguration("output/objects"))
        ],
        parameters=[
             {"use_sim_time": LaunchConfiguration("use_sim_time")},
             {"model":LaunchConfiguration("model")},
             {"camera_calibration_file":LaunchConfiguration("camera_calibration_file",)},
             {"vis": LaunchConfiguration("vis")}
        ]
    )
    launch_description.add_action(node)

    return launch_description
