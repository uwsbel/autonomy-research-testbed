import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, EmitEvent
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import TextSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch.events import Shutdown

from ament_index_python import get_package_share_directory


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

    AddLaunchArgument("input/vehicle_inputs", "/control/vehicle_inputs")
    AddLaunchArgument("output/time", "/clock")
    AddLaunchArgument("output/vehicle", "/vehicle/state")
    AddLaunchArgument("output/camera", "/sensing/front_facing_camera/raw")
    AddLaunchArgument("ip", "")
    AddLaunchArgument("hostname", "")
    AddLaunchArgument("use_sim_time", "True")

    # ------------
    # Launch Files
    # ------------

    stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('miniav_launch'),
                'launch/miniav_stack.launch.py')),
        launch_arguments=[
            ('use_sim_time', 'True'),
            ('use_sim_msg', 'True'),
        ]
    )
    launch_description.add_action(stack)

    # -----
    # Nodes
    # -----

    chrono_ros_bridge = Node(
        package='chrono_ros_bridge',
        namespace='',
        executable='chrono_ros_bridge',
        name='chrono_ros_bridge',
        remappings=[
            ("~/input/driver_inputs", LaunchConfiguration("input/vehicle_inputs")),
            ("~/output/time", LaunchConfiguration("output/time")),
            ("~/output/vehicle", LaunchConfiguration("output/vehicle")),
            ("~/output/camera/front_facing_camera", LaunchConfiguration("output/camera")),
        ],
        parameters=[
             {"use_sim_time": LaunchConfiguration("use_sim_time")},
             {"ip": LaunchConfiguration("ip")},
             {"hostname": LaunchConfiguration("hostname")},
        ],
        on_exit=EmitEvent(event=Shutdown()),
		
    )
    launch_description.add_action(chrono_ros_bridge)

    return launch_description

