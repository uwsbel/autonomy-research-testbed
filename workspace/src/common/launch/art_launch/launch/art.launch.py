# Internal imports
from launch_utils import AddLaunchArgument, GetLaunchArgument
from launch_utils.launch.substitutions import QuoteWrappedPythonExpression

# ROS imports
from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch.actions import IncludeLaunchDescription, SetLaunchConfiguration, LogInfo
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import ComposableNodeContainer

from ament_index_python import get_package_share_directory

CONTAINER_NAME = f"art_container"

def generate_launch_description():
    ld = LaunchDescription()

    # ----------------
    # Launch Arguments
    # ----------------

    AddLaunchArgument(ld, "scenario", None, description="'real' or 'sim'")

    AddLaunchArgument(ld, "use_composability", "True")
    AddLaunchArgument(ld, "container", "")

    # -------------
    # Composability
    # -------------
    # If composability is desired, all included launch descriptions should attach to this container and use
    # intraprocess communication

    use_composability = IfCondition(GetLaunchArgument("use_composability"))

    # If a container name is not provided,
    # set the name of the container launched above for image_proc nodes
    set_container_name = SetLaunchConfiguration(
        condition=use_composability,
        name='container',
        value=CONTAINER_NAME
    )
    ld.add_action(set_container_name)

    container = ComposableNodeContainer(
        name=CONTAINER_NAME,
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[],
        condition=use_composability,
        output="screen",
    )
    ld.add_action(container)

    # ---------------
    # Launch Includes
    # ---------------

    use_sim = QuoteWrappedPythonExpression([GetLaunchArgument("scenario"), " == ", "'sim'"])
    art_stack_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('art_launch'),
                'launch',
                'art_stack.launch.py'
            ])
        ]),
        launch_arguments=[
            ('use_sim_time', use_sim),
            ('use_sim_msg', use_sim),
        ],
    )
    ld.add_action(art_stack_launch)

    art_real_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('art_launch'),
                'launch',
                'art_real.launch.py'
            ])
        ]),
        condition=LaunchConfigurationEquals("scenario", "real"),
    )
    ld.add_action(art_real_launch)

    art_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('art_launch'),
                'launch',
                'art_sim.launch.py'
            ])
        ]),
        condition=LaunchConfigurationEquals("scenario", "sim"),
    )
    ld.add_action(art_sim_launch)

    return ld
