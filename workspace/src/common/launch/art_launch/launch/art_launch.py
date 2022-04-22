# Internal imports
from launch_utils import AddLaunchArgument, GetLaunchArgument
from launch_utils.substitutions import QuoteExpandedPythonExpression

# ROS imports
from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch.actions import IncludeLaunchDescription, SetLaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource

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

    use_sim = QuoteExpandedPythonExpression([GetLaunchArgument("scenario"), " == ", "sim"])
    wauto_stack_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('wauto_launch'),
                'launch',
                'wauto_stack.launch.py'
            ])
        ]),
        launch_arguments=[
            ('use_sim_time', use_sim),
            ('use_sim_msg', use_sim),
        ],
    )
    ld.add_action(wauto_stack_launch)

    wauto_real_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('wauto_launch'),
                'launch',
                'wauto_real.launch.py'
            ])
        ]),
        condition=LaunchConfigurationEquals(GetLaunchArgument("scenario"), "real"),
    )
    ld.add_action(wauto_real_launch)

    wauto_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('wauto_launch'),
                'launch',
                'wauto_sim.launch.py'
            ])
        ]),
        condition=LaunchConfigurationEquals(GetLaunchArgument("scenario"), "sim"),
    )
    ld.add_action(wauto_sim_launch)

    return ld
