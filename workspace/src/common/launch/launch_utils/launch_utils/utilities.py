from launch import LaunchDescription


def AddLaunchArgument(
        ld: LaunchDescription, arg: "Any", default: "Any", **kwargs) -> "LaunchConfiguration":
    """Helper method to add a launch argument to a LaunchDescription"""
    from launch.actions import DeclareLaunchArgument

    ld.add_action(DeclareLaunchArgument(arg, default_value=default, **kwargs))
    return GetLaunchArgument(arg)


def SetLaunchArgument(ld: LaunchDescription, arg: "Any", value: "Any",  **kwargs) -> None:
    """Helper method to set a launch argument of a LaunchDescription"""
    from launch.actions import SetLaunchConfiguration

    ld.add_action(SetLaunchConfiguration(name=arg, value=value, **kwargs))


def GetLaunchArgument(name: str, **kwargs) -> "LaunchConfiguration":
    """Helper method to get a launch configuration value"""

    from launch.substitutions import LaunchConfiguration

    return LaunchConfiguration(name, **kwargs)


def GetPackageSharePath(package: str, *args, **kwargs) -> "pathlib.Path":
    """Wrapper around ament_index_python.packages.get_package_share_directory that 
    returns a pathlib.Path.
    
    Args:
        package (str): the package passed to ``get_package_share_directory``.
    """
    from pathlib import Path
    from ament_index_python.packages import get_package_share_directory

    return Path(get_package_share_directory(package), *args, **kwargs)


def GetPackageSourceDirectory(package: str, base: str = None) -> 'pathlib.Path':
    """Helper method to get the package **source** directory.

    Usually it's recommended to use ament_index_python.packages.get_package_share_directory,
    but this returns the _install_ directory where everything is put after colcon build. In some
    cases, we want to put code/generated data/etc. in the _source_ code. This package helps with this.

    It takes a package name (str) and a base name (str). The base name is the location to begin
    an downward search of the file tree for the package name. If base name is None (the default),
    the environment variable 'ROS_WORKSPACE_SRC' will be used. If 'ROS_WORKSPACE_SRC' is not found,
    the current working directory is used.

    It is considered found if the package name is in the current searched folder absolute path
    and a package.xml file is found in the same folder.

    Will return the path as a pathlib.Path object or None if not found.

    WARNING: Not fast if there are a lot of folders on the system.
    """
    import os
    from pathlib import Path

    if base is None:
        base = os.environ.get("ROS_WORKSPACE_SRC", os.getcwd())

    for root, dirs, files in os.walk(base):
        if package == os.path.basename(root) and 'package.xml' in files:
            return Path(root)

    return None


def AddComposableNode(
    ld: LaunchDescription,
    *,
    plugin: str,
    executable: str,
    composable_conditions: "List[List[SomeSubstitutionType]" = [],
    node_conditions: "List[List[SomeSubstitutionType]]" = [],
    container: str = "container",
    composable_node_kwargs: dict = {},
    load_composable_nodes_kwargs: dict = {},
    node_kwargs: dict = {},
    add_as_regular_node: bool = True,
    **kwargs
) -> None:
    """
    Conditionally add a node dependent on whether the LaunchConfiguration ``container`` is
    not "". If not "", it will load the node into the composable container using ``LoadComposableNodes``.
    In this case, "use_intra_process_comms" will be set to true. If ``container`` is "",
    it will simply load the node using ``Node`` without intraprocess comms.

    If ``add_as_regular_node`` is set to False (defaults to True), it will not create a ``Node`` if ``container`` is unset.
    """

    from launch_utils.conditions import MultipleIfConditions
    from launch_utils.substitutions import QuoteWrappedPythonExpression

    from launch_ros.actions import Node, LoadComposableNodes
    from launch_ros.descriptions import ComposableNode

    container = GetLaunchArgument(container)

    # Add a node the container only if the container launch argument is set
    composable_condition = QuoteWrappedPythonExpression([container, " != ''"])
    composable_nodes = [
        ComposableNode(
            plugin=plugin,
            extra_arguments=[{"use_intra_process_comms": True}],
            **composable_node_kwargs,
            **kwargs,
        )
    ]
    load_composable_nodes = LoadComposableNodes(
        condition=MultipleIfConditions(
            [composable_condition, *composable_conditions]),
        composable_node_descriptions=composable_nodes,
        target_container=container,
        **load_composable_nodes_kwargs,
    )
    ld.add_action(load_composable_nodes)

    # Add a regular nod if container is unset AND add_as_regular_node is true
    node_condition = QuoteWrappedPythonExpression([container, " == ''"])
    node = Node(
        condition=MultipleIfConditions([node_condition, *node_conditions]),
        executable=executable,
        **node_kwargs,
        **kwargs,
    )
    if add_as_regular_node:
        ld.add_action(node)


def IncludeLaunchDescriptionWithCondition(ld: LaunchDescription, share: str, package: str, *, default: str = "False", **kwargs):
    """
    Conditionally include launch file.

    This method is a simple wrapper of ``launch.actions.IncludeLaunchDescription`` that also adds a launch argument
    that is conditionally checked whether it is set to false a runtime.

    By default, the launch arguments name is ``disable_<package>``.

    Args:
        ld (LaunchDescription)
        share (str): The name of the package that has the launch files (assumes to be in <share>/launch directory).
        package (str): The package to launch. The launch file is assumed to be <package>.launch.py.
        default (str): The default value. Defaults to "False".
        **kwargs
    """

    from launch.actions import IncludeLaunchDescription
    from launch.conditions import UnlessCondition
    from launch.launch_description_sources import PythonLaunchDescriptionSource
    from launch.substitutions import PathJoinSubstitution
    from launch_ros.substitutions import FindPackageShare

    launch_argument = f"disable_{package}"
    disable_package = AddLaunchArgument(
        ld, launch_argument, default, description=f"Disable the '{package}.launch.py' file from being included.")

    package = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare(share),
                "launch",
                f"{package}.launch.py"
            ])
        ]),
        condition=UnlessCondition(disable_package),
    )
    ld.add_action(package)