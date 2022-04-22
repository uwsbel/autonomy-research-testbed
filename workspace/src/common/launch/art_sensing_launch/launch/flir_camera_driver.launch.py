# Internal imports
from launch_utils import AddLaunchArgument, GetLaunchArgument, AddComposableNode, SetLaunchArgument
from launch_utils.launch.conditions import MultipleIfConditions

# ROS related imports
from launch import LaunchDescription
from launch.substitutions import PythonExpression

from ament_index_python.packages import get_package_share_directory

# External imports
from pathlib import Path

# Default camera parameters
default_camera_parameters = {
    "debug": False,
    "compute_brightness": False,
    "dump_node_map": False,
    # set parameters defined in blackfly_s.cfg
    "gain_auto": "Continuous",
    "exposure_auto": "Continuous",
    "always_publish": True,
    "frame_rate_auto": "Off",
    "frame_rate": 30.0,
    "frame_rate_enable": True,
    "trigger_mode": "Off",
    "chunk_mode_active": True,
    "chunk_selector_frame_id": "FrameID",
    "chunk_enable_frame_id": True,
    "chunk_selector_exposure_time": "ExposureTime",
    "chunk_enable_exposure_time": True,
    "chunk_selector_gain": "Gain",
    "chunk_enable_gain": True,
    "chunk_selector_timestamp": "Timestamp",
    "chunk_enable_timestamp": True,
}


def generate_launch_description():
    ld = LaunchDescription()

    # ----------------
    # Launch Arguments
    # ----------------

    flir_pkg_directory = Path(
        get_package_share_directory("flir_spinnaker_ros2"))
    flir_config_directory = flir_pkg_directory / "config"

    art_sensing_launch_directory = Path(
        get_package_share_directory("art_sensing_launch"))
    art_sensing_config_directory = art_sensing_launch_directory / "config"

    with open(art_sensing_config_directory / "art_sensing.config.yml", "r") as f:
        import yaml

        data = yaml.safe_load(f)

        flir_spinnaker_ros2_parameters = data['flir_spinnaker_ros2']['launch__parameters']
        art_flir_camera_synchronizer_parameters = data['flir_camera_synchronizer']['ros__parameters']

        if not art_flir_camera_synchronizer_parameters:
            art_flir_camera_synchronizer_parameters = {}

    # If set, use composability with intraprocess communication
    AddLaunchArgument(ld, "container", "")

    cameras = AddLaunchArgument(ld, "cameras", "['fwc']")
    synchronize_cameras = AddLaunchArgument(ld, "synchronize_cameras", "True")

    synchronize = [synchronize_cameras,
                   PythonExpression(["len(", cameras, ") > 1"])]
    synchronize_condition = MultipleIfConditions(synchronize)

    # -----
    # Nodes
    # -----

    topics = []
    for i, (camera_name, camera) in enumerate(flir_spinnaker_ros2_parameters["cameras"].items()):
        # Add a launch argument for the topic name
        camera_topic = camera_name + "_topic"
        AddLaunchArgument(ld, camera_topic, camera["topic"])

        # If "synchronize_cameras" is true, update the topic name to end with "_unsynced"
        SetLaunchArgument(
            ld, camera_topic, camera["topic"] + "_unsynced", condition=synchronize_condition)

        # Basically checks that the current iterated camera is in the requested cameras list
        requested_cameras = PythonExpression([f"'{camera_name}' in ", cameras])

        AddComposableNode(
            ld,
            plugin="flir_spinnaker_ros2::CameraDriver",
            executable="camera_driver_node",
            composable_conditions=[requested_cameras],
            node_conditions=[requested_cameras],
            package="flir_spinnaker_ros2",
            name=camera_name,
            remappings=[
                ("~/control", "/exposure_control/control"),
                (f"/{camera_name}/image_raw", GetLaunchArgument(camera_topic))
            ],
            parameters=[
                default_camera_parameters,
                {
                    "parameter_file": str(flir_config_directory / f"{camera['type']}.cfg"),
                    "serial_number": f"{camera['serial']}",
                }
            ],
            node_kwargs={"output": "screen"},
        )

        # Keep track of the created topics for us to synchronize using flir_camera_synchronizer
        topics.append((camera["topic"], camera["topic"] + "_unsynced"))

    # Remap all synchronized nodes to their original topic names
    remappings = []
    for i, (synced, unsynced) in enumerate(topics):
        remappings.append((f"~/input/topic_{i}", unsynced))
        remappings.append((f"~/output/topic_{i}", synced))

    # Add the synchronizer node, but only if desired
    AddComposableNode(
        ld,
        plugin="flir_camera_synchronizer::FlirCameraSynchronizer",
        executable="flir_camera_synchronizer_node",
        package="flir_camera_synchronizer",
        name="flir_camera_synchronizer",
        composable_conditions=synchronize,
        node_conditions=synchronize,
        parameters=[
            art_flir_camera_synchronizer_parameters,
            {"num_topics": len(topics)}
        ],
        remappings=remappings,
        node_kwargs={"output": "screen"},
    )

    return ld
