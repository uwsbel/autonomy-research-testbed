import os
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_directory


def generate_launch_description():
    launch_description = LaunchDescription()

    # Play a bag file
    play_bag = ExecuteProcess(
        # cmd = (['ros2', 'bag', 'play', 'V2_01_easy', '--topics', '/cam0/image_raw', '/imu0']),
        # cmd = (['ros2', 'bag', 'play', 'camera_imu', '--remap', '/sensing/front_facing_camera/raw:=/cam0/image_raw', '--rate', '5', '-l']),
        cmd = (['ros2', 'bag', 'play', 'camera_imu', '--rate', '2']),
        output = 'screen'
    )
    launch_description.add_action(play_bag)

    # IMU Publisher
    imu_publisher = Node(
        package='imu_publisher',
        namespace='',
        executable='imu_publisher_node',
        name='imu_publisher_node',
        remappings=[],
        parameters=[]
    )
    launch_description.add_action(imu_publisher)

    # Cam Publisher
    cam_publisher = Node(
        package='cam_publisher',
        namespace='',
        executable='cam_publisher_node',
        name='cam_publisher_node',
        remappings=[],
        parameters=[]
    )
    launch_description.add_action(cam_publisher)

    # Open VINS
    config_path = '/home/art-raj/art-raj/workspace/chrono_config/estimator_config.yaml'
    open_vins = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ov_msckf'),
                'launch/subscribe.launch.py')),
        launch_arguments={'config_path': config_path}.items()
    )
    launch_description.add_action(open_vins)

    return launch_description
