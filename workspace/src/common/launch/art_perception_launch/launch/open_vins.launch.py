import os
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_directory


def generate_launch_description():
    launch_description = LaunchDescription()

    # launch arguments
    config_path = '/home/art/art/workspace/chrono_config/estimator_config.yaml'
    config_path_arg = DeclareLaunchArgument(
            'config_path',
            default_value = config_path,
    )
    launch_description.add_action(config_path_arg)

    # Play a bag file
    play_bag = ExecuteProcess(
        cmd = (['ros2', 'bag', 'play', 'camera_imu']),
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

    # Open VINS
    open_vins = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ov_msckf'),
                'launch/subscribe.launch.py')),
        launch_arguments={'config_path': config_path}.items()
    )
    launch_description.add_action(open_vins)

    return launch_description
