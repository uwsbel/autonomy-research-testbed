import os
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_directory


def generate_launch_description():
    launch_description = LaunchDescription()

    # Declare launch arguments
    launch_description.add_action(DeclareLaunchArgument('bag_file', default_value='rosbags/camera_imu'))
    launch_description.add_action(DeclareLaunchArgument('sim_topics', default_value='false'))
    launch_description.add_action(DeclareLaunchArgument('flip_image', default_value='false'))

    # Play a bag file
    play_bag = ExecuteProcess(
        cmd = (['ros2', 'bag', 'play', LaunchConfiguration('bag_file')]),
        output = 'screen'
    )
    launch_description.add_action(play_bag)

    # IMU Publisher
    imu_publisher = Node(
        package='imu_publisher',
        namespace='',
        executable='imu_publisher_node',
        name='imu_publisher_node',
        parameters=[{'sim_topics': LaunchConfiguration('sim_topics')}]
    )
    launch_description.add_action(imu_publisher)

    # Cam Publisher
    cam_publisher = Node(
        package='cam_publisher',
        namespace='',
        executable='cam_publisher_node',
        name='cam_publisher_node',
        parameters=[{'sim_topics': LaunchConfiguration('sim_topics')},
                    {'flip_image': LaunchConfiguration('flip_image')}]
    )
    launch_description.add_action(cam_publisher)

    # Open VINS
    # config_path = '/home/art-raj/art-raj/workspace/chrono_config/estimator_config.yaml'
    open_vins = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ov_msckf'),
                'launch/subscribe.launch.py')),
        launch_arguments={'config': 'art'}.items()
    )
    launch_description.add_action(open_vins)

    return launch_description
