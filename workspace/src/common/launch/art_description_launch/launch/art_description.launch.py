# ros imports
from launch import LaunchDescription

# internal imports
from launch_utils import IncludeLaunchDescriptionWithCondition
from launch_utils import AddLaunchArgument, GetLaunchArgument, GetPackageSharePath
from launch.substitutions import LaunchConfiguration, PythonExpression, PathJoinSubstitution
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ld = LaunchDescription()
    robot_ns = LaunchConfiguration('robot_ns')
    veh_config = LaunchConfiguration('veh_config_file')


    urdf_file = os.path.join(
        get_package_share_directory('description'),
        'urdf',
        'robot.urdf'
    )

    package_share_directory = get_package_share_directory('art_sensing_launch')
    veh_params = PathJoinSubstitution([package_share_directory, 'config', veh_config])

    # ---------------
    # Launch Includes
    # ---------------

    gps = PythonExpression(['"', robot_ns, "/gps", '"'])
    imu = PythonExpression(['"', robot_ns, "/imu", '"'])
    base_link = PythonExpression(['"', robot_ns, "/base_link", '"'])
    odom = PythonExpression(['"', robot_ns, "/odom", '"'])

    # Transforms
    gps_transform = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="gps_transform",
        arguments=["0", "0", "0", "0", "0", "0", base_link, gps],
        parameters=[veh_params]
    )
    ld.add_action(gps_transform)
    imu_transform = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="imu_transform",
        arguments=["0", "0", "0", "0", "0.", "0.", base_link, imu],
        parameters=[veh_params]

    )
    ld.add_action(imu_transform)
    odom_transform = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="odom_tf",
        arguments=["0", "0", "0", "0", "0", "0", odom, base_link],
        parameters=[veh_params]
    )

    ld.add_action(odom_transform)
    
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[veh_params, 
                    { 'robot_description': open(urdf_file).read(),
                    'frame_prefix': PythonExpression(['"', '/', robot_ns, "/", '"']),
                    }]

    )
    ld.add_action(rsp)

    jsp = Node(
        package='description',
        executable='joint_state_publisher.py',
        name='joint_state_publisher',
        output='screen',
        parameters=[veh_params]
    )
    ld.add_action(jsp)
    
    return ld
