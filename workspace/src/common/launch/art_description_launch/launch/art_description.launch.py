# ros imports
from launch import LaunchDescription

# internal imports
from launch_utils import IncludeLaunchDescriptionWithCondition
from launch_utils import AddLaunchArgument, GetLaunchArgument, GetPackageSharePath
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()
    robot_ns = LaunchConfiguration('robot_ns')

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
    )
    ld.add_action(gps_transform)
    imu_transform = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="imu_transform",
        arguments=["0", "0", "0", "3.1416", "-1.5708", "0", base_link, imu],
    )
    ld.add_action(imu_transform)
    odom_transform = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="odom_tf",
        arguments=["0", "0", "0", "0", "0", "0", odom, base_link],
    )
    ld.add_action(odom_transform)


    IncludeLaunchDescriptionWithCondition(
        ld, "description", "robot"
    )
    
    return ld
