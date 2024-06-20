# ros imports
from launch import LaunchDescription
from launch_ros.actions import Node

# internal imports
from launch_utils import AddLaunchArgument, GetLaunchArgument, GetPackageSharePath
from enum import Enum
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction


def generate_launch_description():
    ld = LaunchDescription()

    # ---------------
    # Launch Includes
    # ---------------
    robot_ns = LaunchConfiguration('robot_ns')


    AddLaunchArgument(
        ld, "input/gyroscope/data",  PythonExpression(['"', '/', robot_ns, "/output/gyroscope/data", '"'])      
    )
    AddLaunchArgument(
        ld, "input/accelerometer/data",  PythonExpression(['"', '/', robot_ns, "/output/accelerometer/data", '"']))    
    AddLaunchArgument(
        ld, "output/imu", PythonExpression(['"', '/', robot_ns, "/imu/data", '"']))  
    AddLaunchArgument(
        ld, "output/gps/fix", PythonExpression(['"', '/', robot_ns, "/output/gps/fix", '"']))  

    AddLaunchArgument(
        ld, "output/gps/data", PythonExpression(['"', '/', robot_ns, "/output/gps/data", '"']))  

    AddLaunchArgument(
        ld, "input/magnetometer/data", PythonExpression(['"', '/', robot_ns, "/output/magnetometer/data", '"']))  
    AddLaunchArgument(
        ld, "output/magnetometer/data", PythonExpression(['"', '/', robot_ns, "/output/magnetometer/data/filtered", '"']))  


    node = Node(
        package="imu_filter",
        executable="imu_filter",
        name="imu_filter",
        namespace=robot_ns,
        remappings=[
            (
                "/input/gyroscope/data",
                GetLaunchArgument("input/gyroscope/data"),
            ),
            (
                "/input/accelerometer/data",
                GetLaunchArgument("input/accelerometer/data"),
            ),
            (
                "/output/imu",
                GetLaunchArgument("output/imu"),
            ), 
            (
                "/output/gps/data",
                GetLaunchArgument("output/gps/data"),
            ), 
            (
                "/output/gps/fix",
                GetLaunchArgument("output/gps/fix"),
            ), 
            (
                "/input/magnetometer/data",
                GetLaunchArgument("input/magnetometer/data"),
            ), 
            (
                "/output/magnetometer/data",
                GetLaunchArgument("output/magnetometer/data"),
            )
        ],
        parameters=[{'tf_prefix': robot_ns }],
    )
    ld.add_action(node)

    # name='imu_filter_madgwick'
    
    # imu_filter_madgwick_node = Node(
    #     package="imu_filter_madgwick",
    #     executable="imu_filter_madgwick_node",
    #     name=name,
    #     namespace=robot_ns,
    #     parameters=[
    #         {
    #             "use_mag": True,
    #             "publish_tf": False,
    #             "world_frame": "enu",
    #             "base_frame": PythonExpression(['"', robot_ns, "/base_link", '"']), #"artcar_1/base_link",
    #             "frequency": 50.0
    #         }
    #     ],

    #     remappings=[
    #         ("/imu/data_raw", PythonExpression(['"', '/', robot_ns, "/imu/data_raw", '"'])),
    #         (PythonExpression(['"', '/', robot_ns, "/imu/mag", '"']), PythonExpression(['"', '/', robot_ns, "/output/magnetometer/data/filtered", '"'])),
    #         ("/imu/data", PythonExpression(['"', '/', robot_ns, "/imu/data", '"']))
    #     ]
    # )
    # ld.add_action(imu_filter_madgwick_node)


    return ld
