import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PythonExpression, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Declare the launch arguments
    leader_ns = LaunchConfiguration('leader_ns')
    robot_ns = LaunchConfiguration('robot_ns')
    veh_config = LaunchConfiguration('veh_config_file')

    # ---------------
    # Vehicle Config
    # ---------------
    package_share_directory = FindPackageShare('art_sensing_launch').find('art_sensing_launch')
    veh_config_file_path = PathJoinSubstitution([package_share_directory, 'config', veh_config])

    return LaunchDescription([
        DeclareLaunchArgument(
            'leader_ns',
            default_value='',
            description='Namespace for leader vehicle'
        ),
        DeclareLaunchArgument(
            'robot_ns',
            default_value='',
            description='Namespace for robot vehicle'
        ),
        DeclareLaunchArgument(
            'veh_config_file',
            default_value='default_veh_config.yaml',
            description='Vehicle configuration file'
        ),

        # Conditionally launch the leader vehicle nodes
        GroupAction(
            condition=IfCondition(PythonExpression([
                "'", leader_ns, "'",
                " != 'none'",
            ])),
            actions=[
                Node(
                    package='convoy_controller',
                    executable='follower',  
                    name='follower_control_node',
                    namespace=robot_ns,
                    remappings=[
                        ('/leader_vehicle/odometry/filtered', PythonExpression(['"', '/', leader_ns, "/odometry/filtered", '"'])),
                        ('/follower_vehicle/odometry/filtered', PythonExpression(['"', '/', robot_ns, "/odometry/filtered", '"'])),
                        ('/follower_vehicle/driver_inputs', PythonExpression(['"', '/', robot_ns, "/input/lateral_input", '"'])),
                        ('/leader_vehicle/vehicle_traj', PythonExpression(['"', '/', leader_ns, "/vehicle_traj", '"'])),
                        ('/follower_vehicle/vehicle_traj', PythonExpression(['"', '/', robot_ns, "/vehicle_traj", '"']))
                    ],
                    parameters=[veh_config_file_path],
                    output='screen'
                ),
                Node(
                    package='convoy_controller',  
                    executable='velocity',  
                    name='velocity_controller',
                    namespace=robot_ns,
                    remappings=[
                        ('/odometry/filtered', PythonExpression(['"', '/', robot_ns, "/odometry/filtered", '"'])),
                        ('/driver_inputs', PythonExpression(['"', '/', robot_ns, "/input/long_input", '"']))
                    ],
                    parameters=[veh_config_file_path],
                    output='screen'
                ),
                Node(
                    package='convoy_controller',  
                    executable='vehicle_traj',  
                    name='vehicle_trajectory_publisher',
                    namespace=robot_ns,
                    remappings=[
                        ('/odometry/filtered', PythonExpression(['"', '/', robot_ns, "/odometry/filtered", '"'])),
                        ('/vehicle_traj', PythonExpression(['"', '/', robot_ns, "/vehicle_traj", '"']))
                    ],
                    parameters=[veh_config_file_path],
                    output='screen'
                ),
                Node(
                    package='convoy_controller',  
                    executable='control_mux',  
                    name='combined_input_node',
                    namespace=robot_ns,
                    remappings=[
                        ('/lateral_input', PythonExpression(['"', '/', robot_ns, "/input/lateral_input", '"'])),
                        ('/long_input', PythonExpression(['"', '/', robot_ns, "/input/long_input", '"'])),
                        ('/input/driver_inputs', PythonExpression(['"', '/', robot_ns, "/input/driver_inputs", '"']))
                    ],
                    parameters=[veh_config_file_path],
                    output='screen'
                )
            ]
        ),

        # Conditionally launch the follower vehicle nodes
        GroupAction(
            condition=IfCondition(PythonExpression([
                "'", leader_ns, "'",
                " == 'none'",
            ])),
            actions=[
                Node(
                    package='convoy_controller',  
                    executable='velocity',  
                    name='velocity_controller',
                    namespace=robot_ns,
                    remappings=[
                        ('/odometry/filtered', PythonExpression(['"', '/', robot_ns, "/odometry/filtered", '"'])),
                        ('/driver_inputs', PythonExpression(['"', '/', robot_ns, "/input/long_input", '"']))
                    ],
                    parameters=[veh_config_file_path],
                    output='screen'
                ),
                Node(
                    package='convoy_controller',  
                    executable='vehicle_traj',  
                    name='vehicle_trajectory_publisher',
                    namespace=robot_ns,
                    remappings=[
                        ('/odometry/filtered', PythonExpression(['"', '/', robot_ns, "/odometry/filtered", '"'])),
                        ('/vehicle_traj', PythonExpression(['"', '/', robot_ns, "/vehicle_traj", '"']))
                    ],
                    parameters=[veh_config_file_path],
                    output='screen'
                ),
                Node(
                    package='convoy_controller',  
                    executable='control_mux',  
                    name='combined_input_node',
                    namespace=robot_ns,
                    remappings=[
                        ('/lateral_input', PythonExpression(['"', '/', robot_ns, "/input/lateral_input", '"'])),
                        ('/long_input', PythonExpression(['"', '/', robot_ns, "/input/long_input", '"'])),
                        ('/input/driver_inputs', PythonExpression(['"', '/', robot_ns, "/input/driver_inputs", '"']))
                    ],
                    parameters=[veh_config_file_path],
                    output='screen'
                )
            ]
        )
    ])

if __name__ == '__main__':
    generate_launch_description()

