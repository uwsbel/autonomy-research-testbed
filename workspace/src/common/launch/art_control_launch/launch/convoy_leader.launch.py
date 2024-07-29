import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PythonExpression

def generate_launch_description():
    # Declare the launch arguments
    leader_ns = LaunchConfiguration('leader_ns')
    robot_ns = LaunchConfiguration('robot_ns')

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
                    parameters=[
                        {"predefined_path": False}
                    ],
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
                    output='screen'
                ),
                Node(
                    package='convoy_controller',  # Replace with your package name
                    executable='control_mux',  # Replace with your node executable name if different
                    name='combined_input_node',
                    namespace=robot_ns,
                    remappings=[
                        ('/lateral_input', PythonExpression(['"', '/', robot_ns, "/input/lateral_input", '"'])),
                        ('/long_input', PythonExpression(['"', '/', robot_ns, "/input/long_input", '"'])),
                        ('/input/driver_inputs', PythonExpression(['"', '/', robot_ns, "/input/driver_inputs", '"']))
                    ],
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
                # Node(
                #     package='convoy_controller',
                #     executable='leader',  
                #     name='leader_control_node',
                #     namespace=robot_ns,
                #     remappings=[
                #         ('/leader_vehicle/cmd_vel', PythonExpression(['"', '/', robot_ns, "/cmd_vel", '"'])),
                #         ('/leader_vehicle/odometry/filtered', PythonExpression(['"', '/', robot_ns, "/odometry/filtered", '"'])),
                #         ('/leader_vehicle/control/vehicle_inputs', PythonExpression(['"', '/', robot_ns, "/input/lateral_input", '"'])),
                #         ('/leader_vehicle/vehicle_traj', PythonExpression(['"', '/', robot_ns, "/vehicle_traj", '"']))
                #     ],
                #     output='screen'
                # ),
                Node(
                    package='convoy_controller',  
                    executable='velocity',  
                    name='velocity_controller',
                    namespace=robot_ns,
                    remappings=[
                        ('/odometry/filtered', PythonExpression(['"', '/', robot_ns, "/odometry/filtered", '"'])),
                        ('/driver_inputs', PythonExpression(['"', '/', robot_ns, "/input/long_input", '"']))
                    ],
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
                    output='screen'
                )
            ]
        )
    ])

if __name__ == '__main__':
    generate_launch_description()
