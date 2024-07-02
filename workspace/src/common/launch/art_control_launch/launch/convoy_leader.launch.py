import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PythonExpression

def generate_launch_description():
    # Declare the launch argument
    leader_ns = LaunchConfiguration('leader_ns')
    robot_ns = LaunchConfiguration('robot_ns')


    return LaunchDescription([
        DeclareLaunchArgument(
            'leader_ns',
            default_value='',
            description='Namespace for leader vehicle'
        ),

        # Conditionally launch the leader vehicle node
        GroupAction(
            condition=IfCondition(PythonExpression([
                "'", leader_ns, "'",
                " != 'none'",
            ])),

            actions=[
                Node(
                    package='convoy_controller',  # Replace with your package name
                    executable='follower',  # Replace with your node executable name if different
                    name='follower_control_node',
                    namespace=leader_ns,
                    remappings=[
                        ('/leader_vehicle/odometry/filtered', PythonExpression(['"', '/', leader_ns, "/odometry/filtered", '"'])),
                        ('/follower_vehicle/odometry/filtered', PythonExpression(['"', '/', robot_ns, "/odometry/filtered", '"'])),
                        ('/follower_vehicle/driver_inputs',  PythonExpression(['"', '/', robot_ns, "/control/vehicle_inputs", '"'])),
                        ('/vehicle_traj', PythonExpression(['"', '/', leader_ns, "/vehicle_traj", '"']))

                    ],
                    output='screen'
                )
            ]
        ),

        # Conditionally launch the follower vehicle node
        GroupAction(
            condition=IfCondition(PythonExpression([
                "'", leader_ns, "'",
                " == 'none'",
            ])),
            actions=[
                Node(
                    package='convoy_controller',  # Replace with your package name
                    executable='leader',    # Replace with your node executable name if different
                    name='leader_control_node',
                    remappings=[
                        ('/leader_vehicle/cmd_vel', PythonExpression(['"', '/', robot_ns, "/cmd_vel", '"'])),
                        ('/leader_vehicle/odometry/filtered', PythonExpression(['"', '/', robot_ns, "/odometry/filtered", '"'])),
                        ('/leader_vehicle/control/vehicle_inputs', PythonExpression(['"', '/', robot_ns, "/control/vehicle_inputs", '"'])),
                        ('/leader_vehicle/vehicle_traj', PythonExpression(['"', '/', robot_ns, "/vehicle_traj", '"']))
                    ],
                    output='screen'
                )
            ]
        )
    ])

if __name__ == '__main__':
    generate_launch_description()

