import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PythonExpression, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    leader_ns = LaunchConfiguration('leader_ns')
    robot_ns = LaunchConfiguration('robot_ns')
    veh_config = LaunchConfiguration('veh_config_file')
    package_share_directory = FindPackageShare('art_sensing_launch').find('art_sensing_launch')
    veh_config_file_path = PathJoinSubstitution([package_share_directory, 'config', veh_config])

    common_nodes = [

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
            executable='mpc',
            name='mpc_node',
            namespace=robot_ns,
            remappings=[
                # ('/leader_vehicle/odometry/filtered', PythonExpression(['"', '/', leader_ns, "/odometry/filtered", '"'])),
                ('/odometry/filtered', PythonExpression(['"', '/', robot_ns, "/odometry/filtered", '"'])),
                ('/input/driver_inputs', PythonExpression(['"', '/', robot_ns, "/control/vehicle_inputs", '"'])),
                # ('/cmd_vel', PythonExpression(['"', '/', robot_ns, "/cmd_vel", '"'])),
                ('/local_mpc', PythonExpression(['"', '/', robot_ns, "/local_mpc", '"'])),
                #('/path', PythonExpression(['"', '/', leader_ns, "/vehicle_traj", '"'])),
                # ('/leader_vehicle/vehicle_traj', PythonExpression(['"', '/', leader_ns, "/vehicle_traj", '"'])),
                # ('/follower_vehicle/vehicle_traj', PythonExpression(['"', '/', robot_ns, "/vehicle_traj", '"']))
            ],
            parameters=[veh_config_file_path,{ 'leader_ns': PythonExpression(['"', leader_ns, '"']) }],
            output='screen'
        ),

        # Node(
        #     package='convoy_controller',
        #     executable='velocity',
        #     name='velocity_controller',
        #     namespace=robot_ns,
        #     remappings=[
        #         ('/cmd_vel', PythonExpression(['"', '/', robot_ns, "/cmd_vel", '"'])),
        #         ('/input/driver_inputs', PythonExpression(['"', '/', robot_ns, "/control/vehicle_inputs", '"'])),
        #         ('/odometry/filtered', PythonExpression(['"', '/', robot_ns, "/odometry/filtered", '"']))
        #     ],
        #     parameters=[veh_config_file_path, { 'leader_ns': PythonExpression(['"', leader_ns, '"']) }],
        #     output='screen'
        # ),

    ]

    follower_nodes = [
    ]

    return LaunchDescription([
        DeclareLaunchArgument('leader_ns', default_value='', description='Namespace for leader vehicle'),
        DeclareLaunchArgument('robot_ns', default_value='', description='Namespace for robot vehicle'),
        DeclareLaunchArgument('veh_config_file', default_value='default_veh_config.yaml', description='Vehicle configuration file'),

        GroupAction(
            condition=IfCondition(PythonExpression(["'", leader_ns, "' != 'none'"])),
            actions=follower_nodes + common_nodes
        ),
        GroupAction(
            condition=IfCondition(PythonExpression(["'", leader_ns, "' == 'none'"])),
            actions=common_nodes
        )
    ])

if __name__ == '__main__':
    generate_launch_description()
