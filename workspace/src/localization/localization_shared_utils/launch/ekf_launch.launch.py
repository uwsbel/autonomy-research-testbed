from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    

     # Transforms
    gps_transform = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='gps_transform',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'gps']
    )
    imu_transform = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='imu_transform',
        arguments=['0', '0', '0', '3.1416', '-1.5708', '0', 'base_link', 'imu']
    )

    # Navsat Transform
    navsat_transform_node = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform_node',
        respawn=True,
        parameters=[{
            'magnetic_declination_radians': 0.0,
            'yaw_offset': 0.0,
            'zero_altitude': True,
            'use_odometry_yaw': True,
            'wait_for_datum': False,
            'publish_filtered_gps': True,
            'broadcast_cartesian_transform':False,
        }],
        remappings=[
            ('/imu/data', '/imu'),
            ('/gps/fix', '/fix'),
           #('/odometry/filtered', '/odometry/gps')
        ]
       
    )

    

   
    ekf_filter_node = Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=['/home/art/art/workspace/src/localization/localization_shared_utils/config/ekf_config.yaml'],

        )

    return LaunchDescription([
        gps_transform,
        imu_transform,
        navsat_transform_node,
        ekf_filter_node
    ])
    

    
