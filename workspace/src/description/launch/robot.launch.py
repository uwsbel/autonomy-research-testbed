from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, PythonExpression

def generate_launch_description():

    robot_ns = LaunchConfiguration('robot_ns')

    urdf_file = os.path.join(
        get_package_share_directory('description'),
        'urdf',
        'robot.urdf'
    )

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{ 'robot_description': open(urdf_file).read(),
                          'frame_prefix': PythonExpression(['"', '/', robot_ns, "/", '"']),
                        }]

        ),
        Node(
            package='description',
            executable='joint_state_publisher.py',
            name='joint_state_publisher',
            output='screen'
        ),
    ])
