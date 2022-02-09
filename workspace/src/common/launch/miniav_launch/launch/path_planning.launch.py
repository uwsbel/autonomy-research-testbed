from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

def generate_launch_description():

    DeclareLaunchArgument("vis", default_value=TextSubstitution(text="False"))
    DeclareLaunchArgument("lookahead", default_value=TextSubstitution(text=".75"))

    node = Node(
            package='path_planning',
            namespace='',
            executable='path_planning',
            name='path_planning',
            parameters=[
                {"vis": LaunchConfiguration("vis",default=False)},
                {"lookahead": LaunchConfiguration("lookahead",default=.75)}
            ]
        )
    return  LaunchDescription([
        node
    ])