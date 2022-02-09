from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node


def generate_launch_description():

    DeclareLaunchArgument("use_sim_time", default_value=TextSubstitution(text="False"))
    DeclareLaunchArgument("model", default_value=TextSubstitution(text="data/model_refined"))
    DeclareLaunchArgument("camera_calibration_file", default_value=TextSubstitution(text="data/calibration.json"))
    DeclareLaunchArgument("vis", default_value=TextSubstitution(text="False"))

    node = Node(
            package='cone_detector',
            namespace='',
            executable='object_recognition',
            name='object_recognition',
            parameters=[
                 {"use_sim_time": LaunchConfiguration("use_sim_time",default=False)},
                 {"model":LaunchConfiguration("model",default="data/model_refined")},
                 {"camera_calibration_file":LaunchConfiguration("camera_calibration_file",default="data/calibration.json")},
                 {"vis": LaunchConfiguration("vis",default=False)}
            ]
        )
    return  LaunchDescription([
        node
    ])